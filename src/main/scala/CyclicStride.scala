/* ---------------------------------------------------------------------
Cyclic Stride Prefetcher (CSP)

Novel mechanism: for each candidate lag L ∈ {1…pMax}, a saturating
match counter is incremented whenever the current inter-miss block
delta equals the delta from exactly L snoops ago (a hardware
autocorrelation test).  The smallest L whose counter saturates is
committed as the active period P, and the most-recent P deltas are
frozen as a cyclic template.

Subsequent snoops validate the template at the tracked phase position.
On a match the phase advances and a new prefetch burst walks the
template forward `degree` steps.  Consecutive mismatches beyond the
mismatch tolerance invalidate the template and restart learning.

Targets:
  - Array-of-Structs traversal (period = fields accessed per element)
  - Tiled / blocked matrix access (period = tile width in blocks)
  - CSR sparse-matrix row iteration (period = average nnz per row)
  - Any workload where inter-miss deltas repeat with a fixed period P

Period-1 operation subsumes classic constant-stride prefetching.
--------------------------------------------------------------------- */

package barf

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.subsystem.CacheBlockBytes

case class CyclicStridePrefetcherParams(
  pMax:        Int = 8,   // maximum detectable period
  historyLen:  Int = 16,  // delta history depth; must be >= 2 * pMax
  matchThresh: Int = 4,   // lag-match score needed to commit a period
  mismatchTol: Int = 2,   // consecutive mismatches before invalidating
  degree:      Int = 4,   // prefetch lookahead (steps in the template)
  deltaBits:   Int = 12   // bits per delta (covers ±2K cache blocks)
) extends CanInstantiatePrefetcher {
  require(historyLen >= 2 * pMax,
    s"historyLen ($historyLen) must be >= 2 * pMax (${2 * pMax})")
  require(pMax >= 1 && matchThresh >= 1 && mismatchTol >= 1 && degree >= 1)

  def desc = "Cyclic Stride Prefetcher"
  def instantiate()(implicit p: Parameters) =
    Module(new CyclicStridePrefetcher(this)(p))
}

class CyclicStridePrefetcher(params: CyclicStridePrefetcherParams)(implicit p: Parameters)
    extends AbstractPrefetcher()(p) {

  val blockBits  = log2Up(p(CacheBlockBytes))
  val scoreBits  = log2Ceil(params.matchThresh + 1)
  val pBits      = log2Ceil(params.pMax + 1)          // holds 0..pMax
  val degreeBits = log2Ceil(params.degree + 1)

  // ── Delta history ─────────────────────────────────────────────────────────
  // Shift register; index 0 = most-recent past delta (before this snoop).
  // Initialised to zero so the first few comparisons are benign.
  val deltaHist = RegInit(VecInit(Seq.fill(params.historyLen)(0.U(params.deltaBits.W))))

  // ── Lag-match scores ──────────────────────────────────────────────────────
  // matchScore(i) counts how often curDelta == deltaHist(i), i.e. lag (i+1).
  val matchScore = RegInit(VecInit(Seq.fill(params.pMax)(0.U(scoreBits.W))))

  // ── Period state ──────────────────────────────────────────────────────────
  // activePeriod == 0  →  learning mode
  // activePeriod == P  →  cyclic mode with committed period P
  val activePeriod = RegInit(0.U(pBits.W))
  val template     = Reg(Vec(params.pMax, UInt(params.deltaBits.W)))
  val phase        = RegInit(0.U(pBits.W))   // current position in [0, P)
  val mismatchCnt  = RegInit(0.U(log2Ceil(params.mismatchTol + 1).W))

  // ── Address bookkeeping ───────────────────────────────────────────────────
  val lastBlock = Reg(UInt())
  val everSeen  = RegInit(false.B)

  // ── Prefetch burst state ──────────────────────────────────────────────────
  val prefBase   = Reg(UInt())
  val prefPhase  = RegInit(0.U(pBits.W))
  val prefRemain = RegInit(0.U(degreeBits.W))
  val prefWrite  = Reg(Bool())

  // ── Combinational: delta for the current snoop ────────────────────────────
  // UInt subtraction is modular; taking the lower deltaBits bits gives the
  // correct 2's-complement delta for strides that fit within the range.
  val curDelta = (io.snoop.bits.block - lastBlock)(params.deltaBits - 1, 0)

  // ──────────────────────────────────────────────────────────────────────────
  // Prefetch output
  //
  // request.fire is handled BEFORE the snoop block so that any snoop in the
  // same cycle wins via Chisel's last-connect semantics, cleanly restarting
  // the burst from the new snoop position.
  // ──────────────────────────────────────────────────────────────────────────

  when (io.request.fire) {
    prefBase   := prefBase + template(prefPhase)
    prefPhase  := Mux(prefPhase === activePeriod - 1.U, 0.U, prefPhase + 1.U)
    prefRemain := prefRemain - 1.U
  }

  io.request.valid        := (prefRemain > 0.U) && (activePeriod =/= 0.U)
  io.request.bits.address := (prefBase + template(prefPhase)) << blockBits
  io.request.bits.write   := prefWrite

  // ──────────────────────────────────────────────────────────────────────────
  // Snoop processing
  // ──────────────────────────────────────────────────────────────────────────

  when (io.snoop.valid) {

    // First snoop: record block address and wait for the second to get a delta.
    when (everSeen) {

      // Shift curDelta into the history.  All reads use current-cycle values
      // (Chisel register reads happen before writes), so this is a correct
      // parallel shift despite the sequential syntax.
      for (i <- params.historyLen - 1 until 0 by -1) {
        deltaHist(i) := deltaHist(i - 1)
      }
      deltaHist(0) := curDelta

      // ── Learning: autocorrelation period detection ─────────────────────
      when (activePeriod === 0.U) {

        // For each lag candidate, update its saturating match counter.
        // deltaHist(lag) holds the delta (lag+1) snoops ago — read from the
        // pre-shift register values — so the comparison is valid.
        val nextScore = Wire(Vec(params.pMax, UInt(scoreBits.W)))
        for (lag <- 0 until params.pMax) {
          val isMatch = curDelta === deltaHist(lag)
          nextScore(lag) := Mux(isMatch,
            Mux(matchScore(lag) < params.matchThresh.U,
              matchScore(lag) + 1.U, matchScore(lag)),
            Mux(matchScore(lag) > 0.U,
              matchScore(lag) - 1.U, 0.U))
          matchScore(lag) := nextScore(lag)
        }

        // Commit the smallest-lag (most precise) period that just saturated.
        val saturated    = VecInit(nextScore.map(_ === params.matchThresh.U))
        val anySaturated = saturated.reduce(_ || _)

        when (anySaturated) {
          // bestIdx is 0-based; committed period = bestIdx + 1
          val bestIdx = PriorityEncoder(saturated.asUInt)

          activePeriod := bestIdx + 1.U
          phase        := 0.U
          mismatchCnt  := 0.U
          for (i <- 0 until params.pMax) { matchScore(i) := 0.U }

          // Build template in chronological cycle order:
          //   template(0)       = deltaHist(bestIdx - 1)  [oldest of the P deltas]
          //   template(k)       = deltaHist(bestIdx-1-k)  [for k < bestIdx]
          //   template(bestIdx) = curDelta                 [most recent]
          //
          // MuxLookup handles the dynamic index over the history array.
          for (k <- 0 until params.pMax) {
            when (k.U === bestIdx) {
              template(k) := curDelta
            }.elsewhen (k.U < bestIdx) {
              template(k) := MuxLookup(bestIdx - 1.U - k.U, deltaHist(0),
                (0 until params.historyLen).map(i => i.U -> deltaHist(i)))
            }
          }

          // Immediately kick off a prefetch burst from the current snoop
          // position.  Since activePeriod is still 0 in this cycle (register
          // write takes effect next cycle), the request.valid gate won't open
          // until the next cycle — by which time all registers have committed.
          prefBase   := io.snoop.bits.block
          prefPhase  := 0.U
          prefRemain := params.degree.U
          prefWrite  := io.snoop.bits.write
        }
      } // learning

      // ── Active: phase tracking and mismatch detection ──────────────────
      // This block is guarded by the CURRENT register value of activePeriod,
      // so it does not fire in the same cycle that learning commits a period.
      when (activePeriod =/= 0.U) {
        val nextPhase = Mux(phase === activePeriod - 1.U, 0.U, phase + 1.U)

        when (curDelta === template(phase)) {
          // Delta matches the expected template slot: advance phase and
          // restart the prefetch burst from this snoop's block address.
          phase       := nextPhase
          mismatchCnt := 0.U
          prefBase    := io.snoop.bits.block
          prefPhase   := nextPhase
          prefRemain  := params.degree.U
          prefWrite   := io.snoop.bits.write
        }.otherwise {
          // Mismatch: the access pattern has changed or there is noise.
          val newMismatch = mismatchCnt + 1.U
          mismatchCnt := newMismatch
          when (newMismatch >= params.mismatchTol.U) {
            // Invalidate the learned period and return to learning.
            activePeriod := 0.U
            phase        := 0.U
            mismatchCnt  := 0.U
            prefRemain   := 0.U   // cancel the in-flight burst
            for (i <- 0 until params.pMax) { matchScore(i) := 0.U }
          }
        }
      } // active

    } // everSeen

    lastBlock := io.snoop.bits.block
    everSeen  := true.B

  } // io.snoop.valid
}
