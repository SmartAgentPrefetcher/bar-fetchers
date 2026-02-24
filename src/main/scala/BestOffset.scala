/* ---------------------------------------------------------------------
Best-Offset Prefetcher

Inspired by "Best-offset hardware prefetching" (Michaud, HPCA 2016).

Dynamically evaluates a set of candidate prefetch offsets against a
table of recent accesses, selecting whichever offset would have
predicted the most recent misses. Self-tuning and robust to noisy
or changing access patterns.
--------------------------------------------------------------------- */

package barf

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.subsystem.CacheBlockBytes

case class BestOffsetPrefetcherParams(
  rrEntries: Int = 64,
  scoreMax: Int = 31,
  roundMax: Int = 100,
  badScore: Int = 2,
  degree: Int = 2,
  offsets: Seq[Int] = (1 to 64)
) extends CanInstantiatePrefetcher {
  require((rrEntries & (rrEntries - 1)) == 0 && rrEntries > 0,
    "rrEntries must be a power of 2")
  require(offsets.nonEmpty, "offsets must not be empty")

  def desc = "Best-Offset Prefetcher"
  def instantiate()(implicit p: Parameters) =
    Module(new BestOffsetPrefetcher(this)(p))
}

class BestOffsetPrefetcher(params: BestOffsetPrefetcherParams)(implicit p: Parameters)
    extends AbstractPrefetcher()(p) {

  val blockBits    = log2Up(p(CacheBlockBytes))
  val nOffsets     = params.offsets.size
  val rrIdxBits    = log2Ceil(params.rrEntries)
  val scoreBits    = log2Ceil(params.scoreMax + 1)
  val offsetIdxBits = log2Ceil(nOffsets).max(1)

  // ---- Candidate offsets (in cache-block units) ----
  val offsetROM = VecInit(params.offsets.map(_.U))

  // ---- Recent Requests (RR) table ----
  // Direct-mapped; stores block addresses of recently seen accesses.
  val rrTable = Reg(Vec(params.rrEntries, UInt(48.W)))
  val rrValid = RegInit(VecInit(Seq.fill(params.rrEntries)(false.B)))

  // ---- Per-offset score table (saturating counters) ----
  val scores = RegInit(VecInit(Seq.fill(nOffsets)(0.U(scoreBits.W))))

  // ---- Round-robin test pointer & round counter ----
  val testPtr  = RegInit(0.U(offsetIdxBits.W))
  val roundCnt = RegInit(0.U(log2Ceil(params.roundMax + 1).max(1).W))

  // ---- Best offset selection ----
  val bestOffsetIdx   = RegInit(0.U(offsetIdxBits.W))
  val bestOffsetValid = RegInit(false.B)

  // Current snoop block address
  val snoopBlock = io.snoop.bits.block

  // RR table index: low bits of block address
  def rrIdx(block: UInt): UInt = block(rrIdxBits - 1, 0)

  // ==================================================================
  // Offset evaluation — one test per snoop
  // ==================================================================
  when(io.snoop.valid) {
    // 1. Record current access in RR table
    val wrIdx = rrIdx(snoopBlock)
    rrTable(wrIdx) := snoopBlock
    rrValid(wrIdx) := true.B

    // 2. Test one candidate: was (snoopBlock - offset) recently accessed?
    val testOffset = offsetROM(testPtr)
    val testBlock  = snoopBlock - testOffset
    val rdIdx      = rrIdx(testBlock)
    val rrHit      = rrValid(rdIdx) && rrTable(rdIdx) === testBlock

    // 3. Increment score on hit (saturating at scoreMax)
    when(rrHit && scores(testPtr) < params.scoreMax.U) {
      scores(testPtr) := scores(testPtr) + 1.U
    }

    // 4. Advance to next candidate offset
    testPtr  := Mux(testPtr === (nOffsets - 1).U, 0.U, testPtr + 1.U)
    roundCnt := roundCnt + 1.U

    // 5. End-of-round evaluation
    val anyMaxed = scores.map(_ >= params.scoreMax.U).reduce(_ || _)
    when(roundCnt >= (params.roundMax - 1).U || anyMaxed) {
      // Argmax over score table
      val (bestScore, bestIdx) = scores.zipWithIndex.tail.foldLeft(
        (scores(0), 0.U(offsetIdxBits.W))
      ) { case ((bScore, bIdx), (score, i)) =>
        val better = score > bScore
        (Mux(better, score, bScore), Mux(better, i.U, bIdx))
      }

      bestOffsetValid := bestScore >= params.badScore.U
      when(bestScore >= params.badScore.U) {
        bestOffsetIdx := bestIdx
      }

      // Reset scores and counter for next round
      for (i <- 0 until nOffsets) { scores(i) := 0.U }
      roundCnt := 0.U
    }
  }

  // ==================================================================
  // Prefetch generation
  // ==================================================================
  val prefBase  = Reg(UInt())
  val prefWrite = Reg(Bool())
  val prefCnt   = RegInit(params.degree.U(log2Ceil(params.degree + 1).max(1).W))

  // Handle request.fire first so the snoop restart (below) takes priority
  // via Chisel last-connection-wins semantics.
  when(io.request.fire) {
    prefCnt := prefCnt + 1.U
  }

  // On each snoop with a valid best offset, begin a new prefetch burst
  when(io.snoop.valid && bestOffsetValid) {
    prefBase  := snoopBlock
    prefWrite := io.snoop.bits.write
    prefCnt   := 0.U
  }

  val bestOffset = offsetROM(bestOffsetIdx)
  val prefActive = prefCnt < params.degree.U

  io.request.valid        := prefActive
  io.request.bits.address := (prefBase + (prefCnt +& 1.U) * bestOffset) << blockBits
  io.request.bits.write   := prefWrite
}
