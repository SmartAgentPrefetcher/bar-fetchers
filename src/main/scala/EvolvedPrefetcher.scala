package barf

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.subsystem.CacheBlockBytes

case class EvolvedPrefetcherParams(
  nEntries:        Int = 32,   // page-indexed table entries (power of 2)
  nDeltas:         Int = 8,    // delta slots per page entry
  confidenceMax:   Int = 15,   // saturating confidence ceiling (4 bits)
  confidenceThresh:Int = 6,    // minimum confidence to issue a prefetch
  deltaBits:       Int = 12,   // bits per delta (covers ±2K cache blocks)
  degree:          Int = 4     // max prefetches issued per snoop
) extends CanInstantiatePrefetcher {
  require(nEntries > 0 && (nEntries & (nEntries - 1)) == 0,
    "nEntries must be a power of 2")
  require(nDeltas >= 1 && degree >= 1)

  def desc = "Evolved Prefetcher"
  def instantiate()(implicit p: Parameters) =
    Module(new EvolvedPrefetcher(this)(p))
}

class EvolvedPrefetcher(params: EvolvedPrefetcherParams)(implicit p: Parameters)
    extends AbstractPrefetcher()(p) {

  val blockBits      = log2Up(p(CacheBlockBytes))
  val confBits       = log2Ceil(params.confidenceMax + 1)
  val idxBits        = log2Ceil(params.nEntries)
  val deltaSlotBits  = log2Ceil(params.nDeltas).max(1)
  val addrBlockBits  = 48
  // Block-address bits indexing within a 4 KB page (6 for 64 B lines)
  val pageOffsetBits = 12 - blockBits
  val tagBits        = 32

  // ---- Page-indexed table ------------------------------------------------
  val entryValid = RegInit(VecInit(Seq.fill(params.nEntries)(false.B)))
  val entryTag   = Reg(Vec(params.nEntries, UInt(tagBits.W)))
  val lastBlock  = Reg(Vec(params.nEntries, UInt(addrBlockBits.W)))
  val hasLast    = RegInit(VecInit(Seq.fill(params.nEntries)(false.B)))

  // ---- Per-entry delta sub-table -----------------------------------------
  val dValid = RegInit(VecInit(Seq.fill(params.nEntries)(
    VecInit(Seq.fill(params.nDeltas)(false.B)))))
  val dDelta = Reg(Vec(params.nEntries, Vec(params.nDeltas,
    UInt(params.deltaBits.W))))
  val dConf  = RegInit(VecInit(Seq.fill(params.nEntries)(
    VecInit(Seq.fill(params.nDeltas)(0.U(confBits.W))))))

  // ---- Prefetch scan state -----------------------------------------------
  // After a snoop we iterate through the delta sub-table of the matching
  // entry, issuing one prefetch per cycle for each qualifying delta.
  val pfEntry   = Reg(UInt(idxBits.W))
  val pfBase    = Reg(UInt(addrBlockBits.W))
  val pfScanIdx = RegInit(params.nDeltas.U((log2Ceil(params.nDeltas + 1)).W))
  val pfRemain  = RegInit(0.U(log2Ceil(params.degree + 1).max(1).W))

  // ---- Snoop decode (combinational) --------------------------------------
  val snoopBlock = io.snoop.bits.block
  val snoopPage  = snoopBlock >> pageOffsetBits
  val pageIdx    = snoopPage(idxBits - 1, 0)
  // Pad the page address to ensure enough bits for the tag extraction.
  val snoopPageWide = snoopPage.pad(idxBits + tagBits)
  val pageTag       = snoopPageWide(idxBits + tagBits - 1, idxBits)

  // ==================================================================
  // Prefetch output — scan delta sub-table for high-confidence entries
  //
  // Processed BEFORE the snoop block so that a simultaneous snoop can
  // restart the scan via Chisel last-connection-wins semantics.
  // ==================================================================

  val scanQualify = Wire(Vec(params.nDeltas, Bool()))
  for (i <- 0 until params.nDeltas) {
    scanQualify(i) := dValid(pfEntry)(i) &&
      dConf(pfEntry)(i) >= params.confidenceThresh.U &&
      (i.U >= pfScanIdx)
  }
  val anyScan     = scanQualify.reduce(_ || _)
  val nextScanIdx = PriorityEncoder(scanQualify.asUInt)

  // Sign-extend the delta so that negative strides are added correctly.
  val pfDeltaRaw  = dDelta(pfEntry)(nextScanIdx)
  val pfDeltaSext = Cat(
    Fill(addrBlockBits - params.deltaBits, pfDeltaRaw(params.deltaBits - 1)),
    pfDeltaRaw)
  val pfAddrBlock = pfBase + pfDeltaSext

  when(io.request.fire) {
    pfScanIdx := nextScanIdx + 1.U
    pfRemain  := pfRemain - 1.U
  }

  io.request.valid        := (pfRemain > 0.U) && anyScan
  io.request.bits.address := pfAddrBlock(addrBlockBits - 1, 0) << blockBits
  io.request.bits.write   := false.B

  // ==================================================================
  // Snoop processing
  // ==================================================================

  when(io.snoop.valid) {
    val idx = pageIdx
    val tag = pageTag
    val tagMatch = entryValid(idx) && entryTag(idx) === tag

    // ---- Delta learning --------------------------------------------------
    when(tagMatch && hasLast(idx)) {
      val curDelta = (snoopBlock - lastBlock(idx))(params.deltaBits - 1, 0)
      val nonZero  = curDelta =/= 0.U

      when(nonZero) {
        // Search for a matching delta in this entry's sub-table.
        val matches  = VecInit((0 until params.nDeltas).map(i =>
          dValid(idx)(i) && dDelta(idx)(i) === curDelta))
        val anyMatch = matches.reduce(_ || _)
        val matchI   = PriorityEncoder(matches.asUInt)

        // Find the first free slot.
        val frees   = VecInit((0 until params.nDeltas).map(i =>
          !dValid(idx)(i)))
        val anyFree = frees.reduce(_ || _)
        val freeI   = PriorityEncoder(frees.asUInt)

        // Find the slot with minimum confidence (for replacement).
        val minInit = (params.confidenceMax.U(confBits.W),
                       0.U(deltaSlotBits.W))
        val (minC, minI) = (0 until params.nDeltas).foldLeft(minInit) {
          case ((bestC, bestI), i) =>
            val betterOrInvalid = dValid(idx)(i) && dConf(idx)(i) < bestC
            (Mux(betterOrInvalid, dConf(idx)(i), bestC),
             Mux(betterOrInvalid, i.U, bestI))
        }

        when(anyMatch) {
          // Existing delta — increment confidence (saturating).
          when(dConf(idx)(matchI) < params.confidenceMax.U) {
            dConf(idx)(matchI) := dConf(idx)(matchI) + 1.U
          }
        }.elsewhen(anyFree) {
          // New delta, free slot available — insert.
          dValid(idx)(freeI) := true.B
          dDelta(idx)(freeI) := curDelta
          dConf(idx)(freeI)  := 1.U
        }.otherwise {
          // All slots occupied.  Age the weakest entry; if it reaches
          // zero confidence, replace it with the new delta.
          when(minC <= 1.U) {
            dDelta(idx)(minI) := curDelta
            dConf(idx)(minI)  := 1.U
          }.otherwise {
            dConf(idx)(minI) := minC - 1.U
          }
        }
      }

      // ---- Prefetch generation (reads only) --------------------------------
      when(!io.snoop.bits.write) {
        pfEntry   := idx
        pfBase    := snoopBlock
        pfScanIdx := 0.U
        pfRemain  := params.degree.U
      }
    } // tagMatch && hasLast

    // ---- Table bookkeeping -----------------------------------------------
    when(tagMatch) {
      lastBlock(idx) := snoopBlock
      hasLast(idx)   := true.B
    }.otherwise {
      // Allocate (or replace) the entry for this page.
      entryValid(idx) := true.B
      entryTag(idx)   := tag
      lastBlock(idx)  := snoopBlock
      hasLast(idx)    := true.B
      // Clear the delta sub-table for the new page.
      for (i <- 0 until params.nDeltas) {
        dValid(idx)(i) := false.B
        dConf(idx)(i)  := 0.U
      }
    }
  } // io.snoop.valid
}
