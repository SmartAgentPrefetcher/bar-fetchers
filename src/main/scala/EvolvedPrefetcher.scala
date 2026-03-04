package barf

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.subsystem.CacheBlockBytes

case class EvolvedPrefetcherParams(
) extends CanInstantiatePrefetcher {
  def desc = "Evolved Prefetcher"
  def instantiate()(implicit p: Parameters) =
    Module(new EvolvedPrefetcher(this)(p))
}

class EvolvedPrefetcher(params: EvolvedPrefetcherParams)(implicit p: Parameters)
    extends AbstractPrefetcher()(p) {

  val blockBits = log2Up(p(CacheBlockBytes))

  // TODO: implement prefetcher logic here
  // Inputs:  io.snoop.valid, io.snoop.bits.address, io.snoop.bits.write
  // Outputs: io.request (Decoupled) — set .valid and .bits.address / .bits.write
}
