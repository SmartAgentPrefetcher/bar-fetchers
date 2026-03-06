package barf

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.subsystem.{CacheBlockBytes}

// TileLink address width, set by TLPrefetcher from the bus edge parameters.
// Ensures all prefetchers produce the same port interface regardless of core type (Rocket/BOOM).
case object PrefetchAddressBits extends Field[Int](54)

trait CanInstantiatePrefetcher {
  def desc: String
  def instantiate()(implicit p: Parameters): AbstractPrefetcher
}

class Snoop(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)

  val write = Bool()
  val address = UInt(p(PrefetchAddressBits).W)
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}

class Prefetch(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)

  val write = Bool()
  val address = UInt(p(PrefetchAddressBits).W)
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}

class PrefetcherIO(implicit p: Parameters) extends Bundle {
  val snoop = Input(Valid(new Snoop))
  val request = Decoupled(new Prefetch)
  val hit = Output(Bool())
}

abstract class AbstractPrefetcher(implicit p: Parameters) extends Module {
  val io = IO(new PrefetcherIO)
  // Preserve all IO ports exactly as declared — required for PR port compatibility.
  dontTouch(io)

  io.request.valid := false.B
  io.request.bits := DontCare
  io.request.bits.address := 0.U(1.W)
  io.hit := false.B
}

case class NullPrefetcherParams() extends CanInstantiatePrefetcher {
  def desc() = "Null Prefetcher"
  def instantiate()(implicit p: Parameters) = Module(new NullPrefetcher()(p))
}

class NullPrefetcher(implicit p: Parameters) extends AbstractPrefetcher()(p)

