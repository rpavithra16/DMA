package project_dma;
import FIFO::*;
import FIFOF::*;
import BRAMFIFO::*;
import DefaultValue::*;
import AXI4_Types::*;
import AXI4_Lite_Types::*;
import AXI4_Fabric::*;
import Semi_FIFOF::*;
import SpecialFIFOs::*;
import BUtils::*;
import RegFile::*;

// DMA Control register addresses
typedef enum {
    ADDR_SRC   = 0,
    ADDR_DST   = 1,
    ADDR_COUNT = 2,
    ADDR_START = 3,
    ADDR_DONE  = 4,
    ADDR_HALF_DONE = 5,
    ADDR_ERR = 6
} RegAddr deriving (Bits, Eq);

typedef Bit#(3) AxiAddr;//To access control regs
typedef Bit#(32) AxiData;
 typedef enum {IDLE, READ, WRITE, DONE} State deriving (Bits, Eq);//FSM States
 
interface SimpleDMA_IFC;
    method Action write(AxiAddr addr, AxiData data);//write data to config regs
    method ActionValue#(AxiData) read(AxiAddr addr);
    interface AXI4_Master_IFC#(32,32,0) master;
    method Bool irq_done;
    method Bool irq_half_done;
    method Bool irq_error;
endinterface

(* descending_urgency = "write, start_transfer" *)
(* descending_urgency = "dma_error, monitor_timeout" *)
(* descending_urgency = "start_transfer, dma_error" *)
(* descending_urgency = "dma_error, read_word" *)
(* descending_urgency = "dma_error, write_word" *)
(* descending_urgency = "capture_read_response, write_word" *)


(* synthesize *)
module mkDma(SimpleDMA_IFC);
     AXI4_Master_Xactor_IFC #(32,32,0) m_xactor <- mkAXI4_Master_Xactor;
     FIFOF#(Bit#(32)) fifo <- mkSizedBRAMFIFOF(16);//fifo depth=16, stores 32bit values
     
    // Control registers
    Reg#(Bit#(32)) reg_src   <- mkReg(0);
    Reg#(Bit#(32)) reg_dst   <- mkReg(0);
    Reg#(Bit#(32)) reg_count <- mkReg(0);
    Reg#(Bool) reg_start <- mkReg(False);
    Reg#(Bool)reg_done  <- mkReg(False);
    Reg#(Bool)reg_half_done <- mkReg(False);
    Reg#(Bool)reg_error <- mkReg(False);
    Reg#(Bit#(16)) timeout <- mkReg(0);

    // Internal state
    Reg#(Bit#(32)) words_left <- mkReg(0);
    Reg#(Bit#(32)) words_written <- mkReg(0);
    Reg#(Bit#(32)) curr_src   <- mkReg(0);
    Reg#(Bit#(32)) curr_dst   <- mkReg(0);
    Reg#(Bit#(5)) batch_count <- mkReg(0); // max 16 per batch
    Reg#(Bit#(5)) batch_written <- mkReg(0);
    Reg#(State) state <- mkReg(IDLE);

    rule start_transfer(reg_start && state == IDLE);//FSM Idle state - initialise internal state regs
    $display("FSM: START transfer src=%0d dst=%0d count=%0d", reg_src, reg_dst, reg_count);
        curr_src   <= reg_src;
        curr_dst   <= reg_dst;
        words_left <= reg_count;
        reg_done   <= False;
        reg_half_done <= False;
        state      <= READ;
        reg_start  <= False;
        timeout    <= 0;
        reg_error  <= False;
    endrule

 rule read_word (state == READ && words_left > 0 && fifo.notFull);//FSM Read state
 $display("FSM: Read");
    batch_written <=0;
    let lv_araddr     = curr_src;
    let lv_burst      = 0;     // Single transfer

    let read_request = AXI4_Rd_Addr {
    araddr: lv_araddr,
    arid: 0,
    arlen: lv_burst,
    arsize: 3'b010,
    arburst: 2'b01,
    aruser: 0,
    arprot: 0
};
    m_xactor.i_rd_addr.enq(read_request);
    batch_count <= batch_count + 1;
    curr_src <= curr_src + 4;  // AXI addresses are byte-based
    if (batch_count == 15 || words_left == 1) begin
        state <= WRITE;
    end
    words_left <= words_left - 1;//decrement words left
endrule

rule capture_read_response (m_xactor.o_rd_data.notEmpty && fifo.notFull);
   let rd_data = m_xactor.o_rd_data.first;
   m_xactor.o_rd_data.deq;
   fifo.enq(rd_data.rdata);
   $display("FSM: Enqueued AXI read data %h to FIFO", rd_data.rdata);
endrule


   rule write_word (state == WRITE && fifo.notEmpty() && m_xactor.i_wr_addr.notFull && m_xactor.i_wr_data.notFull);//FSM Write state
    batch_count <=0;
    let word = fifo.first;//extract word frm fifo
    $display("WRITE word %h to dst[%0d]", word, curr_dst);
    fifo.deq();
    batch_written <= batch_written + 1;
    let lv_data       = curr_dst;
    let lv_burst_len  = 0;               // single beat
    //let chanNum       = 0;
    let lv_last       = True;           // single transfer = last

    let write_data = AXI4_Wr_Data {
        wdata: word,
        wstrb: '1,         // assume writing full 4 bytes (Bit#(4)'1)
        wlast: lv_last,
        wid:   0
    };

    let write_addr = AXI4_Wr_Addr {
        awaddr: lv_data,
        awuser: 0,
        awlen:  0,
        awsize: 3'b010,//4 bytes
        awburst: 2'b01,//INCR
        awid: 0,
        awprot: 0
    };


    m_xactor.i_wr_addr.enq(write_addr);
    m_xactor.i_wr_data.enq(write_data);

    words_written <= words_written + 1;
    curr_dst <= curr_dst + 4;
   
    if (words_written+1==reg_count/2)
        reg_half_done<=True;

    if (words_written == reg_count - 1)
        state <= DONE;
   
    else if (batch_written==15) begin
        state <= READ;
    end
    endrule    

    rule finish_transfer (state == DONE);
    $display("FSM Done");
        reg_done <= True;
        state    <= IDLE;
    endrule
   
    rule monitor_timeout (state != IDLE);
    timeout <= timeout + 1;
    endrule
   
    rule dma_error(timeout>1000);
    $display("** DMA ERROR: Transfer stuck or timed out");
    reg_error <= True;
    state <= IDLE;
    endrule

    method Action write(AxiAddr addr, AxiData data);//to write data to config reg
    RegAddr raddr = unpack(addr);
            case (raddr)
                ADDR_SRC:   reg_src   <= data;
                ADDR_DST:   reg_dst   <= data;
                ADDR_COUNT: reg_count <= data;
                ADDR_START: reg_start<=True;
                default: noAction;
            endcase
           
    endmethod


        method ActionValue#(AxiData) read(AxiAddr addr);//read data from config reg
            RegAddr raddr = unpack(addr);
            case (raddr)
               ADDR_SRC:   return reg_src;
               ADDR_DST:   return reg_dst;
               ADDR_COUNT: return reg_count;
               ADDR_START:return 0;
               ADDR_DONE:  return reg_done  ? 1 : 0;
               ADDR_HALF_DONE: return reg_half_done ? 1 : 0;
               ADDR_ERR: return reg_error ? 1 : 0;
               default:    return 0;
            endcase
        endmethod

    interface master= m_xactor.axi_side;
    method Bool irq_done; return reg_done; endmethod
    method Bool irq_half_done; return reg_half_done; endmethod
    method Bool irq_error; return reg_error; endmethod
endmodule

interface Ifc_DMA_AXI4_Lite;
interface AXI4_Master_IFC#(32,32,0) master;
  interface AXI4_Lite_Slave_IFC#(32,32,0) slave;
endinterface

module mkDMA_AXI4_Lite(Ifc_DMA_AXI4_Lite);
SimpleDMA_IFC dma <- mkDma;
AXI4_Lite_Slave_Xactor_IFC#(32,32,0)  s_xactor <- mkAXI4_Lite_Slave_Xactor();

rule axi_read_req;
let req <- pop_o(s_xactor.o_rd_addr);
let data <- dma.read(truncate(req.araddr >> 2));

let r = AXI4_Lite_Rd_Data {rresp:AXI4_LITE_OKAY, rdata: data, ruser: 0};
s_xactor.i_rd_data.enq(r);
endrule

rule axi_write_req;
let aw <- pop_o(s_xactor.o_wr_addr);
let w <- pop_o(s_xactor.o_wr_data);
dma.write(truncate(aw.awaddr >> 2),w.wdata);

let r = AXI4_Lite_Wr_Resp {bresp: AXI4_LITE_OKAY, buser: 0 };
s_xactor.i_wr_resp.enq (r);
endrule

      interface master= dma.master;
               interface slave= s_xactor.axi_side;

endmodule
endpackage
