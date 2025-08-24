//Error and timeout have been commented for now
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
typedef Bit#(32) RegAddr; 
    RegAddr aDDR_SRC   = 'h00061400;
    RegAddr aDDR_DST   = 'h00061404;
    RegAddr aDDR_COUNT = 'h00061408;
    RegAddr aDDR_START = 'h0006140C;
    RegAddr aDDR_DONE  = 'h00061410;
    RegAddr aDDR_HALF_DONE = 'h00061414;
    RegAddr aDDR_ERR = 'h00061418

typedef Bit#(32) AxiAddr;//To access control regs
typedef Bit#(32) AxiData;//Write src memory destn memory no of words start
 typedef enum {IDLE, READ, WRITE, DONE} State deriving (Bits, Eq);//FSM States
 
interface SimpleDMA_IFC;
    method Action write(AxiAddr addr, AxiData data);//write data to control regs
    method ActionValue#(AxiData) read(AxiAddr addr);//Return data when i read control reg 
    interface AXI4_Master_IFC#(32,32,0) master;
    method Bool irq_done;
    method Bool irq_half_done;
    //method Bool irq_error;
endinterface

(* descending_urgency = "write, start_transfer" *)
//(* descending_urgency = "dma_error, monitor_timeout" *)
//(* descending_urgency = "start_transfer, dma_error" *)
//(* descending_urgency = "dma_error, read_word" *)
//(* descending_urgency = "dma_error, write_word" *)
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
    //Reg#(Bool)reg_error <- mkReg(False);
    //Reg#(Bit#(16)) timeout <- mkReg(0);

    // Internal state
    Reg#(Bit#(32)) words_left <- mkReg(0);
    Reg#(Bit#(32)) words_written <- mkReg(0);
    Reg#(Bit#(32)) curr_src   <- mkReg(0);
    Reg#(Bit#(32)) curr_dst   <- mkReg(0);
    Reg#(Bit#(5)) batch_count <- mkReg(0);              // how many read requests we issued
    Reg#(Bit#(5)) batch_read_responses <- mkReg(0);     // how many read responses we received
    Reg#(Bit#(5)) batch_written <- mkReg(0);            // how many write ops we've issued
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

rule read_word (state == READ && words_left > 0 && fifo.notFull && m_xactor.i_rd_addr.notFull);
    $display("FSM: READ -> Issuing AXI read for addr %h", curr_src);

    let read_request = AXI4_Rd_Addr {
        araddr:  curr_src,
        arid:    0,
        arlen:   0,
        arsize:  3'b010,
        arburst: 2'b01,
        aruser:  0,
        arprot:  0
    };

    m_xactor.i_rd_addr.enq(read_request);
    curr_src <= curr_src + 4;
    words_left <= words_left - 1;
    batch_count <= batch_count + 1;

endrule

rule capture_read_response (state == READ && m_xactor.o_rd_data.notEmpty && fifo.notFull);
    let rd_data = m_xactor.o_rd_data.first;
    m_xactor.o_rd_data.deq;
    fifo.enq(rd_data.rdata);
    $display("FSM: READ RESPONSE -> Got data %h", rd_data.rdata);
    let next = batch_read_responses + 1;
    if (next == batch_count || words_left == 0) begin
        $display("FSM: READ BATCH DONE -> moving to WRITE");
        state <= WRITE;
        batch_count <= 0;
        batch_read_responses <= 0;
        batch_written <= 0; // reset write tracker
    end
    else begin 
        batch_read_responses <= next;
    end
endrule


   rule write_word (state == WRITE && fifo.notEmpty && m_xactor.i_wr_addr.notFull && m_xactor.i_wr_data.notFull);
    let word = fifo.first;
    fifo.deq;
    $display("FSM: WRITE -> Writing word %h to addr %h", word, curr_dst);
    let write_data = AXI4_Wr_Data {
        wdata: word,
        wstrb: '1,
        wlast: True,
        wid:   0
    };
    let write_addr = AXI4_Wr_Addr {
        awaddr:  curr_dst,
        awuser:  0,
        awlen:   0,
        awsize:  3'b010,
        awburst: 2'b01,
        awid:    0,
        awprot:  0
    };

    m_xactor.i_wr_addr.enq(write_addr);
    m_xactor.i_wr_data.enq(write_data);

    curr_dst <= curr_dst + 4;
    words_written <= words_written + 1;
    batch_written <= batch_written + 1;

    if (words_written == reg_count) begin
        $display("FSM: WRITE DONE -> transitioning to DONE");
        state <= DONE;
    end else if (batch_written == 16) begin
        $display("FSM: WRITE BATCH DONE -> back to READ");
        state <= READ;
    end
endrule 

rule consume_write_response (m_xactor.o_wr_resp.notEmpty);
    let resp = m_xactor.o_wr_resp.first;
    m_xactor.o_wr_resp.deq;
    $display("AXI: WRITE RESP -> bresp = %0d", resp.bresp);
endrule

    rule finish_transfer (state == DONE);
    $display("FSM Done");
        reg_done <= True;
        state    <= IDLE;
    endrule
/*    
    rule monitor_timeout (state != IDLE);
    timeout <= timeout + 1;
    endrule
    
    rule dma_error(timeout>1000);
    $display("** DMA ERROR: Transfer stuck or timed out");
    reg_error <= True;
    state <= IDLE;
    endrule
*/

    method Action write(AxiAddr addr, AxiData data);//to write data to config reg
    RegAddr raddr = unpack(addr);
            case (raddr)
                aDDR_SRC:   reg_src   <= data;
                aDDR_DST:   reg_dst   <= data;
                aDDR_COUNT: reg_count <= data;
                aDDR_START: reg_start<=True;
                default: noAction;
            endcase
            
    endmethod


        method ActionValue#(AxiData) read(AxiAddr addr);//read data from config reg
            RegAddr raddr = unpack(addr);
            case (raddr) 
               aDDR_SRC:   return reg_src; 
               aDDR_DST:   return reg_dst; 
               aDDR_COUNT: return reg_count;
               aDDR_START:return 0;
               aDDR_DONE:  return reg_done  ? 1 : 0;
               aDDR_HALF_DONE: return reg_half_done ? 1 : 0;
               //ADDR_ERR: return reg_error ? 1 : 0;
               default:    return 0;
            endcase
        endmethod

    interface master= m_xactor.axi_side;
    method Bool irq_done; return reg_done; endmethod
    method Bool irq_half_done; return reg_half_done; endmethod
    //method Bool irq_error; return reg_error; endmethod
endmodule

interface Ifc_DMA_AXI4_Lite;
	interface AXI4_Master_IFC#(32,32,0) master;
 	interface AXI4_Lite_Slave_IFC#(32,32,0) slave;
endinterface

(* synthesize *)
module mkDMA_AXI4_Lite(Ifc_DMA_AXI4_Lite);
		SimpleDMA_IFC dma <- mkDma;
		AXI4_Lite_Slave_Xactor_IFC#(32,32,0)  s_xactor <- mkAXI4_Lite_Slave_Xactor();

	 	rule axi_read_req;
	 		let req <- pop_o(s_xactor.o_rd_addr);
			//let data <- dma.read(truncate(req.araddr));
			let data <- dma.read(req.araddr);
	 	
	 		let r = AXI4_Lite_Rd_Data {rresp:AXI4_LITE_OKAY, rdata: data, ruser: 0};
	 		s_xactor.i_rd_data.enq(r);
		endrule
		
	 	rule axi_write_req;
	 		let aw <- pop_o(s_xactor.o_wr_addr);
	 		let w <- pop_o(s_xactor.o_wr_data);
	 		//dma.write(truncate(aw.awaddr),w.wdata);
		        dma.write(aw.awaddr,w.wdata);
	 		let r = AXI4_Lite_Wr_Resp {bresp: AXI4_LITE_OKAY, buser: 0 };
	 		s_xactor.i_wr_resp.enq (r);
	 	endrule

	       interface master= dma.master;
               interface slave= s_xactor.axi_side;
		
	endmodule
endpackage

