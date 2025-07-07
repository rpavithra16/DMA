package dma;
import RegFile::*;
import FIFOF::*;
import BUtils::*;
import ConfigReg::*;
import DReg::*;
import Semi_FIFOF::*;  
/*
//Config Registers
typedef struct {
    Bit#(32) reg_src;
    Bit#(32) reg_dst;
    Bit#(32) reg_count;
    Bit#(1)  reg_start;
    Bit#(1)  reg_done;
} DMA_Config deriving (Bits, Eq);
*/

// DMA Control register addresses
typedef enum {
    ADDR_SRC   = 0,
    ADDR_DST   = 1,
    ADDR_COUNT = 2,
    ADDR_START = 3,
    ADDR_DONE  = 4
} RegAddr deriving (Bits, Eq);

typedef Bit#(3) AxiAddr;//To access control regs
typedef Bit#(32) AxiData;
 typedef enum {IDLE, READ, WRITE, DONE} State deriving (Bits, Eq);//FSM States
 
interface SimpleDMA_IFC;
    method Bit#(32) peekMem(Bit#(32) addr);//To read memory contents for my ref
    method Action startTransfer();//manual transfer and transfer done
    method Bool transferDone();
    method Action write(AxiAddr addr, AxiData data);//write data to config regs
    method ActionValue#(AxiData) read(AxiAddr addr);
endinterface

(* descending_urgency = "write, start_transfer" *)

(* synthesize *)
module mkDma(SimpleDMA_IFC);
     FIFOF#(Bit#(32)) fifo <- mkSizedFIFOF(16);//fifo depth=16, stores 32bit values 
     
    // Control registers
    Reg#(Bit#(32)) reg_src   <- mkReg(0);
    Reg#(Bit#(32)) reg_dst   <- mkReg(0);
    Reg#(Bit#(32)) reg_count <- mkReg(0);
    Reg#(Bool) reg_start <- mkReg(False);
    Reg#(Bool)reg_done  <- mkReg(False);

    // Internal state
    Reg#(Bit#(32)) words_left <- mkReg(0);
    Reg#(Bit#(32)) words_written <- mkReg(0);
    Reg#(Bit#(32)) curr_src   <- mkReg(0);
    Reg#(Bit#(32)) curr_dst   <- mkReg(0);

    Reg#(State) state <- mkReg(IDLE);


    //Replace with actual mem - using hex file for simulation
    RegFile#(Bit#(32), Bit#(32)) temp_mem_src <- mkRegFileLoad("input_data.hex",0,255);
    RegFile#(Bit#(32), Bit#(32)) temp_mem_dst <- mkRegFileLoad("output_data.hex",0,255);
    
    rule start_transfer(reg_start && state == IDLE);//FSM Idle state - initialise internal state regs
    $display("FSM: START transfer src=%0d dst=%0d count=%0d", reg_src, reg_dst, reg_count);
        curr_src   <= reg_src;
        curr_dst   <= reg_dst;
        words_left <= reg_count;
        reg_done   <= False;
        state      <= READ;
        reg_start<=False;
    endrule

 rule read_word (state == READ && words_left > 0 && fifo.notFull);//FSM Read state
 $display("FSM: Read");
    let word = temp_mem_src.sub(curr_src);//load word frm memory
    fifo.enq(word);//enqueue to fifo
    curr_src <= curr_src + 1;//increment source address for next read
    if (words_left == 1)
        state <= WRITE;
    words_left <= words_left - 1;//decrement words left
endrule

   rule write_word (state == WRITE && fifo.notEmpty());//FSM Write state
    let word = fifo.first;//extract word frm fifo
    $display("WRITE word %h to dst[%0d]", word, curr_dst);
    fifo.deq();
    temp_mem_dst.upd(curr_dst, word);//write word to destn
    words_written<=words_written+1;
    curr_dst <= curr_dst + 1;//increment dest address for next write
    if (words_written == reg_count-1)//when all words written
        state <= DONE;
endrule

    rule finish_transfer (state == DONE);
    $display("FSM Done");
        reg_done <= True;
        state    <= IDLE;
    endrule


    // Manual method to start transfer
    method Action startTransfer();
      reg_start<=True;
    endmethod

    // Manual method to check done
    method Bool transferDone();
        return reg_done;
    endmethod
    
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
               default:    return 0;
            endcase
        endmethod
        
    method Bit#(32) peekMem(Bit#(32) addr);
        return temp_mem_dst.sub(addr);
        
    endmethod
endmodule

endpackage
