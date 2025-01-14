`include "machine.vh"
`include "rob.vh"
`include "uop.vh"

//`define FPGA64_32

module
`ifndef FPGA64_32
   core_l1d_l1i
`else
  core_l1d_l1i_64
`endif
    (		   
		   clk, 
		   reset,
		   syscall_emu,
		   took_exc,
		   paging_active,
		   page_table_root,
		   extern_irq,
		   in_flush_mode,
		   resume,
		   resume_pc,
		   ready_for_resume,
		   
		       mem_req_valid, 
		       mem_req_addr, 
		       mem_req_store_data,
		       mem_req_opcode,
		       mem_rsp_valid,
		       mem_rsp_load_data,
		       alloc_valid,
		       alloc_two_valid,
		       iq_one_valid,
		       iq_none_valid,
		       in_branch_recovery,
		       retire_reg_ptr,
		       retire_reg_data,
		       retire_reg_valid,
		       retire_reg_two_ptr,
		       retire_reg_two_data,
		       retire_reg_two_valid,
		       retire_valid,
		       retire_two_valid,
		   rob_empty,
		       retire_pc,
		       retire_two_pc,
		       branch_pc,
		       branch_pc_valid,		    
		       branch_fault,
		       l1i_cache_accesses,
		       l1i_cache_hits,
		       l1d_cache_accesses,
		       l1d_cache_hits,
		       l2_cache_accesses,
		       l2_cache_hits,
		       monitor_ack,
		       got_break,
		       got_ud,
		       got_bad_addr,
		       got_monitor,
		       inflight,
		       epc,
		   restart_ack);

   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);
   
   input logic clk;
   input logic reset;
   input logic syscall_emu;
   output logic	took_exc;
   output logic	paging_active;
   output logic	[63:0] page_table_root;
   
   input logic extern_irq;
   input logic resume;
   input logic [(`M_WIDTH-1):0] resume_pc;
   output logic 		in_flush_mode;
   output logic 		ready_for_resume;
   
   output logic [(`M_WIDTH-1):0] branch_pc;
   output logic 		 branch_pc_valid;
   output logic 		 branch_fault;

   output logic [63:0] 			l1i_cache_accesses;
   output logic [63:0] 			l1i_cache_hits;
   output logic [63:0] 			l1d_cache_accesses;
   output logic [63:0] 			l1d_cache_hits;
   output logic [63:0] 			l2_cache_accesses;
   output logic [63:0] 			l2_cache_hits;   

   
   /* mem port */
   output logic 			mem_req_valid;
   output logic [`M_WIDTH-1:0] 		mem_req_addr;
   output logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0] mem_req_store_data;
   output logic [3:0] 				 mem_req_opcode;
   
   input logic 					 mem_rsp_valid;
   input logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0]  mem_rsp_load_data;
   
   output logic					 alloc_valid;
   output logic					 alloc_two_valid;
   
   output logic					 iq_one_valid;
   output logic					 iq_none_valid;
   output logic					 in_branch_recovery;
   output logic [4:0] 			  retire_reg_ptr;
   output logic [`M_WIDTH-1:0] 		  retire_reg_data;
   output logic 			  retire_reg_valid;
   output logic [4:0]			  retire_reg_two_ptr;
   output logic [`M_WIDTH-1:0] 		  retire_reg_two_data;
   output logic 			  retire_reg_two_valid;
   output logic 			  retire_valid;
   output logic 			  retire_two_valid;
   output logic				  rob_empty;   
   output logic [(`M_WIDTH-1):0] 	  retire_pc;
   output logic [(`M_WIDTH-1):0] 	  retire_two_pc;
   input logic 				  monitor_ack;
   output logic 			  got_break;
   output logic 			  got_ud;
   output logic 			  got_bad_addr;
   output logic 			  got_monitor;
   
   output logic [`LG_ROB_ENTRIES:0] 	  inflight;
   output logic [`M_WIDTH-1:0] 		  epc;
   output logic				  restart_ack;

   logic [(`M_WIDTH-1):0] 	restart_pc;
   logic [(`M_WIDTH-1):0] 	restart_src_pc;
   logic 			restart_src_is_indirect;
   logic 			restart_valid;

   logic [`LG_PHT_SZ-1:0] 	branch_pht_idx;
   logic 			took_branch;

   logic [(`M_WIDTH-1):0] 	t_branch_pc;
   logic 			t_branch_pc_valid;
   logic 			t_branch_fault;
   
   assign branch_pc = t_branch_pc;
   assign branch_pc_valid = t_branch_pc_valid;   
   assign branch_fault = t_branch_fault;

   logic 				  retired_call;
   logic 				  retired_ret;

   logic 				  retired_rob_ptr_valid;
   logic 				  retired_rob_ptr_two_valid;
   logic [`LG_ROB_ENTRIES-1:0] 		  retired_rob_ptr;
   logic [`LG_ROB_ENTRIES-1:0] 		  retired_rob_ptr_two;
   

   logic 				  head_of_rob_ptr_valid;   
   logic [`LG_ROB_ENTRIES-1:0] 		  head_of_rob_ptr;

   wire 				  flush_req_l1i, flush_req_l1d;
   logic 				  flush_cl_req;
   logic [`M_WIDTH-1:0] 		  flush_cl_addr;
   wire 				  l1d_flush_complete;
   wire 				  l1i_flush_complete;

   
   mem_req_t core_mem_req;
   mem_rsp_t core_mem_rsp;
   mem_data_t core_store_data;
   
   logic 				  core_mem_req_valid;
   logic 				  core_mem_req_ack;
   logic 				  core_mem_rsp_valid;
   logic 				  core_store_data_valid;
   logic 				  core_store_data_ack;
   
   
   typedef enum logic [2:0] {
			     FLUSH_IDLE = 'd0,
			     WAIT_FOR_L1D_L1I = 'd1,
			     GOT_L1D = 'd2,
			     GOT_L1I = 'd3,
			     FLUSH_L2 = 'd4
   } flush_state_t;
   flush_state_t n_flush_state, r_flush_state;
   logic 	r_flush, n_flush;
   logic 	r_flush_l2, n_flush_l2;
   wire 	w_l2_flush_complete;
   logic 	memq_empty;   
   assign in_flush_mode = r_flush;


 
   always_ff@(posedge clk)
     begin
	if(reset)
	  begin
	     r_flush_state <= FLUSH_IDLE;
	     r_flush <= 1'b0;
	     r_flush_l2 <= 1'b0;
	  end
	else
	  begin
	     r_flush_state <= n_flush_state;
	     r_flush <= n_flush;
	     r_flush_l2 <= n_flush_l2;
	  end
     end // always_ff@ (posedge clk)


   //always_ff@(negedge clk)
     //begin
   //$display("r_flush_state = %d", r_flush_state);
   //end
	      
     always_comb
     begin
	n_flush_state = r_flush_state;
	n_flush = r_flush;
	n_flush_l2 = 1'b0;
	
	case(r_flush_state)
	  FLUSH_IDLE:
	    begin
	       if(flush_req_l1i && flush_req_l1d)
		 begin
		    n_flush_state = WAIT_FOR_L1D_L1I;
		    n_flush = 1'b1;
		 end
	       else if(flush_req_l1i && !flush_req_l1d)
		 begin
		    n_flush_state = GOT_L1D;
		    n_flush = 1'b1;
		 end
	       else if(!flush_req_l1i && flush_req_l1d)
		 begin
		    n_flush_state = GOT_L1I;
		    n_flush = 1'b1;
		 end
	    end
	  WAIT_FOR_L1D_L1I:
	    begin
	       if(l1d_flush_complete && !l1i_flush_complete)
		 begin
		    n_flush_state = GOT_L1D;		    
		 end
	       else if(!l1d_flush_complete && l1i_flush_complete)
		 begin
		    n_flush_state = GOT_L1I;
		 end
	       else if(l1d_flush_complete && l1i_flush_complete)
		 begin
		    $display("flush l2");
		    n_flush_state = FLUSH_L2;
		    n_flush_l2 = 1'b1;
		 end
	    end
	  GOT_L1D:
	    begin
	       if(l1i_flush_complete)
		 begin
		    $display("flush l2");
		    n_flush_state = FLUSH_L2;
		    n_flush_l2 = 1'b1;
		 end
	    end
	  GOT_L1I:
	    begin
	       if(l1d_flush_complete)
		 begin
		    $display("flush l2");
		    n_flush_state = FLUSH_L2;
		    n_flush_l2 = 1'b1;
		 end
	    end
	  FLUSH_L2:
	    begin
	       if(w_l2_flush_complete)
		 begin
		    $display("L2 FLUSH COMPLETE");
		    n_flush = 1'b0;
		    n_flush_state = FLUSH_IDLE;
		 end
	    end
	  default:
	    begin
	    end
	endcase // case (r_flush_state)
     end // always_comb
   

   wire 	l1d_mem_rsp_valid, l1i_mem_rsp_valid;
   
   
   logic 				  l1d_mem_req_ack;
   logic 				  l1d_mem_req_valid;
   logic [(`M_WIDTH-1):0] 		  l1d_mem_req_addr;
   logic [L1D_CL_LEN_BITS-1:0] 		  l1d_mem_req_store_data;
   logic [3:0] 				  l1d_mem_req_opcode;

   logic 				  l1i_mem_req_valid;
   logic [(`M_WIDTH-1):0] 		  l1i_mem_req_addr;
   logic [3:0] 				  l1i_mem_req_opcode;


   logic 				  insn_valid,insn_valid2;
   logic 				  insn_ack, insn_ack2;
   insn_fetch_t insn, insn2;

   wire w_l1_mem_req_ack;
   
   wire [127:0] w_l1_mem_load_data;
   wire		w_mode64, w_paging_active;
   wire [1:0]	w_priv;
   wire [63:0]	w_page_table_root;
   
   assign page_table_root = w_page_table_root;
   assign paging_active = w_paging_active;
   
   wire		w_clear_tlb;

   always_ff@(negedge clk)
     begin
	if(w_paging_active & l1d_mem_req_valid)
	  begin
	     $display("w_page_table_root = %x", w_page_table_root);
	     $display("dfetch addr %x", l1d_mem_req_addr);
	     $stop();		  
	  end
	
     end
   
   l2 l2cache (
	       .clk(clk),
	       .reset(reset),

	       .l1d_req(l1d_mem_req_valid),
	       .l1i_req(l1i_mem_req_valid),
	       .l1d_addr(l1d_mem_req_addr),
	       .l1i_addr(l1i_mem_req_addr),
	       .l1d_opcode(l1d_mem_req_opcode),

	       .l1d_rsp_valid(l1d_mem_rsp_valid),
	       .l1i_rsp_valid(l1i_mem_rsp_valid),
	       
	       .l1i_flush_req(flush_req_l1i),
	       .l1d_flush_req(flush_req_l1d),
	       .l1i_flush_complete(l1i_flush_complete),
	       .l1d_flush_complete(l1d_flush_complete),
	       
	       .flush_complete(w_l2_flush_complete),
	       
	       .l1_mem_req_ack(w_l1_mem_req_ack),
	       .l1_mem_req_store_data(l1d_mem_req_store_data),
	       
	       .l1_mem_load_data(w_l1_mem_load_data),
	       
	       .mem_req_valid(mem_req_valid),
	       .mem_req_addr(mem_req_addr),
	       .mem_req_store_data(mem_req_store_data),
	       .mem_req_opcode(mem_req_opcode),

	       .mem_rsp_valid(mem_rsp_valid),
	       .mem_rsp_load_data(mem_rsp_load_data),
	       .cache_accesses(l2_cache_accesses),
	       .cache_hits(l2_cache_hits)

	       );
   
   

   logic	drain_ds_complete;
   logic [(1<<`LG_ROB_ENTRIES)-1:0] dead_rob_mask;
`define PERFECT_L1D
`ifdef PERFECT_L1D
   perfect_l1d 
`else
     l1d
`endif
       dcache (
	       .clk(clk),
	       .reset(reset),
	       .page_table_root(w_page_table_root),
	       .paging_active(w_paging_active),
	       .clear_tlb(w_clear_tlb),
	       .head_of_rob_ptr_valid(head_of_rob_ptr_valid),
	       .head_of_rob_ptr(head_of_rob_ptr),
	       .retired_rob_ptr_valid(retired_rob_ptr_valid),
	       .retired_rob_ptr_two_valid(retired_rob_ptr_two_valid),
	       .retired_rob_ptr(retired_rob_ptr),
	       .retired_rob_ptr_two(retired_rob_ptr_two),
	       .restart_valid(restart_valid),
	       .memq_empty(memq_empty),
	       .drain_ds_complete(drain_ds_complete),
	       .dead_rob_mask(dead_rob_mask),
	       .flush_req(flush_req_l1d),
	       .flush_cl_req(flush_cl_req),
	       .flush_cl_addr(flush_cl_addr),
	       .flush_complete(l1d_flush_complete),
	       .core_mem_req_valid(core_mem_req_valid),
	       .core_mem_req(core_mem_req),
	       .core_mem_req_ack(core_mem_req_ack),

	       .core_store_data_valid(core_store_data_valid),
	       .core_store_data(core_store_data),
	       .core_store_data_ack(core_store_data_ack),
	       
	       .core_mem_rsp_valid(core_mem_rsp_valid),
	       .core_mem_rsp(core_mem_rsp),

	       .mem_req_valid(l1d_mem_req_valid),
	       .mem_req_addr(l1d_mem_req_addr),
	       .mem_req_store_data(l1d_mem_req_store_data),
	       .mem_req_opcode(l1d_mem_req_opcode),
	       
	       .mem_rsp_valid(l1d_mem_rsp_valid),
	       .mem_rsp_load_data(w_l1_mem_load_data),

	       .cache_accesses(l1d_cache_accesses),
	       .cache_hits(l1d_cache_hits)
	       );

   l1i icache(
	      .clk(clk),
	      .reset(reset),
	      .mode64(w_mode64),
	      .priv(w_priv),
	      .page_table_root(w_page_table_root),
	      .paging_active(w_paging_active),
	      .clear_tlb(w_clear_tlb),
	      .flush_req(flush_req_l1i),
	      .flush_complete(l1i_flush_complete),
	      .restart_pc(restart_pc),
	      .restart_src_pc(restart_src_pc),
	      .restart_src_is_indirect(restart_src_is_indirect),
	      .restart_valid(restart_valid),
	      .restart_ack(restart_ack),
	      .retire_reg_ptr(retire_reg_ptr),
	      .retire_reg_data(retire_reg_data),
	      .retire_reg_valid(retire_reg_valid),	      
	      .branch_pc_valid(t_branch_pc_valid),
	      .branch_pc(t_branch_pc),
	      .took_branch(took_branch),
	      .branch_fault(t_branch_fault),
	      .branch_pht_idx(branch_pht_idx),
	      .retire_valid(retire_valid),
	      .retired_call(retired_call),
	      .retired_ret(retired_ret),
	      .insn(insn),
	      .insn_valid(insn_valid),
	      .insn_ack(insn_ack),
	      .insn_two(insn2),
	      .insn_valid_two(insn_valid2),
	      .insn_ack_two(insn_ack2),
	      .mem_req_valid(l1i_mem_req_valid),
	      .mem_req_addr(l1i_mem_req_addr),
	      .mem_req_opcode(l1i_mem_req_opcode),
	      .mem_rsp_valid(l1i_mem_rsp_valid),
	      .mem_rsp_load_data(w_l1_mem_load_data),
	      .cache_accesses(l1i_cache_accesses),
	      .cache_hits(l1i_cache_hits)	      
	      );
   	      
   core cpu (
	     .clk(clk),
	     .reset(reset),
	     .syscall_emu(syscall_emu),
	     .took_exc(took_exc),
	     .priv(w_priv),
	     .clear_tlb(w_clear_tlb),
	     .paging_active(w_paging_active),
	     .page_table_root(w_page_table_root),
	     .mode64(w_mode64),
	     .extern_irq(extern_irq),
	     .resume(resume),
	     .memq_empty(memq_empty),
	     .drain_ds_complete(drain_ds_complete),
	     .dead_rob_mask(dead_rob_mask),	     
	     .head_of_rob_ptr_valid(head_of_rob_ptr_valid),	     
	     .head_of_rob_ptr(head_of_rob_ptr),
	     .resume_pc(resume_pc),
	     .ready_for_resume(ready_for_resume),  
	     .flush_req_l1d(flush_req_l1d),
	     .flush_req_l1i(flush_req_l1i),	     
	     .flush_cl_req(flush_cl_req),
	     .flush_cl_addr(flush_cl_addr),	     
	     .l1d_flush_complete(l1d_flush_complete),
	     .l1i_flush_complete(l1i_flush_complete),
	     .l2_flush_complete(w_l2_flush_complete),
	     .insn(insn),
	     .insn_valid(insn_valid),
	     .insn_ack(insn_ack),
	     .insn_two(insn2),
	     .insn_valid_two(insn_valid2),
	     .insn_ack_two(insn_ack2),
	     .branch_pc(t_branch_pc),
	     .branch_pc_valid(t_branch_pc_valid),
	     .branch_fault(t_branch_fault),
	     .took_branch(took_branch),
	     .branch_pht_idx(branch_pht_idx),
	     .restart_pc(restart_pc),
	     .restart_src_pc(restart_src_pc),
	     .restart_src_is_indirect(restart_src_is_indirect),
	     .restart_valid(restart_valid),
	     .restart_ack(restart_ack),
	     
	     .core_mem_req_ack(core_mem_req_ack),
	     .core_mem_req_valid(core_mem_req_valid),
	     .core_mem_req(core_mem_req),
	     
	     .core_store_data_valid(core_store_data_valid),
	     .core_store_data(core_store_data),
	     .core_store_data_ack(core_store_data_ack),
	     
	     .core_mem_rsp_valid(core_mem_rsp_valid),
	     .core_mem_rsp(core_mem_rsp),
	     .alloc_valid(alloc_valid),
	     .alloc_two_valid(alloc_two_valid),
	     .iq_one_valid(iq_one_valid),
	     .iq_none_valid(iq_none_valid),
	     .in_branch_recovery(in_branch_recovery),
	     .retire_reg_ptr(retire_reg_ptr),
	     .retire_reg_data(retire_reg_data),
	     .retire_reg_valid(retire_reg_valid),
	     .retire_reg_two_ptr(retire_reg_two_ptr),
	     .retire_reg_two_data(retire_reg_two_data),
	     .retire_reg_two_valid(retire_reg_two_valid),
	     .retire_valid(retire_valid),
	     .retire_two_valid(retire_two_valid),
	     .retire_pc(retire_pc),
	     .retire_two_pc(retire_two_pc),
	     .rob_empty(rob_empty),
	     .retired_call(retired_call),
	     .retired_ret(retired_ret),
	     .retired_rob_ptr_valid(retired_rob_ptr_valid),
	     .retired_rob_ptr_two_valid(retired_rob_ptr_two_valid),
	     .retired_rob_ptr(retired_rob_ptr),
	     .retired_rob_ptr_two(retired_rob_ptr_two),
	     .monitor_ack(monitor_ack),
	     .got_break(got_break),
	     .got_ud(got_ud),
	     .got_bad_addr(got_bad_addr),
	     .got_monitor(got_monitor),
	     .inflight(inflight),
	     .epc(epc)
	     );
   

endmodule // core_l1d_l1i


`ifdef FPGA64_32
module core_l1d_l1i(clk, 
		    reset,
		    syscall_emu,
		    extern_irq,
		    in_flush_mode,
		    resume,
		    resume_pc,
		    ready_for_resume,
		    
		    mem_req_valid, 
		    mem_req_addr, 
		    mem_req_store_data,
		    mem_req_opcode,
		    mem_rsp_valid,
		    mem_rsp_load_data,
		    alloc_valid,
		    alloc_two_valid,
		    iq_one_valid,
		    iq_none_valid,
		    in_branch_recovery,
		    retire_reg_ptr,
		    retire_reg_data,
		    retire_reg_valid,
		    retire_reg_two_ptr,
		    retire_reg_two_data,
		    retire_reg_two_valid,
		    retire_valid,
		    retire_two_valid,
		    rob_empty,
		    retire_pc,
		    retire_two_pc,
		    branch_pc,
		    branch_pc_valid,		    
		    branch_fault,
		    l1i_cache_accesses,
		    l1i_cache_hits,
		    l1d_cache_accesses,
		    l1d_cache_hits,
		    l2_cache_accesses,
		    l2_cache_hits,
		    monitor_ack,
		    got_break,
		    got_ud,
		    got_bad_addr,
		    got_monitor,
		    inflight,
		    epc);

   localparam L1D_CL_LEN = 1 << `LG_L1D_CL_LEN;
   localparam L1D_CL_LEN_BITS = 1 << (`LG_L1D_CL_LEN + 3);
   
   input logic clk;
   input logic reset;
   input logic syscall_emu;
   input logic extern_irq;
   input logic resume;
   input logic [31:0] resume_pc;
   output logic 		in_flush_mode;
   output logic 		ready_for_resume;
   
   output logic [31:0] branch_pc;
   output logic 		 branch_pc_valid;
   output logic 		 branch_fault;

   output logic [63:0] 			l1i_cache_accesses;
   output logic [63:0] 			l1i_cache_hits;
   output logic [63:0] 			l1d_cache_accesses;
   output logic [63:0] 			l1d_cache_hits;
   output logic [63:0] 			l2_cache_accesses;
   output logic [63:0] 			l2_cache_hits;   

   
   /* mem port */
   output logic 			mem_req_valid;
   output logic [31:0] 		mem_req_addr;
   output logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0] mem_req_store_data;
   output logic [3:0] 				 mem_req_opcode;
   
   input logic 					 mem_rsp_valid;
   input logic [(1 << (`LG_L2_CL_LEN+3)) - 1:0]  mem_rsp_load_data;
   
   output logic					 alloc_valid;
   output logic					 alloc_two_valid;
   output logic					 iq_one_valid;
   output logic					 iq_none_valid;
   output logic					 in_branch_recovery;
   output logic [4:0]				 retire_reg_ptr;
   output logic [31:0]				 retire_reg_data;
   output logic					 retire_reg_valid;
   output logic [4:0]				 retire_reg_two_ptr;
   output logic [31:0]				 retire_reg_two_data;
   output logic					 retire_reg_two_valid;
   output logic					 retire_valid;
   output logic					 retire_two_valid;
   output logic					 rob_empty;   
   output logic [31:0]				 retire_pc;
   output logic [31:0]				 retire_two_pc;
   input logic					 monitor_ack;
   output logic					 got_break;
   output logic					 got_ud;
   output logic					 got_bad_addr;
   output logic					 got_monitor;
   
   output logic [`LG_ROB_ENTRIES:0]		 inflight;
   output logic [31:0]				 epc;

   wire [63:0]					 w_resume_pc64 = {32'd0, resume_pc};
   
   wire [63:0]					 w_mem_req_addr64;
   assign mem_req_addr = w_mem_req_addr64[31:0];
   
   wire [63:0]					 w_retire_reg_data64, w_retire_reg_two_data64;
   assign retire_reg_data = w_retire_reg_data64[31:0];
   assign retire_reg_two_data = w_retire_reg_two_data64[31:0];
   
   wire [63:0]					 w_retire_pc64, w_retire_two_pc64;
   assign retire_pc = w_retire_pc64[31:0];
   assign retire_two_pc = w_retire_two_pc64[31:0];
   
   
   wire [63:0]					 w_branch_pc64;
   assign branch_pc = w_branch_pc64[31:0];
   
   wire [63:0]					 w_epc64;
   assign epc = w_epc64[31:0];
   
   
   core_l1d_l1i_64 c(
		     .clk(clk),
                     .reset(reset),
		     .syscall_emu(syscall_emu),
                     .extern_irq(extern_irq),
                     .in_flush_mode(in_flush_mode),
                     .resume(resume),
                     .resume_pc(w_resume_pc64),
                     .ready_for_resume(ready_for_resume),
                     .mem_req_valid(mem_req_valid),
                     .mem_req_addr(w_mem_req_addr64),
                     .mem_req_store_data(mem_req_store_data),
                     .mem_req_opcode(mem_req_opcode),
                     .mem_rsp_valid(mem_rsp_valid),
                     .mem_rsp_load_data(mem_rsp_load_data),
                     .alloc_valid(alloc_valid),
                     .alloc_two_valid(alloc_two_valid),
                     .iq_one_valid(iq_one_valid),
                     .iq_none_valid(iq_none_valid),
                     .in_branch_recovery(in_branch_recovery),
                     .retire_reg_ptr(retire_reg_ptr),
                     .retire_reg_data(w_retire_reg_data64),
                     .retire_reg_valid(retire_reg_valid),
                     .retire_reg_two_ptr(retire_reg_two_ptr),
                     .retire_reg_two_data(w_retire_reg_two_data64),
                     .retire_reg_two_valid(retire_reg_two_valid),
                     .retire_valid(retire_valid),
                     .retire_two_valid(retire_two_valid),
		     .rob_empty(rob_empty),
                     .retire_pc(w_retire_pc64),
                     .retire_two_pc(w_retire_two_pc64),
                     .branch_pc(w_branch_pc64),
                     .branch_pc_valid(branch_pc_valid),		     
                     .branch_fault(branch_fault),
                     .l1i_cache_accesses(l1i_cache_accesses),
                     .l1i_cache_hits(l1i_cache_hits),
                     .l1d_cache_accesses(l1d_cache_accesses),
                     .l1d_cache_hits(l1d_cache_hits),
                     .l2_cache_accesses(l2_cache_accesses),
                     .l2_cache_hits(l2_cache_hits),
                     .monitor_ack(monitor_ack),
                     .got_break(got_break),
                     .got_ud(got_ud),
                     .got_bad_addr(got_bad_addr),
                     .got_monitor(got_monitor),
                     .inflight(inflight),
                     .epc(w_epc64)
		     );

   
endmodule
`endif
