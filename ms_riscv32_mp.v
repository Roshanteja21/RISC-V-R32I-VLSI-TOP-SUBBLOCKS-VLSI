`timescale 1ns/1ps

module ms_risc32_mp #(parameter BOOT_ADDRESS = 32'h00000000)
		   (input ms_riscv32_mp_clk_in,
		    input ms_riscv32_mp_rst_in, 
		    input ms_risc32_mp_instr_hready_in, ms_riscv32_mp_hresp_in,ms_risc32_mp_data_hready_in,
		    input ms_riscv32_eirq_in,ms_riscv32_tirq_in,ms_riscv32_sirq_in, 
		    input [31:0] ms_risc32_mp_dmdata_in,ms_risc32_mp_instr_in,
		    input  [3:0] ms_risc32_mp_dmwr_mask_in,
		    input [63:0] ms_risc32_mp_rc_in,
		    output ms_riscv32_mp_dmwr_req_out,
		    output [31:0] ms_risc32_mp_imaddr_out, ms_risc32_mp_dmaddr_out, ms_risc32_mp_dmdata_out,
		    output [1:0] ms_risc32_mp_data_htrans_out,
		    output [3:0] ms_risc32_mp_dmwr_mask_out);


// ms_risc32_mp_rc_in -> Connection with Real Time Counter
//ms_risc32_mp_imaddr_out, ms_risc32_mp_instr_in, ms_risc32_mp_instr_hready_in -> connection with Instruction Memory
// Connections with data memory:
// ms_risc32_mp_dmdata_in, ms_risc32_mp_data_hready_in, ms_risc32_mp_dmwr_mask_in, ms_risc32_mp_dmdata_out, ms_risc32_mp_data_htrans_out, ms_risc32_mp_dmwr_mask_out
// ms_risc32_mp_data_hready_in -> AHB ready
// ms_risc32_mp_data_htrans_out -> AHB data trans
// ms_riscv32_mp_hresp_in -> AHB response
// Connections with Interrupt Controller


// writeback selection
parameter WB_ALU = 3'b000;
parameter WB_LU = 3'b001;
parameter WB_IMM = 3'b010;
parameter WB_IADDER_OUT = 3'b011;
parameter WB_CSR = 3'b100;
parameter WB_PC_PLUS = 3'b101;


//INTERNAL WIRES AND REGISTERS
//pc_mux
wire [1:0] pc_src;
wire [31:0] pc;
wire [31:0] epc;
wire [31:0] trap_address;
wire branch_taken;
wire [31:0] pc_plus_4;
wire misaligned_instr;
wire pc_mux;

//imm_generator
wire [31:7] instr;
wire [2:0] imm_type;
wire [31:0] immout;

//imm adder
wire [31:0] rs_1,rs_2;
wire iadder_src;
wire [31:0] iadder;

//integer file
wire [31:0] rd;
wire wr_en;
wire [4:0] rs1_add,rs2_add,rd_add;

//write enable
wire flush;
wire rf_wr_en, csr_wr_en, wr_en_csr_out;

// instruction mux
wire [6:0] opcode, funct7;
wire [2:0] funct3;
wire [11:0] csr_add;

//decoder
wire trap_taken;
wire [2:0] wb_mux;
wire [2:0] csr_op,load_size;
wire [3:0] alu_opcode;
wire mem_wr_req, load_unsigned, alu_src,illegal_instr, misaligned_load, misaligned_store;

//reg block 2
wire alu_src_reg,csr_wr_en_reg, rf_wr_en_reg;
wire [4:0] rd_addr_reg;
wire [11:0] csr_addr_reg;
wire [31:0] rs1_reg,rs2_reg, pc_reg, pc_plus_4_reg, iadder_reg, imm_reg;
wire [1:0] load_size_reg;
wire [2:0] wb_mux_sel_reg;
wire load_unsigned_reg;

// Load unit
wire [31:0] lu_out;

//alu
wire [31:0] result;
wire [31:0] alu_2nd_src;

//wbmux
wire [31:0] csr_data;

// machine control
wire i_or_e;
wire [3:0] cause;
wire instrct_inc;
wire mie, meie, mtie, msie, meip, mtip, msip;
wire set_epc, set_cause, mie_clear,mie_set, misaligned_exception;



msrv32_pc PC (.rst_in(ms_riscv32_mp_rst_in),
		  .pc_src_in(pc_src),
		  .pc_in(pc),
		  .epc_in(epc),
		  .trap_address_in(trap_address),
		  .branch_taken_in(branch_taken),
		  .iaddr_in(iadder[31:1]),
		  .ahb_ready_in(ms_risc32_mp_instr_hready_in), // ahb_ready_in = ms_risc32_mp_instr_hready_in
		  .iaddr_out(ms_risc32_mp_imaddr_out),
		  .pc_plus_4_out(pc_plus_4),
		  .misaligned_instr_logic_out(misaligned_instr),
		  .pc_mux_out(pc_mux)); 

msrv32_reg_block_1 REG_1(.pc_mux_in(pc_mux),
		   	 .ms_risc32_mp_clk_in(ms_riscv32_mp_clk_in),
		   	 .ms_risc32_mp_rst_in(ms_riscv32_mp_rst_in),
		   	 .pc_out(pc));

msrv32_imm_generator IMM_GEN(.instr_in(instr),
			     .imm_type_in(imm_type),
			     .imm_out(immout));

msrv32_immediate_adder IMM_ADD(.pc_in(pc),
			 .rs_1_in(rs_1),
			 .imm_in(immout),
			 .iadder_src_in(iadder_src),
			 .iadder_out(iadder));

msrv32_integer_file INT_FILE (.ms_riscv32_mp_clk_in(ms_riscv32_mp_clk_in),
			      .ms_riscv32_mp_rst_in(ms_riscv32_mp_rst_in),
			      .wr_en_in(wr_en),
			      .rs_2_addr_in(rs2_add),
			      .rd_addr_in(rd_add),
			      .rs_1_addr_in(rs1_add),
			      .rd_in(rd),
			      .rs_1_out(rs_1),
			      .rs_2_out(rs_2));

msrv32_wr_en_generator WR_EN(.flush_in(flush),
			     .rf_wr_en_reg_in(rf_wr_en_reg),
			     .csr_wr_en_reg_in(csr_wr_en_reg),
			     .wr_en_integer_file_out(wr_en),
			     .wr_en_csr_file_out(wr_en_csr_out));

msrv32_instruction_mux IR_MUX(.flush_in(flush),
			      .ms_riscv32_mp_instr_in(ms_risc32_mp_instr_in),
			      .opcode_out(opcode),
			      .funct7_out(funct7),
			      .funct3_out(funct3),
			      .rs_1_addr_out(rs1_add),
			      .rs_2_addr_out(rs2_add),
			      .rd_addr_out(rd_add),
			      .csr_addr_out(csr_add),
			      .instr_31_7_out(instr));

msrv32_branch_unit BRANCH_UNIT (.rs_1_in(rs_1),
			        .rs_2_in(rs_2),
			        .opcode_in(opcode[6:2]),
			        .funct3_in(funct3),
				.branch_taken_out(branch_taken));

msrv32_decoder DECODER (.trap_taken_in(trap_taken),
			.funct7_5_in(funct7[5]),
			.opcode_in(opcode),
			.funct3_in(funct3),
			.iadder_out_1_to_0_in(iadder[1:0]),
			.wb_mux_sel_out(wb_mux),
			.imm_type_out(imm_type),
			.csr_op_out(csr_op),
			.alu_opcode_out(alu_opcode),
			.load_size_out(load_size),
			.mem_wr_req_out(mem_wr_req),
			.load_unsigned_out(load_unsigned),
			.alu_src_out(alu_src),
			.iadder_src_out(iadder),
			.csr_wr_en_out(csr_wr_en),
			.rf_wr_en_out(rf_wr_en),
			.illegal_instr_out(illegal_instr),
			.misaligned_load_out(misaligned_load),
			.misaligned_store_out(misaligned_store));

msrv32_machine_control MAC_CTRL(.clk_in(ms_riscv32_mp_clk_in),
		    		.reset_in(ms_riscv32_mp_rst_in),
				.e_irq_in(ms_riscv32_eirq_in),
				.t_irq_in(ms_riscv32_tirq_in),
				.s_irq_in(ms_riscv32_sirq_in),
				.illegal_instr_in(illegal_instr),
				.misaligned_load_in(misaligned_load),
				.misaligned_store_in(misaligned_store),
				.misaligned_instr_in(misaligned_instr),
				.opcode_6_to_2_in(opcode[6:2]),
				.funct3_in(funct3),
				.funct7_in(funct7),
				.rs1_addr_in(rs1_add),
				.rs2_addr_in(rs2_add),
				.rd_addr_in(rd_add),
				.mie_in(mie),
				.meie_in(meie),
				.mtie_in(mtie),
				.msie_in(msie),
				.meip_in(meip),
				.mtip_in(mtip),
				.msip_in(msip),
				.i_or_e_out(i_or_e),
				.cause_out(cause),
				.instret_inc_out(instrct_inc),
				.set_epc_out(set_epc),
				.set_cause_out(set_cause),
				.mie_clear_out(mie_clear),
				.mie_set_out(mie_set),
				.misaligned_exception_out(misaligned_exception),
				.pc_src_out(pc_src),
				.flush_out(flush),
				.trap_taken_out(trap_taken));

msrv32_csr_file CSR(.clk_in(ms_riscv32_mp_clk_in),
		    .rst_in(ms_riscv32_mp_rst_in),
		    .wr_en_in(wr_en_csr_out),
		    .csr_addr_in(csr_addr_reg),
		    .csr_op_in(csr_op_reg),
		    .csr_uimm_in(imm_reg[31:28]),
		    .csr_data_in(rs1_reg),
		    .pc_in(pc_reg),
		    .iadder_in(iadder_reg),
		    .e_irq_in(ms_riscv32_eirq_in),
		    .s_irq_in(ms_riscv32_sirq_in),
		    .t_irq_in(ms_riscv32_tirq_in),
		    .set_cause_in(set_cause),
		    .set_epc_in(set_epc),
		    .instret_inc_in(instrct_inc),
		    .mie_clear_in(mie_clear),
		    .mie_set_in(mie_set),
		    .cause_in(cause),
		    .real_time_in(ms_risc32_mp_rc_in),
		    .misaligned_exception_in(misaligned_exception),
		    .csr_data_out(csr_data),
		    .mie_out(mie),
		    .epc_out(epc),
		    .trap_address_out(trap_address),
		    .meie_out(meie),
		    .mtie_out(mtie),
		    .msie_out(msie),
		    .meip_out(meip),
		    .mtip_out(mtip),
		    .msip_out(msip));

msrv32_reg_block2 REG_2(.clk_in(ms_riscv32_mp_clk_in),
			.reset_in(ms_riscv32_mp_rst_in),
			.branch_taken_in(branch_taken),
			.load_unsigned_in(load_unsigned),
			.alu_src_in(alu_src),
			.csr_wr_en_in(csr_wr_en),
			.rf_wr_en_in(rf_wr_en),
			.rd_addr_in(rd_add),
			.csr_addr_in(csr_add),
			.rs1_in(rs_1),
			.rs2_in(rs_2),
			.pc_in(pc),
			.pc_plus_4_in(pc_plus_4),
			.iadder_in(iadder),
			.imm_in(immout),
			.alu_opcode_in(alu_opcode),
			.load_size_in(load_size),
			.wb_mux_sel_in(wb_mux),
			.csr_op_in(csr_op),
			.load_unsigned_reg_out(load_unsigned_reg),
			.alu_src_reg_out(alu_src_reg),
			.csr_wr_en_reg_out(csr_wr_en_reg),
			.rf_wr_en_reg_out(rf_wr_en_reg),
			.rd_addr_reg_out(rd_addr_reg),
			.csr_addr_reg_out(csr_addr_reg),
			.rs1_reg_out(rs1_reg),
			.rs2_reg_out(rs2_reg),
			.pc_reg_out(pc_reg),
			.pc_plus_4_reg_out(pc_plus_4_reg),
			.iadder_reg_out(iadder_reg),
			.imm_reg_out(imm_reg),
			.alu_opcode_reg_out(alu_opcode_reg),
			.load_size_reg_out(load_size_reg),
			.wb_mux_sel_reg_out(wb_mux_sel_reg),
			.csr_op_reg_out(csr_op_reg));

msrv32_store_unit STORE(.funct3_in(funct3),
			.iadder_in(iadder),
			.rs2_in(rs_2),
			.mem_wr_req_in(mem_wr_req),
			.ahb_ready_in(ms_risc32_mp_instr_hready_in),
			.ms_riscv32_mp_dmdata_out(ms_risc32_mp_dmdata_out),
			.ms_riscv32_mp_dmaddr_out(ms_risc32_mp_dmaddr_out),
			.ms_riscv32_mp_dmwr_mask_out(ms_risc32_mp_dmwr_mask_out),
			.ms_riscv32_mp_dmwr_req_out(ms_riscv32_mp_dmwr_req_out),
			.ahb_htrans_out(ms_risc32_mp_data_htrans_out));

msrv32_load_unit LOAD(.ahb_resp_in(ms_riscv32_mp_hresp_in),
		      .load_unsigned_in(load_unsigned_reg),
		      .ms_riscv32_mp_dmdata_in(ms_risc32_mp_dmdata_in),
		      .iadder_out_1_to_0_in(iadder_reg),
		      .load_size_in(load_size_reg),
		      .lu_output_out(lu_out));

msrv32_alu ALU (.op_1_in(rs1_reg),
		.op_2_in(alu_2nd_src),
		.opcode_in(alu_opcode_reg),
		.result_out(result));


msrv32_wb_mux_sel_unit WB_MUX (.alu_src_reg_in(alu_src_reg),
				.wb_mux_sel_reg_in(wb_mux_sel_reg),
				.alu_result_in(result),
				.lu_output_in(lu_out),
				.imm_reg_in(imm_reg),
				.iadder_out_reg_in(iadder_reg),
				.csr_data_in(csr_data),
				.pc_plus_4_reg_in(pc_plus_4_reg),
				.rs2_reg_in(rs2_reg),
				.wb_mux_out(rd),
				.alu_2nd_src_mux_out(alu_2nd_src));

endmodule
