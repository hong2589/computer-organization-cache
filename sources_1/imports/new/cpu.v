`timescale 1ns/1ns
`define WORD_SIZE 16    // data and address word size

`include "opcodes.v"
`define FETCH_SIZE 64

module cpu(
    input Clk, 
    input Reset_N, 

    // Instruction memory interface
    output i_readM, 
    output i_writeM, 
    output [`WORD_SIZE-1:0] i_address, 
    inout [`FETCH_SIZE-1:0] i_data, 

	// Data memory interface
    output d_readM, 
    output d_writeM, 
    output [`WORD_SIZE-1:0] d_address, 
    inout [`FETCH_SIZE-1:0] d_data, 

    output [`WORD_SIZE-1:0] num_inst, 
    output [`WORD_SIZE-1:0] output_port, 
    output is_halted
);
    // TODO : Implement your pipelined CPU!
	// control signal declaration
	// opcode, function code
	wire [3:0] opcode; // inst[15:12]
    wire [5:0] func_code; // inst[5:0] 
	
	// control signals from control_unit to datapath
	wire isWWD_WB; // assert output_port = RF[$0]
	wire [1:0] RegDst; // RF address to write data. 0: rt, 1: rd, 2: $2
	wire [3:0] ALUOp; // opcode for ALU. It's defined at opcodes.v
	wire [1:0] ALUSrcB; // select 2nd source of ALU. 0: RF_B, 1: sign_immediate, 2: LHI_immediate
	wire d_writeM; // write signal to data mermory interface
	wire d_readM; // read signal to data memory interface
	wire RegWrite; // write signal to RF
	wire [1:0] WBSrc; // select data to write back into RF. 0: lwData(LWD), 1: wbData, 2: PC_WB(for JPL, JRL)
	
	// control signals from control_unit to hazard_control
	wire RegWrite_EX; // RegWrite in EX
	wire RegWrite_M; // RegWrite in MEM
	wire RegWrite_WB; // RegWrite in WB

	// control signals from hazard_control to datapath
	wire PCWrite; // PCWrite enable signal
	wire IDWrite; // IF/ID update enable signal
	wire EXWrite; // ID/EX update enable signal
	wire MWrite; // EX/MEM update enable signal
	wire WBWrite; // MEM/WB update enable signal
	wire [1:0] btbSrc; // select signal for address to update BTB. 0: brTarget, 1: rfData_1, 2: jumpAddr
	wire btbWrite; // BTB write enable signal
	wire flush; // flush signal to disenable all the control signal from EX
	wire flush_EX; 
	wire isPredict; // when the instruction in ID stage is branch or jump -> isPredict=1

	// Select signal for forwarding 
	// 0: rfData_1/rfData_2, 1: ALUResult_main, 2: d_data, 3: rfData_w 
	wire [1:0] forwardSrcA; wire [1:0] forwardSrcB;

	// wires from datapath to control_unit, hazard_control
	wire [1:0] rs; // 1st rf address, inst[11:10]
	wire [1:0] rt; // 2nd rf address, inst[9:8]
	wire [1:0] destEX; // WB register destination in EX stage
	wire [1:0] destM; // WB register destination in MEM stage
	wire [1:0] destWB; // WB register destination in WB stage
	wire bcond; // branch condition. 1: branch actually taken, 0: branch actually not taken
	wire [`WORD_SIZE-1:0] pcTarget; // target address to update BTB and to control PC in flush situation
	wire [`WORD_SIZE-1:0] predictedPC; // predictedPC from previous IF stage
	wire [`WORD_SIZE-1:0] nextPC; // PC + 1
	wire [`WORD_SIZE-1:0] brTarget; // branch target addresss (BNE, BEQ, BGZ, BLZ)
	wire [`WORD_SIZE-1:0] jrTarget; // 1st register file value referenced by $rs
	wire [`WORD_SIZE-1:0] jumpAddr; // jump address
	wire [3:0] opcode_EX; // opcode of the instruction in EX stage
	wire [3:0] opcode_M; // opcode of the instruction in MEM stage
	wire [3:0] opcode_WB; // opcode of the instruction in WB stage

	// cache - datapath interface
	wire i_readC; wire i_writeC; wire [`WORD_SIZE-1:0] i_addressC; wire [`WORD_SIZE-1:0] i_dataC;
	wire d_readC; wire d_writeC; wire [`WORD_SIZE-1:0] d_addressC; wire [`WORD_SIZE-1:0] d_dataC;
	// cache hit indicator
	wire i_cache_hit; wire d_cache_hit;
	// cache block ready indicator
	wire i_ready; wire d_ready;
	// indicator of the case that there are both I-cache, D-cache access
	wire both_access; 

	// In cpu module, three submodules interact each other - datapath, control_unit, and hazard_control.

	// 1. datapath : include all the wires of piepeline & stage latch registers. 
	// It deals with data flow from IF stage to WB stage of pipelined cpu
	// It is typically controlled by control_unit, but if there is data/control hazard, 
	// it is controlled by hazard_control by flush signals and latch-enable signals for pipeline stage registers
	datapath DP (
		.clk(Clk),
		.reset_n(Reset_N),
		.opcode(opcode),
		.func_code(func_code),
		.i_address(i_addressC),
		.i_data(i_dataC),
		.d_address(d_addressC),
		.d_data(d_dataC),
		.output_port(output_port),
		.isWWD_WB(isWWD_WB),
		.RegDst(RegDst),
		.ALUOp(ALUOp),
		.ALUSrcB(ALUSrcB),
		.i_writeM(i_writeC),
		.i_readM(i_readC),
		.d_writeM(d_writeC),
		.d_readM(d_readC), 
		.RegWrite(RegWrite), 
		.WBSrc(WBSrc), 
		.PCWrite(PCWrite),
		.IDWrite(IDWrite),
		.EXWrite(EXWrite),
		.MWrite(MWrite),
		.WBWrite(WBWrite),
		.btbSrc(btbSrc),
		.btbWrite(btbWrite),
		.flush(flush),
		.forwardSrcA(forwardSrcA),
		.forwardSrcB(forwardSrcB),
		.isPredict(isPredict),
		.flush_EX(flush_EX),
		.rs(rs),
		.rt(rt), 
		.destEX(destEX), 
		.destM(destM), 
		.destWB(destWB), 
		.bcond(bcond),
		.brTarget(brTarget),
		.jrTarget(jrTarget),
		.jumpAddr(jumpAddr),
		.pcTarget(pcTarget),
		.predictedPC(predictedPC),
		.nextPC(nextPC),
		.opcode_EX(opcode_EX),
		.opcode_M(opcode_M),
		.opcode_WB(opcode_WB)
	);

	// 2. control_unit : manage all the control signals used in datapath, control_hazard modules
	// It transfers different control signals to datapath, like multipe pipeline latch write signals(MWrite, WBWrite ...),
	// and control units contained in datapath(ALU, RF)
	// It also transfers RegWrite signals from different pipeline stages to control_hazard module,
	// which enable the control_hazard module to detect data hazard. 
	control_unit Control (
		.clk(Clk),
		.reset_n(Reset_N),
		.opcode(opcode),
		.func_code(func_code),
		.bcond(bcond),
		.EXWrite(EXWrite),
		.MWrite(MWrite),
		.WBWrite(WBWrite),
		.opcode_M(opcode_M),
		.opcode_WB(opcode_WB),
		.num_inst(num_inst),
		.i_writeC(i_writeC),
		.i_readC(i_readC),
		.isWWD_WB(isWWD_WB),
		.RegDst(RegDst),
		.ALUOp(ALUOp),
		.ALUSrcB(ALUSrcB),		
		.d_writeC(d_writeC),
		.d_readC(d_readC),
		.RegWrite(RegWrite),
		.WBSrc(WBSrc),
		.is_halted(is_halted),
		.RegWrite_EX(RegWrite_EX),
		.RegWrite_M(RegWrite_M), 
		.RegWrite_WB(RegWrite_WB),
		.flush_EX(flush_EX)
	);

	// 3. hazard_control : manage data/control hazard situations, resolve the hazard
	// It detects hazard situations, and stop(stall) datapath by stage latch enable signals(PCWrite, EXWrite, MWrite, WBWrite, WBWrite)
	// and flush the instructions in datapath by flush signal.
	hazard_control HC (
		.clk(Clk),
		.reset_n(Reset_N),
		.rs(rs),
		.rt(rt),
		.opcode(opcode),
		.opcode_EX(opcode_EX),
		.opcode_M(opcode_M),
		.opcode_WB(opcode_WB),
		.func_code(func_code),
		.brTarget(brTarget),
		.jrTarget(jrTarget),
		.jumpAddr(jumpAddr),
		.RegWrite_EX(RegWrite_EX),
		.RegWrite_M(RegWrite_M),
		.RegWrite_WB(RegWrite_WB),
		.is_halted(is_halted),
		.i_cache_hit(i_cache_hit),
		.d_cache_hit(d_cache_hit),
		.i_ready(i_ready),
		.d_ready(d_ready),
		.both_access(both_access),
		.destEX(destEX),
		.destM(destM),
		.destWB(destWB),
		.bcond(bcond),
		.predictedPC(predictedPC),
		.nextPC(nextPC),
		.PCWrite(PCWrite),
		.IDWrite(IDWrite),
		.EXWrite(EXWrite),
		.MWrite(MWrite),
		.WBWrite(WBWrite),
		.btbSrc(btbSrc),
		.btbWrite(btbWrite),
		.flush(flush),
		.isPredict(isPredict),
		.forwardSrcA(forwardSrcA),
		.forwardSrcB(forwardSrcB),
		.flush_EX(flush_EX)
	);

	// 4. cache : datapath accesses cache instead of accessing memory directly.
	// If referenced data exists in cache(cache hit), then transmit the data to datapath without memory access.
	// cache access latency is 1 cycle and memory access latency is 4 cycles.
	cache cache(
		.clk(Clk),
		.reset_n(Reset_N),
		.i_readC(i_readC),
		.i_writeC(i_writeC),
		.i_addressC(i_addressC),
		.i_dataC(i_dataC),
		.d_readC(d_readC),
		.d_writeC(d_writeC),
		.d_addressC(d_addressC),
		.d_dataC(d_dataC),
		.i_readM(i_readM),
		.i_writeM(i_writeM),
		.i_addressM(i_address),
		.i_dataM(i_data),
		.d_readM(d_readM),
		.d_writeM(d_writeM),
		.d_addressM(d_address),
		.d_dataM(d_data),
		.i_cache_hit(i_cache_hit),
		.d_cache_hit(d_cache_hit),
		.i_ready(i_ready),
		.d_ready(d_ready),
		.both_access(both_access),
		.EXWrite(EXWrite)
	);

endmodule
