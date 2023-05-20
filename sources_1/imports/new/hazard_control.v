`include "opcodes.v"
module hazard_control (
	input clk,
	input reset_n,

	// signals from datapath
	input [1:0] rs, // rs address to used in ID
	input [1:0] rt, // rt address to used in ID
	input [3:0] opcode, // inst[15:12]
	input [3:0] opcode_EX, // opcode of the instruction in EX stage
	input [3:0] opcode_M, // opcode of the instruction in MEM stage
	input [3:0] opcode_WB, // opcode of the instruction in WB stage
	input [5:0] func_code, // inst[5:0]
	input IFState, // I-memory access state in IF stage from datapath
	input MState, // D-memory access state in MEM stage from datapath
	input [`WORD_SIZE-1:0] brTarget, // actual branch target address
	input [`WORD_SIZE-1:0] jrTarget, // RF[$rs]
	input [`WORD_SIZE-1:0] jumpAddr, // actual jump address
	
	// RegWrite signal from contorl_unit
	input RegWrite_EX, // Register write signal in EX
	input RegWrite_M, // Register write signal in MEM
	input RegWrite_WB, // Register write signal in WB
	input is_halted, // signal indicating HLT instruction committed

	// cache hit signal from cache
	input i_cache_hit,
	input d_cache_hit,

	// WB destination from datapath
	input [1:0] destEX, // WB register address in EX
	input [1:0] destM, // WB register address in MEM
	input [1:0] destWB, // WB register address in WB

	// signals related to branch/jump prediction from datapath 
	input bcond, // branch condition
	input [`WORD_SIZE-1:0] predictedPC, // predict result
	input [`WORD_SIZE-1:0] nextPC, // PC + 1

	// control signal for stage latch registers
	output PCWrite, // PC write enable signal
	output IFWrite, // IF/ID write enable signal
	output IDWrite, // ID/EX write enable signal
	output EXWrite, // EX/M write enable signal
	output MWrite, // M/WB write enable signal

	// control signals for branch/jump prediction
	output [1:0] btbSrc, // select signal for BTB update. 0 : brTarget, 1: rfData_1, 2: jumpAddr, 3: nextPC
	output btbWrite, // BTB update enable signal
	output flush, // signal to flush the mispredicted instruction in IF stage
	output isPredict, // signal to indicate branch / jump instruction in ID stage

	// control signal for forwarding
	output [1:0] forwardSrcA, // 1st forward source select signal
	output [1:0] forwardSrcB // 2nd forward source select signal
); 
	// stage write enable signals 
	reg PCWrite; reg IFWrite; reg IDWrite; reg EXWrite; reg MWrite;

	// signals about control prediction
	reg [1:0] btbSrc; // select signal for btb write
	reg btbWrite; // write enable signal for BTB
	reg flush; // flush == 1 only when there is misprediction
	reg isStall; // indicating signal of STALL for debugging

	// forward source output register
	reg [1:0] forwardSrcA; reg [1:0] forwardSrcB;
	reg isPredict;

	// state parameters, registers
	parameter RESET = 2'd0;
	parameter ACCESS_I = 2'd1;
	parameter ACCESS_D = 2'd2;
	parameter HAZARD_STALL = 2'd3;
	reg [1:0] control_state;
	reg [1:0] next_control_state;

	// LWD hazard indicator
	wire rs_dependence_EX; wire rs_dependence_M; wire rs_dependence_WB; // indicate whether there is data dependence on rs register
	wire rt_dependence_EX; wire rt_dependence_M; wire rt_dependence_WB; // indicate whether there is data dependence on rt register
	wire use_rs_ID; wire use_rt_ID; // indicate whether the instruction in ID stage uses rs or rt register.
	wire LWD_hazard; // indicate the stall case : LWD in EX stage and data dependence exists with the instruction in ID stage
	assign {rs_dependence_EX, rs_dependence_M, rs_dependence_WB} = {(rs == destEX & RegWrite_EX), (rs == destM & RegWrite_M), (rs == destWB & RegWrite_WB)};
	assign {rt_dependence_EX, rt_dependence_M, rt_dependence_WB} = {(rt == destEX & RegWrite_EX), (rt == destM & RegWrite_M), (rt == destWB & RegWrite_WB)};
	assign use_rs_ID = (opcode == `OPCODE_RTYPE && func_code != `FUNC_HLT) || opcode == `OPCODE_SWD || opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ;
	assign use_rt_ID = (opcode == `OPCODE_RTYPE && (func_code == `FUNC_ADD || func_code == `FUNC_SUB || func_code == `FUNC_AND || func_code == `FUNC_ORR))
						|| opcode == `OPCODE_SWD || opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ;
	assign LWD_dependence_hazard = (opcode_EX == `OPCODE_LWD) && ((use_rs_ID && rs_dependence_EX) || (use_rt_ID && rt_dependence_EX));

	// update next_control_state
	always @(*) begin
		case(control_state)
			RESET : begin
				casex ({d_cache_hit, i_cache_hit})
					2'b0x : next_control_state <= ACCESS_D;
					2'b10 : next_control_state <= ACCESS_I;
					2'b11 : next_control_state <= (LWD_dependence_hazard)? HAZARD_STALL : RESET;
				endcase
			end
			ACCESS_I : next_control_state <= (i_ready)? RESET : ACCESS_I;
			ACCESS_D : next_control_state <= (d_ready)? RESET : ACCESS_D;
			HAZARD_STALL : next_control_state <= RESET;
		endcase
	end

	// update synchronously
	always @(posedge clk, negedge reset_n) begin
		if (!reset_n) begin
			{PCWrite , IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // turn on all the write signals
			{btbSrc, btbWrite, flush} <= 4'b0000; // disable BTB write, reset flush to 0
			{isStall, forwardSrcA, forwardSrcB, isPredict} <= {1'd0, 2'd0, 2'd0, 1'd0};
			{control_state, next_control_state} <= {RESET, RESET};
		end
		else begin
			control_state <= next_control_state; // update current control_state
		end
	end

	// update signal asynchronously
	always @(*) begin
		// flushed instruction
		if (opcode == `OPCODE_FLUSH) begin
			case (control_state)
				RESET : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
				ACCESS_I : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111; // stall IF, ID
				ACCESS_D : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				HAZARD_STALL : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00011; // stall all
			endcase



			casex ({MState, IFState})
				2'b0x : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				2'b10 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111; // stall PC
				2'b11 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
			endcase
			{isStall, btbWrite, btbSrc, flush, isPredict} <= {1'd0, 1'd0, 2'd0, 1'd0, 1'd0};
		end

		// use both $rs, $rt - ADD, SUB, AND, ORR, SWD
		else if ((opcode == `OPCODE_RTYPE && (func_code == `FUNC_ADD || func_code == `FUNC_SUB || func_code == `FUNC_AND || func_code == `FUNC_ORR)) || opcode == `OPCODE_SWD) begin
			// $rs dependence check
			if (rs == destM && RegWrite_M) forwardSrcA <= 2'd2; // $rs dependent with instruction in MEM stage
			else if (rs == destWB && RegWrite_WB) forwardSrcA <= 2'd3; // $rs dependent with instruction in WB stage
			else forwardSrcA <= 2'd0; // $rs not dependent with any instruction

			// $rt dependence check
			if (rt == destM && RegWrite_M) forwardSrcB <= 2'd2; // $rt dependent with instruction in MEM stage
			else if (rt == destWB && RegWrite_WB) forwardSrcB <= 2'd3; // $rt dependent with instruction in WB stage
			else forwardSrcB <= 2'd0; // $rt not dependent with any instruction
			
			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				2'b10 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111; // stall PC
				2'b11 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
			endcase
			{isStall, btbWrite, btbSrc, flush, isPredict} <= {1'd0, 1'd0, 2'd0, 1'd0, 1'd0};
		end

		// use both $rs, $rt - BNE, BEQ
		else if (opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ) begin
			// $rs dependence check
			if (rs == destM && RegWrite_M) forwardSrcA <= 2'd2; // $rs dependent with instruction in MEM stage
			else if (rs == destWB && RegWrite_WB) forwardSrcA <= 2'd3; // $rs dependent with instruction in WB stage
			else forwardSrcA <= 2'd0; // $rs not dependent with any instruction

			// $rt dependence check
			if (rt == destM && RegWrite_M) forwardSrcB <= 2'd2; // $rs dependent with instruction in MEM stage
			else if (rt == destWB && RegWrite_WB) forwardSrcB <= 2'd3; // $rs dependent with instruction in WB stage
			else forwardSrcB <= 2'd0; // $rs not dependent with any instruction

			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : begin
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
					{btbWrite, btbSrc, flush} <= {1'd0, 2'd0, 1'd0};
				end
				2'b10 : begin
					// branch taken -> Write brTarget to BTB
					if (bcond) begin
						{btbWrite, btbSrc} <= {1'd1, 2'd0};
						if (predictedPC != brTarget) begin
							flush <= 1'b1; // mispredict branch target -> flush
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= (!IFState)? 5'b00000 : 5'b11111;
						end
						else begin
							flush <= 1'b0; // mispredict branch target -> flush
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111;
						end
					end
					else begin
						{btbWrite, btbSrc} <= {1'd0, 2'd3};
						if (predictedPC != nextPC) begin
							flush <= 1'b1;
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= (!IFState)? 5'b00000 : 5'b11111;
						end
						else begin
							flush <= 1'b0;
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111;
						end
						flush <= (predictedPC != nextPC)? 1'd1 : 1'd0; // mispredict branch target -> flush 
					end
				end
				2'b11 : begin
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
				end
			endcase
			{isStall, isPredict} <= {1'd0, 1'd1};
		end

		// use only $rs - NOT, TCP, SHL, SHR, WWD, ADI, ORI, LWD
		else if ((opcode == `OPCODE_RTYPE && (func_code == `FUNC_NOT || func_code == `FUNC_TCP || func_code == `FUNC_SHL || func_code == `FUNC_SHR ||
				func_code == `FUNC_WWD)) || opcode == `OPCODE_ADI || opcode == `OPCODE_ORI || opcode == `OPCODE_LWD) begin
			// $rs dependence check
			if (rs == destM && RegWrite_M) forwardSrcA <= 2'd2; // $rs dependent with instruction in MEM stage
			else if (rs == destWB && RegWrite_WB) forwardSrcA <= 2'd3; // $rs dependent with instruction in WB stage
			else forwardSrcA <= 2'd0; // $rs not dependent with any instruction

			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				2'b10 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111; // stall PC
				2'b11 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
			endcase
			{isStall, btbWrite, btbSrc, flush, isPredict} <= {1'd0, 1'd0, 2'd0, 1'd0, 1'd0};
		end

		// use only $rs - BGZ, GLZ
		else if (opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ) begin
			// $rs dependence check
			if (rs == destM && RegWrite_M) forwardSrcA <= 2'd2; // $rs dependent with instruction in MEM stage
			else if (rs == destWB && RegWrite_WB) forwardSrcA <= 2'd3; // $rs dependent with instruction in WB stage
			else forwardSrcA <= 2'd0; // $rs not dependent with any instruction
			
			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : begin
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
					{btbWrite, btbSrc, flush} <= {1'd0, 2'd0, 1'd0};
				end
				2'b10 : begin
					// branch taken -> Write brTarget to BTB
					if (bcond) begin
						{btbWrite, btbSrc} <= {1'd1, 2'd0};
						if (predictedPC != brTarget) begin
							flush <= 1'b1; // mispredict branch target -> flush
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= (!IFState)? 5'b00000 : 5'b11111;
						end
						else begin
							flush <= 1'b0; // mispredict branch target -> flush
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111;
						end
					end
					else begin
						{btbWrite, btbSrc} <= {1'd0, 2'd3};
						if (predictedPC != nextPC) begin
							flush <= 1'b1;
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= (!IFState)? 5'b00000 : 5'b11111;
						end
						else begin
							flush <= 1'b0;
							{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111;
						end
						flush <= (predictedPC != nextPC)? 1'd1 : 1'd0; // mispredict branch target -> flush 
					end
				end
				2'b11 : begin
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
				end
			endcase
			{isStall, isPredict} <= {1'd0, 1'd1};
		end

		// use only $rs - JPR, JRL
		else if (opcode == `OPCODE_RTYPE && (func_code == `FUNC_JPR || func_code == `FUNC_JRL)) begin
			// $rs dependence check
			if (rs == destM && RegWrite_M) forwardSrcA <= 2'd2; // $rs dependent with instruction in MEM stage
			else if (rs == destWB && RegWrite_WB) forwardSrcA <= 2'd3; // $rs dependent with instruction in WB stage
			else forwardSrcA <= 2'd0; // $rs not dependent with any instruction

			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : begin
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				end
				2'b10 : begin 
					{btbWrite, btbSrc} <= {1'd1, 2'd1};
					if (predictedPC != jrTarget) begin
						flush <= 1'd1;
						{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= (!IFState)? 5'b00000 : 5'b11111;
					end
					else begin
						flush <= 1'd0;
						{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111;
					end
				end
				2'b11 : begin
					{btbWrite, btbSrc} <= {1'd1, 2'd1};
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
				end
			endcase
			{isStall, isPredict} <= {1'd0, 1'd1};
		end

		// no dependency - JMP, JAL
		else if (opcode == `OPCODE_JMP || opcode == `OPCODE_JAL) begin
			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : begin
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				end
				2'b10 : begin 
					{btbWrite, btbSrc} <= {1'd1, 2'd2};
					if (predictedPC != jumpAddr) begin
						flush <= 1'b1;
						{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= (!IFState)? 5'b00000 : 5'b11111;
					end
					else begin
						flush <= 1'b0;
						{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111;
					end
				end
				2'b11 : begin
					{btbWrite, btbSrc} <= {1'd1, 2'd2};
					{PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
				end
			endcase
			{isStall, isPredict} <= {1'd0, 1'd1};
		end
		// no dependency - LHI
		else if (opcode == `OPCODE_LHI) begin
			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				2'b10 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111; // stall PC
				2'b11 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
			endcase
			{isStall, btbWrite, btbSrc, flush, isPredict} <= {1'd0, 1'd0, 2'd0, 1'd0, 1'd0};
		end
		// no dependency - HLT
		else begin 
			// update stage write signals according to MState, IFState
			casex ({MState, IFState})
				2'b0x : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00000; // stall all
				2'b10 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b00111; // stall PC
				2'b11 : {PCWrite, IFWrite, IDWrite, EXWrite, MWrite} <= 5'b11111; // no stall
			endcase
			{isStall, btbWrite, btbSrc, flush, isPredict} <= {1'd0, 1'd0, 2'd0, 1'd1, 1'd0}; // flush = 1
		end
	end
endmodule