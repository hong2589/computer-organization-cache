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
	input i_ready,
	input d_ready,

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
	output IDWrite, // IF/ID write enable signal
	output EXWrite, // ID/EX write enable signal
	output MWrite, // EX/M write enable signal
	output WBWrite, // M/WB write enable signal

	// control signals for branch/jump prediction
	output [1:0] btbSrc, // select signal for BTB update. 0 : brTarget, 1: rfData_1, 2: jumpAddr, 3: nextPC
	output btbWrite, // BTB update enable signal
	output flush, // signal to flush the mispredicted instruction in IF stage
	output isPredict, // signal to indicate branch / jump instruction in ID stage

	// control signal for forwarding
	output [1:0] forwardSrcA, // 1st forward source select signal
	output [1:0] forwardSrcB, // 2nd forward source select signal
	output flush_EX
); 
	// stage write enable IDWrite
	reg PCWrite; reg IDWrite; reg EXWrite; reg MWrite; reg WBWrite;

	// signals about control prediction
	reg [1:0] btbSrc; // select signal for btb write
	reg btbWrite; // write enable signal for BTB
	reg flush; // flush == 1 only when there is misprediction
	reg flush_EX; // flush opcode_EX
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
	wire LWD_dependence_hazard; // indicate the stall case : LWD in EX stage and data dependence exists with the instruction in ID stage
	assign {rs_dependence_EX, rs_dependence_M, rs_dependence_WB} = {(rs == destEX & RegWrite_EX), (rs == destM & RegWrite_M), (rs == destWB & RegWrite_WB)};
	assign {rt_dependence_EX, rt_dependence_M, rt_dependence_WB} = {(rt == destEX & RegWrite_EX), (rt == destM & RegWrite_M), (rt == destWB & RegWrite_WB)};
	assign use_rs_ID = (opcode == `OPCODE_RTYPE && func_code != `FUNC_HLT) || opcode == `OPCODE_SWD || opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ;
	assign use_rt_ID = (opcode == `OPCODE_RTYPE && (func_code == `FUNC_ADD || func_code == `FUNC_SUB || func_code == `FUNC_AND || func_code == `FUNC_ORR))
						|| opcode == `OPCODE_SWD || opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ;
	assign LWD_dependence_hazard = (opcode_EX == `OPCODE_LWD) && ((use_rs_ID && rs_dependence_EX) || (use_rt_ID && rt_dependence_EX));

	// assign forward source
	assign forwardSrcA = (rs_dependence_EX && use_rs_ID)? 2'd1 :
						 (rs_dependence_M && use_rs_ID)? 2'd2 :
						 (rs_dependence_WB && use_rs_ID)? 2'd3 : 2'd0;
	assign forwardSrcB = (rt_dependence_EX && use_rt_ID)? 2'd1 :
						 (rt_dependence_M && use_rt_ID)? 2'd2 :
						 (rt_dependence_WB && use_rt_ID)? 2'd3 : 2'd0;

	// update next_control_state
	always @(*) begin
		if (!reset_n) next_control_state <= RESET;
		else begin
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
	end

	// update synchronously
	always @(posedge clk, negedge reset_n) begin
		if (!reset_n) control_state <= RESET;
		else control_state <= next_control_state; // update current control_state
	end

	// update signal asynchronously
	always @(*) begin
		if (!reset_n) begin
			{PCWrite , IDWrite, EXWrite, MWrite, WBWrite} <= 5'b11111; // turn on all the write signals
			{btbSrc, btbWrite, flush, flush_EX} <= 5'b00000; // disable BTB write, reset flush to 0
			isPredict <= 1'd0;
		end
		else begin
			// operate according to control_state
			case (control_state)
				RESET : begin
					casex ({d_cache_hit, i_cache_hit})
						2'b0x : begin
							// next_control_state <= ACCESS_D;
							flush <= 1'd0; flush_EX <= 1'd0;
							{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b00000; // stall all
						end
						2'b10 : begin
							// next_control_state <= ACCESS_I;
							flush <= 1'd0; flush_EX <= 1'd0;
							{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b00111; // stall IF, ID
						end
						2'b11 : begin
							// next_control_state <= (LWD_dependence_hazard)? HAZARD_STALL : RESET;
							if (LWD_dependence_hazard) begin
								flush <= 1'd0; flush_EX <= 1'd1; // flush_EX = 1
								{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b00011; // stall IF, ID, EX
							end
							else begin
								flush_EX <= 1'd0;
								{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b11111; // no stall
								if (opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ) begin
									isPredict <= 1'd1;
									if (bcond) begin
										{btbWrite, btbSrc} <= {1'd1, 2'd0};
										flush <= (predictedPC != brTarget)? 1'd1 : 1'd0;
									end
									else begin
										{btbWrite, btbSrc} <= {1'd0, 2'd3};
										flush <= (predictedPC != nextPC)? 1'd1 : 1'd0;
									end
								end
								else if (opcode == `OPCODE_RTYPE && (func_code == `FUNC_JPR || func_code == `FUNC_JRL)) begin
									{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd1, 1'd1};
									flush <= (predictedPC != jrTarget)? 1'd1 : 1'd0;
								end
								else if (opcode == `OPCODE_JMP || opcode == `OPCODE_JAL) begin
									{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd2, 1'd1};
									flush <= (predictedPC != jumpAddr)? 1'd1 : 1'd0;
								end
								else begin
									{btbWrite, btbSrc, flush, isPredict} <= {1'd0, 2'd0, 1'd0, 1'd0};
								end
							end
						end
					endcase
				end
				ACCESS_I : begin
					if (i_ready) begin
						flush_EX <= 1'd1; // flush_EX <= 1'd1
						{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b11111; // no stall
						if (opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ) begin
							isPredict <= 1'd1;
							if (bcond) begin
								{btbWrite, btbSrc} <= {1'd1, 2'd0};
								flush <= (predictedPC != brTarget)? 1'd1 : 1'd0;
							end
							else begin
								{btbWrite, btbSrc} <= {1'd0, 2'd3};
								flush <= (predictedPC != nextPC)? 1'd1 : 1'd0;
							end
						end
						else if (opcode == `OPCODE_RTYPE && (func_code == `FUNC_JPR || func_code == `FUNC_JRL)) begin
							{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd1, 1'd1};
							flush <= (predictedPC != jrTarget)? 1'd1 : 1'd0;
						end
						else if (opcode == `OPCODE_JMP || opcode == `OPCODE_JAL) begin
							{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd2, 1'd1};
							flush <= (predictedPC != jumpAddr)? 1'd1 : 1'd0;
						end
						else begin
							{btbWrite, btbSrc, isPredict} <= {1'd0, 2'd0, 1'd0};
							flush <= 1'd0;
						end
					end
					else begin
						flush <= 1'd0; flush_EX <= 1'd1;
						{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b00111; // stall IF, ID
					end
				end
				ACCESS_D : begin 
					if (d_ready) begin
						flush_EX <= 1'd0;
						{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b11111; // no stall
						if (opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ) begin
							isPredict <= 1'd1;
							if (bcond) begin
								{btbWrite, btbSrc} <= {1'd1, 2'd0};
								flush <= (predictedPC != brTarget)? 1'd1 : 1'd0;
							end
							else begin
								{btbWrite, btbSrc} <= {1'd0, 2'd3};
								flush <= (predictedPC != nextPC)? 1'd1 : 1'd0;
							end
						end
						else if (opcode == `OPCODE_RTYPE && (func_code == `FUNC_JPR || func_code == `FUNC_JRL)) begin
							{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd1, 1'd1};
							flush <= (predictedPC != jrTarget)? 1'd1 : 1'd0;
						end
						else if (opcode == `OPCODE_JMP || opcode == `OPCODE_JAL) begin
							{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd2, 1'd1};
							flush <= (predictedPC != jumpAddr)? 1'd1 : 1'd0;
						end
						else begin
							{btbWrite, btbSrc, isPredict} <= {1'd0, 2'd0, 1'd0};
							flush <= 1'd0;
						end
					end
					else begin
						flush <= 1'd0; flush_EX <= 1'd0;
						{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b00000; // stall all
					end
				end
				HAZARD_STALL : begin 
					{PCWrite, IDWrite, EXWrite, MWrite, WBWrite} <= 5'b11111; // no stall
					flush_EX <= 1'd0;
					if (opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ) begin
						isPredict <= 1'd1;
						if (bcond) begin
							{btbWrite, btbSrc} <= {1'd1, 2'd0};
							flush <= (predictedPC != brTarget)? 1'd1 : 1'd0;
						end
						else begin
							{btbWrite, btbSrc} <= {1'd0, 2'd3};
							flush <= (predictedPC != nextPC)? 1'd1 : 1'd0;
						end
					end
					else if (opcode == `OPCODE_RTYPE && (func_code == `FUNC_JPR || func_code == `FUNC_JRL)) begin
						{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd1, 1'd1};
						flush <= (predictedPC != jrTarget)? 1'd1 : 1'd0;
					end
					else if (opcode == `OPCODE_JMP || opcode == `OPCODE_JAL) begin
						{btbWrite, btbSrc, isPredict} <= {1'd1, 2'd2, 1'd1};
						flush <= (predictedPC != jumpAddr)? 1'd1 : 1'd0;
					end
					else begin
						{btbWrite, btbSrc, flush, isPredict} <= {1'd0, 2'd0, 1'd0, 1'd0};
					end
				end
			endcase
		end
	end
endmodule


