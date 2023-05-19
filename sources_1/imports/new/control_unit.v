`include "opcodes.v"
`define WORD_SIZE 16
module control_unit (
	input clk, // clock signal
    input reset_n, // active-low RESET signal
    input [3:0] opcode, // receive opcode from control_unit
    input [5:0] func_code, // receive func_code from control_unit
	input bcond, // branch condition
	input IDWrite, // ID/EX write enable signal
	input EXWrite, // EX/MEM write enable signal
	input MWrite, // MEM/WB write enable signal
	input IFState, // I-memory access state in IF stage from datapath
	input MState, // D-memory access state in MEM stage from datapath
	input [3:0] opcode_M, // opcode in MEM stage
	input [3:0] opcode_WB, // opcode in WB stage

    output [`WORD_SIZE-1:0] num_inst, // number of instruction during execution

	// control signals transferred to datapath
	// IF
	output i_writeM, // write enable signal to i-mem
	output i_readM, // read enable signal to i-mem
    
	// ID
	output isWWD_EX,
	output isWWD_M,
	output isWWD_WB, // assert output_port = RF[$0]

	// EX
	output [1:0] RegDst, // RF address to write data. 0: rt, 1: rd, 2: $2
	output [3:0] ALUOp, // opcode for ALU. It's defined at opcodes.v
	output [1:0] ALUSrcB, // select 2nd source of ALU. 0: RF_B, 1: sign_immediate, 2: LHI_immediate

	// MEM
	output d_writeM, // write signal to data mermory interface
	output d_readM, // read signal to data memory interface

	// WB
	output RegWrite, // write signal to RF
	output [1:0] WBSrc, // select data to write back into RF. 0: lwData(LWD), 1: wbData, 2: PC_WB(for JPL, JRL)
	output is_halted,// 1 if the cpu is halted

	// transfer control signal to Hazard_control
	output RegWrite_EX, // RegWrite in ID/EX
	output RegWrite_M, // RegWrite in EX/MEM
	output RegWrite_WB // RegWrite in MEM/WB
); 
	// output register
	reg [`WORD_SIZE-1:0] num_inst;
	reg is_halted;

	// signal from control_unit to memory
	reg i_readM; 
	reg i_writeM;
	reg d_readM;
	reg d_writeM;

	// register for remembering previous opcode_WB
	reg [3:0] previous_opcode_WB_reg;

	// ID/EX control latching
	reg isWWD_EX;
	reg [1:0] RegDst_EX; 
	reg [3:0] ALUOp_EX; 
	reg [1:0] ALUSrcB_EX; 
	reg RegWrite_EX; 
	reg [1:0] WBSrc_EX;
	reg is_halted_EX;

	// EX/MEM control latching
	reg isWWD_M;
	reg RegWrite_M; 
	reg [1:0] WBSrc_M;
	reg is_halted_M;

	// MEM/WB control latching
	reg isWWD_WB;
	reg RegWrite_WB; 
	reg [1:0] WBSrc_WB;
	reg is_halted_WB;

	// assignment for output control signal
	assign RegDst = RegDst_EX; 
	assign ALUOp = ALUOp_EX;
	assign ALUSrcB = ALUSrcB_EX;
	assign RegWrite = RegWrite_WB;
	assign WBSrc = WBSrc_WB;

	// update num_inst, is_halted
	always @(posedge clk, negedge reset_n) begin
		if (reset_n == 1'd0) begin
			num_inst <= `WORD_SIZE'd0;
			is_halted <= 1'd0;
		end
		else begin
			// the instruction in WB is not flushed(null) instruction && different with previous value -> add num_inst by 1
			if (opcode_WB != `OPCODE_FLUSH && opcode_WB != previous_opcode_WB_reg) num_inst <= num_inst + `WORD_SIZE'd1;
			previous_opcode_WB_reg <= opcode_WB; // previous opcode_WB
			is_halted <= is_halted_WB;
		end
	end

	// update i_readM, i_writeM, d_readM, d_writeM
	always @(*) begin
		if (!reset_n) begin
			i_writeM <= 1'd0;
			i_readM <= 1'd1;
			d_readM <= 1'd0;
			d_writeM <= 1'd0;
		end
		else begin
			i_readM <= ~IFState;
			if (opcode_M == `OPCODE_LWD) d_readM <= ~MState;
			if (opcode_M == `OPCODE_SWD) d_writeM <= ~MState;
		end
	end

	// ID -> EX
	always @(posedge clk, negedge reset_n) begin
		if (reset_n == 1'd0) begin
			{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
				<= {1'd0, 2'd0, `ALU_ID_A, 2'd1, 1'd0, 2'd1, 1'd0};
			num_inst <= `WORD_SIZE'd0;		
			RegWrite_EX <= 1'd0;
		end
		else begin
			if (opcode == `OPCODE_FLUSH) begin
				{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
					<= {1'd0, 2'd0, `ALU_ID_A, 2'd1, 1'd0, 2'd1, 1'd0};				
			end
			// transfer control signals to EX stage only when IDWrite == 1
			else if (IDWrite) begin
				case(opcode) 
					// set control signals according to opcode, func_code
					// ADD, SUB, AND, ORR, NOT, TCP, SHL, SHR, WWD, JPR, JRL, HLT
					`OPCODE_RTYPE : begin
						case(func_code)
							`FUNC_ADD : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_ADD, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_SUB : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_SUB, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_AND : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_AND, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_ORR : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_ORR, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_NOT : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_NOT, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_TCP : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_TCP, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_SHL : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_SHL, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_SHR : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd1, `ALU_SHR, 2'd0, 1'd1, 2'd1, 1'd0};
							end
							`FUNC_WWD : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd1, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd0}; 
							end
							`FUNC_JPR : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd0};
							end
							`FUNC_JRL : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd2, `ALU_ID_A, 2'd0, 1'd1, 2'd2, 1'd0};
							end
							`FUNC_HLT : begin
								{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
									<= {1'd0, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd1}; // is_halted_EX = 1'd1
							end 
						endcase
					end
					`OPCODE_ADI : begin // ADI
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ADD, 2'd1, 1'd1, 2'd1, 1'd0};
					end
					`OPCODE_ORI : begin // ORI
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ORR, 2'd3, 1'd1, 2'd1, 1'd0};
					end
					`OPCODE_LHI : begin // LHI
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ID_B, 2'd2, 1'd1, 2'd1, 1'd0};
					end
					`OPCODE_LWD : begin // LWD
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ADD, 2'd1, 1'd1, 2'd0, 1'd0};
					end
					`OPCODE_SWD : begin // SWD
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ADD, 2'd1, 1'd0, 2'd0, 1'd0};
					end
					`OPCODE_BNE : begin // BNE
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd0};
					end
					`OPCODE_BEQ : begin // BEQ
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd0};
					end
					`OPCODE_BGZ : begin // BGZ
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd0};
					end
					`OPCODE_BLZ : begin // BLZ
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd0};
					end
					`OPCODE_JMP : begin // JMP
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd0, `ALU_ID_A, 2'd0, 1'd0, 2'd1, 1'd0};
					end
					`OPCODE_JAL : begin // JAL
						{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, RegWrite_EX, WBSrc_EX, is_halted_EX} 
							<= {1'd0, 2'd2, `ALU_ID_A, 2'd0, 1'd1, 2'd2, 1'd0}; // RegDst_EX = 2'd2, RegWrite_EX = 1'd1, WBSrc_EX = 2'd2
					end
				endcase
			end
			else begin // IDWrite==0 -> maintain current ID/EX control signals
				{isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, WBSrc_EX, is_halted_EX} 
					<= {isWWD_EX, RegDst_EX, ALUOp_EX, ALUSrcB_EX, WBSrc_EX, is_halted_EX};
				RegWrite_EX <= RegWrite_EX;
			end
		end
	end

	// EX -> MEM
	always @(posedge clk, negedge reset_n) begin
		if (reset_n == 1'd0) begin
			{RegWrite_M, WBSrc_M, is_halted_M, isWWD_M} <= {1'd0, 2'd1, 1'd0, 1'd0};
		end
		else begin
			if (EXWrite) begin // transfer control signals from ID/EX latches
				{RegWrite_M, WBSrc_M, is_halted_M, isWWD_M} <= {RegWrite_EX, WBSrc_EX, is_halted_EX, isWWD_EX};
			end
			else begin // EXWrite==0 -> maintain current EX/M control signals
				{RegWrite_M, WBSrc_M, is_halted_M, isWWD_M} <= {RegWrite_M, WBSrc_M, is_halted_M, isWWD_M};
			end
		end
	end

	// MEM -> WB
	always @(posedge clk, negedge reset_n) begin
		if (reset_n == 1'd0) begin
			{RegWrite_WB, WBSrc_WB, is_halted_WB, isWWD_WB} <= {1'd0, 2'd1, 1'd0, 1'd0};
		end
		else begin
			if (MWrite) begin //transfer control signals from EX/MEM latches
				{RegWrite_WB, WBSrc_WB, is_halted_WB, isWWD_WB} <= {RegWrite_M, WBSrc_M, is_halted_M, isWWD_M};
			end
			else begin // MWrite==0 -> maintain current M/WB control signals
				{RegWrite_WB, WBSrc_WB, is_halted_WB} <= {RegWrite_WB, WBSrc_WB, is_halted_WB};
				isWWD_WB <= 1'b0;
			end
		end
	end

endmodule