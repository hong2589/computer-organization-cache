module datapath (
	input clk,
	input reset_n,
	output [3:0] opcode,
	output [5:0] func_code,

	// interface with memory
	output [`WORD_SIZE-1:0] i_address, // instruction memory address
	inout [`WORD_SIZE-1:0] i_data, // data from I-memory(instruction)
	output [`WORD_SIZE-1:0] d_address, // data memory address
	inout [`WORD_SIZE-1:0] d_data, // data from D-memory

	// output_port for WWD instruction
	output [`WORD_SIZE-1:0] output_port,

	// control signal from control_unit
	input isWWD_WB, // assert output_port = RF[$0]
	input [1:0] RegDst, // RF address to write data. 0: rt, 1: rd, 2: $2
	input [3:0] ALUOp, // opcode for ALU. It's defined at opcodes.v
	input [1:0] ALUSrcB, // select 2nd source of ALU. 0: RF_B, 1: sign_immediate, 2: LHI_immediate
	input i_writeM, // write signal for i-mem
	input i_readM, // read signal for i-mem
	input d_writeM, // write signal to data mermory interface
	input d_readM, // read signal to data memory interface
	input RegWrite, // write signal to RF
	input [1:0] WBSrc, // select data to write back into RF. 0: lwData(LWD), 1: wbData, 2: PC_WB(for JPL, JRL)

	// input signal from hazard control unit
	input PCWrite, // PCWrite enable signal
	input IFWrite, // IF/ID update enable signal
	input IDWrite, // ID/EX update enable signal
	input EXWrite, // EX/MEM update enable signal
	input MWrite, // MEM/WB update enable signal
	input [1:0] btbSrc, // select signal for BTB update value
	input btbWrite, // BTB write enable signal
	input flush, // flush signal to disenable all the control signal from EX.
	input [1:0] forwardSrcA, // signal to select forwardTargetA
	input [1:0] forwardSrcB, // signal to select forwardTargetB
	input isPredict, // when the instruction in ID stage is branch or jump -> isPredict=1

	// output signal to be transferred to hazard control unit
	output [1:0] rs, // rs address in ID stage
	output [1:0] rt, // rt address in ID stage
	output [1:0] destEX, // destination(to write back) RF address in EX stage
	output [1:0] destM, // destination(to write back) RF address in EX stage
	output [1:0] destWB, // destination(to write back) RF address in EX stage
	output bcond, // 0: actually branch not taken, 1: actually branch taken
	output [`WORD_SIZE-1:0] brTarget, // branch target addresss
	output [`WORD_SIZE-1:0] jrTarget, // JR target address
	output [`WORD_SIZE-1:0] jumpAddr, // jump address
	output [`WORD_SIZE-1:0] pcTarget, // pcTarget to be determined btbSrc
	output [`WORD_SIZE-1:0] predictedPC, // predictedPC to be determined tag hit
	output [`WORD_SIZE-1:0] nextPC, // PC + 1
	output [3:0] opcode_EX, // opcode in EX stage
	output [3:0] opcode_M, // opcode in MEM stage
	output [3:0] opcode_WB, // opcode in WB stage
	output IFState, // I-memory access state in IF stage from datapath
	output MState // D-memory access state in MEM stage from datapath
);

	// state for I-memory, D-memory access
	reg IFState; // state for I-memory
	reg MState; // state for D-memory

	// output registers
	reg [`WORD_SIZE-1:0] output_port;

	// forwarding target
	wire [`WORD_SIZE-1:0] forwardTarget_EX;
	wire [`WORD_SIZE-1:0] forwardTarget_M;
	wire [`WORD_SIZE-1:0] forwardTarget_WB;
	wire [`WORD_SIZE-1:0] RF_A_target;
	wire [`WORD_SIZE-1:0] RF_B_target;

	// IF stage variables
	parameter tagLength = 8;
	parameter btbSize = 256;
	reg [`WORD_SIZE-1:0] PC_reg; // current PC
	wire [`WORD_SIZE-1:0] PC;
	wire predictTaken;
	wire [`WORD_SIZE-1:0] predictResult;
	wire [`WORD_SIZE-1:0] selectedPC;
	wire [`WORD_SIZE-1:0] PC_add_1; // PC + 1

	// ID stage variables
	reg [`WORD_SIZE-1:0] nextPC_reg; // PC + 1
	reg [`WORD_SIZE-1:0] predictedPC_reg; // predictedPC from control_predictor
	reg [`WORD_SIZE-1:0] originalPC_reg; // PC
	reg [`WORD_SIZE-1:0] inst_reg; // instruction register
	wire [`WORD_SIZE-1:0] nextPC;
	wire [1:0] rs; wire[1:0] rt; wire[1:0] rd;
	wire [1:0] rfAddr_1; // 1st register address to read 
    wire [1:0] rfAddr_2; // 2nd register address to read
    wire [`WORD_SIZE-1:0] rfData_1; // data stored in RF[regAddr_1]
    wire [`WORD_SIZE-1:0] rfData_2; // data stored in RF[regAddr_2]
	wire [`WORD_SIZE-1:0] brTarget; // actual branch target address
	wire [`WORD_SIZE-1:0] jumpAddr; // actual jump target address
	
	// EX stage variables
	reg [`WORD_SIZE-1:0] PC_EX_reg; // PC+1 latching for JAL, JRL
	reg [`WORD_SIZE-1:0] RF_A_reg; // RF[$rs] 
	reg [`WORD_SIZE-1:0] RF_B_reg; // RF[$rt]
	reg [`WORD_SIZE-1:0] sign_immediate_reg; // sign_extended immediate(16bits)	
	reg [`WORD_SIZE-1:0] LHI_immediate_reg; // Left_shifted immediate(16bits) 
	reg [`WORD_SIZE-1:0] ORI_immediate_reg; // concatenated with inst like {8'h00, inst[7:0]}
	reg [1:0] rs_reg; reg [1:0] rt_reg; reg [1:0] rd_reg; // rs, rt, rd address
	reg [`WORD_SIZE-1:0] output_port_EX_reg; // latching output_port in ID/EX
	reg [3:0] opcode_EX_reg;
	reg [5:0] func_code_EX_reg;
	wire [`WORD_SIZE-1:0] originalPC;
	wire [`WORD_SIZE-1:0] predictedPC;
	wire [`WORD_SIZE-1:0] sign_immediate;
	wire [`WORD_SIZE-1:0] LHI_immediate;
	wire [`WORD_SIZE-1:0] ORI_immediate;
	wire [1:0] destEX;
	wire [`WORD_SIZE-1:0] output_port_EX;
	wire [3:0] opcode_EX;
	wire [5:0] func_code_EX;

	// MEM stage variables
	reg [`WORD_SIZE-1:0] PC_M_reg; // PC + 1 latching for JAL, JRL
	reg [`WORD_SIZE-1:0] ALUOut_reg; // latching result of ALU_main
	reg [`WORD_SIZE-1:0] swData_reg; // data to store in memory
	reg [1:0] destM_reg; // WB destination address
	reg [`WORD_SIZE-1:0] output_port_M_reg; // latching output_port in EX->MEM
	reg [3:0] opcode_M_reg;
	reg [5:0] func_code_M_reg;
	wire [`WORD_SIZE-1:0] ALUSrcA_main;
	wire [`WORD_SIZE-1:0] ALUSrcB_main;
	wire [`WORD_SIZE-1:0] ALUResult_main;
	wire [`WORD_SIZE-1:0] output_port_M;
	wire [5:0] func_code_M;
	wire [1:0] destM;

	// WB stage variables
	reg [`WORD_SIZE-1:0] PC_WB_reg; // PC + 1 latching for JAL, JRL
	reg [`WORD_SIZE-1:0] lwData_reg; // data from d-mem in SWD instruction
	reg [`WORD_SIZE-1:0] wbData_reg; // data to write back to the RF
	reg [1:0] destWB_reg; // WB destination address
	reg [`WORD_SIZE-1:0] output_port_WB_reg; //latching output_port in MEM -> WB
	reg [3:0] opcode_WB_reg;
	reg [5:0] func_code_WB_reg;
	wire [`WORD_SIZE-1:0] rfData_w;  // data to store into RF[regAddr_w]
	wire [1:0] destWB;
	wire [`WORD_SIZE-1:0] output_port_WB;
	wire [5:0] func_code_WB;

	// assignment of forwarding target
	assign forwardTarget_EX = (opcode_EX == `OPCODE_JAL || (opcode_EX == `OPCODE_RTYPE && func_code_EX == `FUNC_JRL))? PC_EX_reg : ALUResult_main;
	assign forwardTarget_M = (opcode_M == `OPCODE_LWD == 1'b1)? d_data :
							 (opcode_M == `OPCODE_JAL || (opcode_M == `OPCODE_RTYPE && func_code_M == `FUNC_JRL))? PC_M_reg : ALUOut_reg;
	assign forwardTarget_WB = (WBSrc == 2'd0)? lwData_reg :
							  (WBSrc == 2'd1)? wbData_reg :
							  (WBSrc == 2'd2)? PC_WB_reg : `WORD_SIZE'bz;

	// assign RF target according to forwardSrcA, forwardSrcB
	assign RF_A_target = (forwardSrcA == 2'd0)? rfData_1 :
						 (forwardSrcA == 2'd1)? forwardTarget_EX :
						 (forwardSrcA == 2'd2)? forwardTarget_M :
						 (forwardSrcA == 2'd3)? forwardTarget_WB : `WORD_SIZE'd0;
	assign RF_B_target = (forwardSrcB == 2'd0)? rfData_2 :
						 (forwardSrcB == 2'd1)? forwardTarget_EX :
						 (forwardSrcB == 2'd2)? forwardTarget_M :
						 (forwardSrcB == 2'd3)? forwardTarget_WB : `WORD_SIZE'd0;

	// 1. Data flow in IF stage
	assign PC = PC_reg;
	assign nextPC = nextPC_reg;
	assign i_address = PC_reg;
	assign selectedPC = (predictTaken)? predictResult : PC_add_1;

	// ALU for PC + 1 
	// It add 1 to PC, caculate next instruction address
	ALU ALU_addPC(.A(PC), .B(`WORD_SIZE'd1), .OP(`ALU_UNSIGNED_ADD), .C(PC_add_1));

	// control predictor unit - always taken version (BTB + tag table)
	// If branch/jump instruction was executed before, control_predictor predict the branch/jump address(predictResult) from its BTB.
	// It is controlled by hazard_control by btbWrite signal.
	// If predictTaken == 1, then datapath selects PC source to predictResult from this module
	control_predictor #(.tagLength(tagLength), .btbSize(btbSize)) 
	CP(
	  .clk(clk),
	  .reset_n(reset_n),
	  .readPC(PC),
	  .writePC(originalPC),
	  .pcTarget(pcTarget),
	  .btbWrite(btbWrite),
	  .flush(flush),
	  .isPredict(isPredict),
	  .predictResult(predictResult),
	  .predictTaken(predictTaken)
	);
	
	// 2. Data flow in ID stage
	assign {opcode, func_code} = {inst_reg[15:12], inst_reg[5:0]}; 
	assign {originalPC, predictedPC} = {originalPC_reg, predictedPC_reg};
	assign {sign_immediate, LHI_immediate, ORI_immediate} = {{{8{inst_reg[7]}}, inst_reg[7:0]}, {inst_reg[7:0], 8'd0}, {8'd0, inst_reg[7:0]}};
	assign {rs, rt, rd} = {inst_reg[11:10], inst_reg[9:8], inst_reg[7:6]};
	assign {rfAddr_1, rfAddr_2} = {inst_reg[11:10], inst_reg[9:8]};
	assign pcTarget = (btbSrc == 2'd0)? brTarget : 
					  (btbSrc == 2'd1)? jrTarget : 
					  (btbSrc == 2'd2)? jumpAddr :
					  (btbSrc == 2'd3)? nextPC : pcTarget;
	assign {jumpAddr, jrTarget} = {{PC[15:12], inst_reg[11:0]}, RF_A_target};

	// ALU for branch target address(brTarget)
	// It calculates branch target address by nextPC + sign_immediate
	ALU ALU_brTarget(.A(nextPC), .B(sign_immediate), .OP(`ALU_UNSIGNED_ADD), .C(brTarget));

	// RF
	// This module manages 4 registers
	// using asynchronous read, synchronous write
	RF rf (.write(RegWrite), .clk(clk), .reset_n(reset_n), .addr1(rfAddr_1), .addr2(rfAddr_2), .addr_w(destWB), .data1(rfData_1), .data2(rfData_2), .data_w(rfData_w));

	// comparator for RF[$rs], RF[$rt] to determine branch condition
	// It compare rfData_1, and rfData_2 according to opcode(BNE, BEQ, BGZ, BLZ)
	comparator comp(.A(RF_A_target), .B(RF_B_target), .opcode(opcode), .bcond(bcond));
	
	// 3. Data flow in EX stage
	assign ALUSrcA_main = RF_A_reg;
	assign ALUSrcB_main = (ALUSrcB==2'd0)? RF_B_reg :
						  (ALUSrcB==2'd1)? sign_immediate_reg :
						  (ALUSrcB==2'd2)? LHI_immediate_reg :
						  (ALUSrcB==2'd3)? ORI_immediate_reg : `WORD_SIZE'bz;
	assign destEX = (RegDst==2'd0)? rt_reg :
					(RegDst==2'd1)? rd_reg :
					(RegDst==2'd2)? 2'd2 : 2'dz;
	assign {output_port_EX, opcode_EX, func_code_EX} = {output_port_EX_reg, opcode_EX_reg, func_code_EX_reg};

	// main ALU
	// It executes the calculation according to ALUOp.
	ALU ALU_main(.A(ALUSrcA_main), .B(ALUSrcB_main), .OP(ALUOp), .C(ALUResult_main));
	
	// 4. Data flow in MEM stage
	assign d_address = ALUOut_reg;
	assign d_data = (d_writeM)? swData_reg : `WORD_SIZE'bz;
	assign {destM, output_port_M, opcode_M, func_code_M} = {destM_reg, output_port_M_reg, opcode_M_reg, func_code_M_reg};
	
	// 5. Data flow in WB stage
	assign rfData_w = (WBSrc == 2'd0)? lwData_reg :
					  (WBSrc == 2'd1)? wbData_reg :
					  (WBSrc == 2'd2)? PC_WB_reg : `WORD_SIZE'bz;
	assign {destWB, output_port_WB, opcode_WB, func_code_WB} = {destWB_reg, output_port_WB_reg, opcode_WB_reg, func_code_WB_reg};
		
	always @(posedge clk, negedge reset_n) begin
		if (reset_n==1'b0) begin
			// reset IF stage registers
			{PC_reg, inst_reg, nextPC_reg, predictedPC_reg, originalPC_reg}
				<= {`WORD_SIZE'd0, `WORD_SIZE'he000, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0};
			IFState <= 1'd0;
			
			// reset ID stage registers
			{PC_EX_reg, RF_A_reg, RF_B_reg, sign_immediate_reg, LHI_immediate_reg, ORI_immediate_reg, output_port_EX_reg}
				<= {`WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0};
			{rs_reg, rt_reg, rd_reg} <= 6'd0;
			{opcode_EX_reg, func_code_EX_reg} <= {`OPCODE_FLUSH, 6'd0};

			// reset EX stage registers
			{PC_M_reg, ALUOut_reg, swData_reg, destM_reg, output_port_M_reg, opcode_M_reg, func_code_M_reg}
				<= {`WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, 2'd0, `WORD_SIZE'd0, `OPCODE_FLUSH, 6'd0};

			// reset MEM stage registers
			{PC_WB_reg, lwData_reg, wbData_reg, destWB_reg, output_port_WB_reg, opcode_WB_reg, func_code_WB_reg}
				<= {`WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, 2'd0, `WORD_SIZE'd0, `OPCODE_FLUSH, 6'd0};
			MState <= 1'd1;

			// reset output_port
			output_port <= `WORD_SIZE'd0;
		end
		else begin
			// PC
			IFState <= (IFState || !MState)? 1'b0 : 1'b1; // update IFState
			if (PCWrite) PC_reg <= (flush)? pcTarget : selectedPC; // update PC only when previous state == 1
			
			// IF -> ID
			if (IFWrite) begin
				{nextPC_reg, predictedPC_reg, originalPC_reg} <= {PC_add_1, selectedPC, PC};
				inst_reg <= (flush)? `WORD_SIZE'he000 : i_data;
			end
			else begin
				inst_reg <= (!MWrite)? inst_reg : `WORD_SIZE'he000;
			end

			// ID -> EX : update only when IDWrite == 1(not stall)
			if (IDWrite) begin
				// forwarding data to RF_A_reg, RF_B_reg according to forwardSrcA, forwardSrcB from hazard_control
				{RF_A_reg, RF_B_reg, output_port_EX_reg} <= {RF_A_target, RF_B_target, RF_A_target};
				{PC_EX_reg, sign_immediate_reg, LHI_immediate_reg, ORI_immediate_reg} <= {nextPC, sign_immediate, LHI_immediate, ORI_immediate};
				{rs_reg, rt_reg, rd_reg, opcode_EX_reg, func_code_EX_reg} <= {rs, rt, rd, opcode, func_code};
			end
			else begin
				{RF_A_reg, RF_B_reg, output_port_EX_reg} <= {RF_A_reg, RF_B_reg, output_port};
			end

			// EX -> MEM : update only when EXWrite == 1(not stall)	
			MState <= ((opcode_EX == `OPCODE_LWD || opcode_EX == `OPCODE_SWD) && MState == 1'b1)? 1'b0 : 1'b1; // update MState
			if (EXWrite) begin
				{PC_M_reg, ALUOut_reg, swData_reg, destM_reg, output_port_M_reg, opcode_M_reg, func_code_M_reg}
					<= {PC_EX_reg, ALUResult_main, RF_B_reg, destEX, output_port_EX_reg, opcode_EX_reg, func_code_EX_reg};
			end

			// MEM -> WB : update only when MWrite == 1(not stall)
			if (MWrite) begin
				{PC_WB_reg, lwData_reg, wbData_reg, destWB_reg, output_port_WB_reg, opcode_WB_reg, func_code_WB_reg}
					<= {PC_M_reg, d_data, ALUOut_reg, destM, output_port_M, opcode_M_reg, func_code_M_reg};
			end

			// update output_port only when isWWD_WB == 1 (isWWD_WB is the signal indicating WWD instruction in WB stage)
			output_port <= (isWWD_WB)? output_port_WB_reg : output_port;
		end
	end
endmodule