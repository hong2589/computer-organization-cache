module control_predictor #(parameter tagLength = 8, parameter btbSize = 256)
(
	input clk, // clock
	input reset_n, // reset
	input [`WORD_SIZE-1:0] readPC, // current PC value in IF stage
	input [`WORD_SIZE-1:0] writePC, // address to modify BTB
	input [`WORD_SIZE-1:0] pcTarget, // PC target value to modify BTB
	input btbWrite, // BTB write enable signal
	input flush, // flush indicator to update BHT
	input isPredict, // when the instruction in ID stage is branch or jump -> isPredict=1
	output [`WORD_SIZE-1:0] predictResult, // Result of branch/jump prediction
	output predictTaken // prediction result : taken or not taken
);
	// output registers
	reg [`WORD_SIZE-1:0] predictResult;
	reg tag_hit;

	// BTB, tagTable
	reg [`WORD_SIZE-1:0] BTB [btbSize-1:0];
	reg [tagLength-1:0] tagTable [btbSize-1:0];
	integer i;

	// BHT - 2 bit saturation 
	// 11: strongly taken, 10: weakly taken, 01: weakly not taken, 00: strongly not taken
	reg predictTaken;
	reg [1:0] BHT[btbSize-1:0];
	parameter ST = 3; // strongly taken
	parameter WT = 2; // weakly taken
	parameter WNT = 1; // weakly not taken
	parameter SNT = 0; // strongly not taken

	// flush, predict counter for accuracy measurement
	reg [`WORD_SIZE-1:0] flushCnt;
	reg [`WORD_SIZE-1:0] predictCnt;
	
	// predict branch / jump
	always @(*) begin
		if (reset_n == 1'b0) begin
			// reset {tagTable, BTB} to {8'hff, 16'h0000}
			for (i = 0; i < btbSize; i = i + 1) begin
				BTB[i] <= `WORD_SIZE'h0000;
				tagTable[i] <= {tagLength{1'd1}};
				BHT[i] <= 2'b11; // initial BHT : strongly taken
			end
			predictResult <= 16'h0000;
			tag_hit <= 1'd0;
			
		end
		else begin
			// tag hit & BHT == ST or WT -> predict result according to BTB value
			if (tagTable[readPC[7:0]] == readPC[15:8] && (BHT[readPC[7:0]] == ST || BHT[readPC[7:0]] == WT)) begin
				predictResult <= BTB[readPC[7:0]];
				predictTaken <= 1'd1;
			end
			// tag miss -> just increase PC by 1 in datapath according to tag_hit = 0
			else begin
				predictResult <= 16'h0000;
				predictTaken <= 1'd0;
			end
		end
	end

	// Update BTB, tabTable synchronously only when btbWrite == 1
	// Update BHT
	always @(posedge clk, negedge reset_n) begin
		if (reset_n == 1'd0) begin
			flushCnt <= `WORD_SIZE'd0;
			predictCnt <= `WORD_SIZE'd0;
		end
		else begin
			flushCnt <= (flush)? flushCnt + `WORD_SIZE'd1 : flushCnt;
			predictCnt <= (isPredict)? predictCnt + `WORD_SIZE'd1 : predictCnt;
			// update BHT only when updateBHT == 1 from hazard_control
			if (isPredict) begin
				case(BHT[writePC[7:0]])
					ST : BHT[writePC[7:0]] <= (flush)? WT : ST;
					WT : BHT[writePC[7:0]] <= (flush)? WNT : ST;
					WNT : BHT[writePC[7:0]] <= (flush)? WT : SNT;
					SNT : BHT[writePC[7:0]] <= (flush)? WNT : SNT;
				endcase
			end
			if (btbWrite) begin
				BTB[writePC[7:0]] <= pcTarget;
				tagTable[writePC[7:0]] <= writePC[15:8];
			end
		end
	end
endmodule