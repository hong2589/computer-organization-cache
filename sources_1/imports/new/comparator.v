`include "opcodes.v"
`define WORD_SIZE 16
module comparator(
	input [`WORD_SIZE-1:0] A, // 1st operand
	input [`WORD_SIZE-1:0] B, // 2nd operand
	input [3:0] opcode, // opcode from datapath
	output bcond // branch condition
);
	reg bcond;
	always @(*) begin
		case(opcode)
			`OPCODE_BNE: bcond = (A != B)? 1'd1 : 1'd0;
			`OPCODE_BEQ: bcond = (A == B)? 1'd1 : 1'd0;
			`OPCODE_BGZ: begin
				if (A[`WORD_SIZE-1] == 1'b1) bcond = 1'd0; // A < 0
				else if (A == `WORD_SIZE'b0) bcond = 1'd0; // A == 0
				else bcond = 1'd1; // A > 0
			end
			`OPCODE_BLZ: begin
				if (A[`WORD_SIZE-1] == 1'b1) bcond = 1'd1; // A < 0
				else if (A == `WORD_SIZE'b0) bcond = 1'd0; // A == 0
				else bcond = 1'd0; // A > 0
			end
			default: bcond = 1'd0;			
		endcase
	end
endmodule