`include "opcodes.v"
`define WORD_SIZE 16    // data and address word size
module ALU(
	input [`WORD_SIZE-1:0] A, // operand 1
	input [`WORD_SIZE-1:0] B, // operand 2
	input [3:0] OP, // ALU opcode
	output [`WORD_SIZE-1:0] C // ALU output
	);
	
	reg [`WORD_SIZE-1:0] C; // reg for ALU output C
	reg Cout; // register for saving overflow bit

	always @(*) begin
		case(OP) 
			// Compute ALU operation
			`ALU_ADD: {Cout, C} = $signed(A) + $signed(B); // signed ADD
			`ALU_UNSIGNED_ADD: C = $signed({1'b0, A}) + $signed(B); // ADD between unsgined variable and signed variable
			`ALU_SUB: {Cout, C} = $signed(A) - $signed(B); // signed SUB
			`ALU_AND: C = A & B; // bitwise AND
			`ALU_ORR: C = A | B; // bitwise OR
			`ALU_NOT: C = ~A; // bitwise NOT
			`ALU_TCP: C = ~A + `WORD_SIZE'd1; // two's complement
			`ALU_SHL: C = {A[`WORD_SIZE-2:0], 1'b0}; // shift left
			`ALU_SHR: C = {A[`WORD_SIZE-1], A[`WORD_SIZE-1:1]}; // shift right
			`ALU_ID_A: C = A; // identity A : just output A
			`ALU_ID_B: C = B; // identity B : just output B
			default: C = A; 
		endcase
	end
endmodule