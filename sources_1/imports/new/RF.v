`define WORD_SIZE 16    // data and address word size
module RF(
    input write, // write enable signal
    input clk, // clock signal
    input reset_n, // low-active reset signal
    input [1:0] addr1, // 1st read address
    input [1:0] addr2, // 2nd read address
    input [1:0] addr_w, // write address
    output [`WORD_SIZE-1:0] data1, // data from 1st read address
    output [`WORD_SIZE-1:0] data2, // data from 2nd read address
    input [`WORD_SIZE-1:0] data_w // data to write in RF
    );
	// Register array for storing data
	reg [`WORD_SIZE-1:0] RF [3:0]; 

	// Reading process
	assign data1 = RF[addr1];
	assign data2 = RF[addr2];

	// Writing process 
	always @(posedge clk, negedge reset_n) begin
		// If reset_n is low, all of RF vectors set to 0
		if (!reset_n) begin
			{RF[3], RF[2], RF[1], RF[0]} = {4{16'd0}};
		end
		else begin
			// Only when clk is positive edge and write is high, we can write data3 to RF according to addr3
			if (write) begin
				RF[addr_w] = data_w;
			end
		end
	end
endmodule

