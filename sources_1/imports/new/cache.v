`include "opcodes.v"
`define WORD_SIZE 16    // data and address word size
`define FETCH_SIZE 64 // fetch size from memory (4 words = 64bits)
module cache(
	input clk,
	input reset_n,

	// interface between datapath and I-cache
	input IFState, // IF state in datapath. 0: wait, 1: fetch
	input i_readC, // read signal for i-mem
	input i_writeC // write signal for i-mem
	input [`WORD_SIZE-1:0] i_addressC, // memory address to fetch instruction
	output [`WORD_SIZE-1:0] i_dataC, // instruction data

	// interface between datapath and D-cache
	input MState,
	input d_readC,
	input d_writeC,
	input [`WORD_SIZE-1:0] d_addressC,
	inout [`WORD_SIZE-1:0] d_dataC,

	// interface wires with I-memory
	output i_readM,
	output i_writeM,
	output [`WORD_SIZE-1:0] i_addressM,
	inout [`FETCH_SIZE-1:0] i_dataM,

	//interface wires with D-memory
	output d_readM,
	output d_writeM,
	output [`WORD_SIZE-1:0] d_addressM,
	inout [`FETCH_SIZE-1:0] d_dataM,

	// cache hit
	output i_cache_hit,
	output d_cache_hit
);
	parameter tagSize = 12; // bit size of tag field in 16 bit address
	parameter blockNum = 4; // # of cache block
	parameter blockSize = 64; // Size of one cache block = 4 words(64bits)
	reg [`WORD_SIZE-1:0] i_hitCnt; // counter for I-cache hit
	reg [`WORD_SIZE-1:0] d_hitCnt; // counter for D-cache hit
	reg [`WORD_SIZE-1:0] i_accessCnt; // counter for I-memory access from datapath
	reg [`WORD_SIZE-1:0] d_accessCnt; // counter for D-memory access from datapath
	integer i;

	// cache hit registers
	reg i_cache_hit;
	reg d_cache_hit;

	// I-cache
	reg [tagSize-1:0] i_tagBank [blockNum-1:0];
	reg i_valid [blockNum-1:0];
	reg i_dirty [blockNum-1:0];
	reg [blockSize-1:0] i_dataBank [blockNum-1:0];

	// D-cache
	reg [tagSize-1:0] d_tagBank [blockNum-1:0];
	reg d_valid [blockNum-1:0];
	reg d_dirty [blockNum-1:0];
	reg [blockSize-1:0] d_dataBank [blockNum-1:0];

	// input address wires
	// i_address
	wire [tagSize-1:0] i_tag;
	wire [1:0] i_idx;
	wire [1:0] i_blockOffset;
	// d_address
	wire [tagSize-1:0] d_tag;
	wire [1:0] d_idx;
	wire [1:0] d_blockOffset;

	// interface between datapath and cache
	reg [`WORD_SIZE-1:0] i_outputDataC;
	reg [`WORD_SIZE-1:0] d_outputDataC;

	// interface with memory
	reg [`WORD_SIZE-1:0] i_addressM;
	reg [`FETCH_SIZE-1:0] i_outputDataM;
	reg i_readM;
	reg i_writeM;
	reg [`WORD_SIZE-1:0] d_addressM;
	reg [`FETCH_SIZE-1:0] d_outputDataM;
	reg d_readM;
	reg d_writeM;


	// FSM state
	// parameters for states
	parameter RESET = 4'h0;
	parameter READ_M0 = 4'h1;
	parameter READ_M1 = 4'h2;
	parameter READ_M2 = 4'h3;
	parameter READ_M3 = 4'h4;
	parameter FETCH_READY = 4'h5;
	parameter WRITE_M0 = 4'h6;
	parameter WRITE_M1 = 4'h7;
	parameter WRITE_M2 = 4'h8;
	parameter WRITE_M3 = 4'h9;
	parameter WRITE_READY = 4'ha;

	// state register
	reg [3:0] i_nextState;
	reg [3:0] i_state;
	reg [3:0] d_nextState;
	reg [3:0] d_state;

	assign {i_tag, i_idx, i_blockOffset} = i_addressC;
	assign {d_tag, d_idx, d_blockOffset} = d_addressC;
	assign {i_dataC, d_dataC, i_dataM, d_dataM} = {i_outputDataC, d_outputDataC, i_outputDataM, d_outputDataM};

	// update i_nextState, d_nextState
	always @(*) begin
		// update i_nextState 
		case(i_state)
			RESET : i_nextState <= (i_readC && (i_tagBank[i_idx] != i_tag || !i_valid[i_idx]))? READ_M0 : RESET;
			READ_M0 : i_nextState <= READ_M1;
			READ_M1 : i_nextState <= READ_M2;
			READ_M2 : i_nextState <= READ_M3;
			READ_M3 : i_nextState <= FETCH_READY;
			FETCH_READY : i_nextState <= RESET;
		endcase
		// update d_nextState
		case (d_state)
			RESET : begin
				if ((d_readC || d_writeC) && (d_tagBank[d_idx] != d_tag || !d_valid[d_idx]) && d_dirty[d_idx]) d_nextState <= WRITE_M0;
				else if (d_readC && (d_tagBank[d_idx] != d_tag || !d_valid[d_idx]) && !d_dirty[d_idx]) d_nextState <= READ_M0;
				else d_nextState <= RESET;
			end
			WRITE_M0 : d_nextState <= WRITE_M1;
			WRITE_M1 : d_nextState <= WRITE_M2;
			WRITE_M2 : d_nextState <= WRITE_M3;
			WRITE_M3 : d_nextState <= READ_M0;
			READ_M0 : d_nextState <= READ_M1;
			READ_M1 : d_nextState <= READ_M2;
			READ_M2 : d_nextState <= READ_M3;
			READ_M3 : d_nextState <= (d_readC)? FETCH_READY : WRITE_READY;
			FETCH_READY : d_nextState <= RESET;
			WRITE_READY : d_nextState <= RESET;
		endcase
	end

	// synchronous
	always @(posedge clk, negedge reset_n) begin
		if (!reset_n) begin
			// reset cache
			for (i = 0; i < blockNum; i = i + 1) begin
				// reset I-cache, D-cache
				{i_tagBank[i], i_valid[i], i_dirty[i], i_dataBank[i]} <= {tagSize'd0, 1'd0, 1'd0, blockSize'd0};
				{d_tagBank[i], d_valid[i], d_dirty[i], d_dataBank[i]} <= {tagSize'd0, 1'd0, 1'd0, blockSize'd0};

				// reset counter
				hitCnt <= `WORD_SIZE'd0;
				accessCnt <= `WORD_SIZE'd0;
				i_cache_hit <= 1'd0;
				d_cache_hit <= 1'd0;
				
				// reset memory interface
				{i_addressM, d_addressM} <= {`WORD_SIZE'd0, `WORD_SIZE'd0};
				{i_readM, i_writeM, d_readM, d_writeM} <= 4'd0;
				{i_outputDataC, d_outputDataC, i_outputDataM, d_outputDataM} <= {`WORD_SIZE'dz, `WORD_SIZE'dz, `FETCH_SIZE'dz, `FETCH_SIZE'dz};
			end
			{i_state, d_state} <= {RESET, RESET}; // reset state
			{i_accessCnt, d_accessCnt, i_hitCnt, d_hitCnt} <= {`WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0};
		end
		else begin
			{i_state, d_state} <= {i_nextState, d_nextState}; // update current state
			if (i_state == READ_M3) i_dataBank[i_idx] <= i_dataM; // update i_dataBank
			if (d_state == READ_M3) d_dataBank[d_idx] <= d_dataM; // allocate cache block from D-memory
			if (d_state == WRITE_READY) begin // write D-cache(SWD) & dirty = 1
				d_dataBank[d_idx][7 + 8*d_blockOffset : 8*d_blockOffset] <= d_dataC;
				dirty[d_idx] <= 1'd1;
			end
		end
	end

	// asynchronous
	always @(*) begin
		case (i_state)
			RESET : begin
				{i_readM, i_writeM} <= 2'b00;
				if (i_readC) begin
					// I-cache hit -> return i_dataC
					if (i_tagBank[i_idx] == i_tag && i_valid[i_idx]) begin
						i_cache_hit <= 1'd1;
						i_outputDataC <= i_dataBank[i_idx][7 + 8*i_blockOffset : 8*i_blockOffset];
					end
					else begin
						i_cache_hit <= 1'd0;
						i_outputDataC <= `WORD_SIZE'dz;
					end
				end
				else begin
					i_cache_hit <= 1'd0;
					i_outputDataC <= `WORD_SIZE'dz;
				end
			end
			READ_M0 : i_readM <= 1'd1;
			FETCH_READY : i_outputDataC <= i_dataBank[i_idx][7 + 8*i_blockOffset : 8*i_blockOffset];
		endcase
		case (d_state)
			RESET : begin
				{d_readM, d_writeM} <= 2'b00;
				if (d_readC) begin
					// D-cache hit -> return d_dataC
					if (d_tagBank[d_idx] == d_tag && d_valid[d_idx]) begin
						d_cache_hit <= 1'd1;
						d_outputDataC <= d_dataBank[d_idx][7 + 8*d_blockOffset : 8*d_blockOffset];
					end
					else begin
						d_cache_hit <= 1'd0;
						d_outputDataC <= `WORD_SIZE'dz;
					end
				end
				else if (d_writeC) begin
					d_outputDataC <= `WORD_SIZE'dz;
					// D-cache hit -> update d_dataBank, dirty = 1
					if (d_tagBank[d_idx] == d_tag && d_valid[d_idx]) begin
						d_dataBank[d_idx][7 + 8*d_blockOffset : 8*d_blockOffset] <= d_dataC;
						dirty[d_idx] <= 1'b1;
						d_cache_hit <= 1'b1;
					end
					// D-cache miss
					else begin
						d_cache_hit <= 1'b0;
					end
				end
				else begin
					d_outputDataC <= `WORD_SIZE'dz;
				end
			end
			READ_M0 : begin
				d_readM <= 1'd1;
				d_outputDataM <= `FETCH_SIZE'dz;
			end
			FETCH_READY : d_outputDataC <= d_dataBank[d_idx][7 + 8*d_blockOffset : 8*d_blockOffset]; // fetch data to datapath
			WRITE_M0 : d_writeM <= 1'd1;
			WRITE_M3 : d_outputDataM <= d_dataBank[d_idx]; // transfer data to memory
		endcase
	end



endmodule