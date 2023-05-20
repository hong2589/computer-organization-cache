`include "opcodes.v"
`define WORD_SIZE 16    // data and address word size
`define FETCH_SIZE 64 // fetch size from memory (4 words = 64bits)
`define BLOCK_NUM 4
`define TAG_SIZE 12
module cache(
	input clk,
	input reset_n,

	// interface between datapath and I-cache
	input i_readC, // read signal for i-mem
	input i_writeC, // write signal for i-mem
	input [`WORD_SIZE-1:0] i_addressC, // memory address to fetch instruction
	output [`WORD_SIZE-1:0] i_dataC, // instruction data

	// interface between datapath and D-cache
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
	output d_cache_hit,
	output i_ready,
	output d_ready
);
	reg [`WORD_SIZE-1:0] i_hitCnt; // counter for I-cache hit
	reg [`WORD_SIZE-1:0] d_hitCnt; // counter for D-cache hit
	reg [`WORD_SIZE-1:0] i_accessCnt; // counter for I-memory access from datapath
	reg [`WORD_SIZE-1:0] d_accessCnt; // counter for D-memory access from datapath
	reg [`WORD_SIZE-1:0] next_i_hitCnt; 
	reg [`WORD_SIZE-1:0] next_d_hitCnt; 
	reg [`WORD_SIZE-1:0] next_i_accessCnt; 
	reg [`WORD_SIZE-1:0] next_d_accessCnt; 
	integer i;

	// cache hit registers
	reg i_cache_hit;
	reg d_cache_hit;

	// I-cache
	reg [`TAG_SIZE-1:0] i_tagBank [`BLOCK_NUM-1:0];
	reg i_valid [`BLOCK_NUM-1:0];
	reg i_dirty [`BLOCK_NUM-1:0];
	reg [`FETCH_SIZE-1:0] i_dataBank [`BLOCK_NUM-1:0];

	// D-cache
	reg [`TAG_SIZE-1:0] d_tagBank [`BLOCK_NUM-1:0];
	reg d_valid [`BLOCK_NUM-1:0];
	reg d_dirty [`BLOCK_NUM-1:0];
	reg [`FETCH_SIZE-1:0] d_dataBank [`BLOCK_NUM-1:0];

	// input address wires
	// i_address
	wire [`TAG_SIZE-1:0] i_tag;
	wire [1:0] i_idx;
	wire [1:0] i_blockOffset;
	// d_address
	wire [`TAG_SIZE-1:0] d_tag;
	wire [1:0] d_idx;
	wire [1:0] d_blockOffset;

	// interface between datapath and cache
	reg [`WORD_SIZE-1:0] i_outputDataC;
	reg [`WORD_SIZE-1:0] d_outputDataC;
	reg i_ready; reg d_ready;

	// interface with memory
	reg [`FETCH_SIZE-1:0] i_outputDataM;
	reg i_readM;
	reg i_writeM;
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
	assign {i_addressM, d_addressM} = {i_addressC, d_addressC};

	// update i_nextState, d_nextState
	always @(*) begin
		if (!reset_n) begin
			i_nextState <= RESET;
			d_nextState <= RESET;
		end
		else begin
			// update i_nextState 
			case(i_state)
				RESET : begin
					if (d_state == RESET && (d_readC || d_writeC) && (d_tagBank[d_idx] != d_tag || !d_valid[d_idx]) && d_dirty[d_idx]) i_nextState <= RESET; // D-cache hit -> maintatin RESET
					else if (i_readC && (i_tagBank[i_idx] != i_tag || !i_valid[i_idx])) i_nextState <= READ_M0;
					else i_nextState <= RESET;
				end
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
	end

	// synchronous
	always @(posedge clk, negedge reset_n) begin
		if (!reset_n) begin
			// reset cache
			for (i = 0; i < `BLOCK_NUM; i = i + 1) begin
				// reset I-cache, D-cache
				{i_tagBank[i], i_valid[i], i_dirty[i], i_dataBank[i]} <= {12'd0, 1'd0, 1'd0, 64'd0};
				{d_tagBank[i], d_valid[i], d_dirty[i], d_dataBank[i]} <= {12'd0, 1'd0, 1'd0, 64'd0};
			end
			{i_state, d_state} <= {RESET, RESET}; // reset state
			{i_accessCnt, d_accessCnt, i_hitCnt, d_hitCnt} <= {`WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0}; // reset counter
		end
		else begin
			{i_state, d_state} <= {i_nextState, d_nextState}; // update current state
			{i_accessCnt, i_hitCnt, d_accessCnt, d_hitCnt} <= {next_i_accessCnt, next_i_hitCnt, next_d_accessCnt, next_d_hitCnt};
			if (i_state == READ_M3) begin
				// update i_dataBank, i_tagBank, i_valid from I-memory
				i_dataBank[i_idx] <= i_dataM; 
				i_tagBank[i_idx] <= i_addressC[15:4];
				i_valid[i_idx] <= 1'd1;
			end
			if (d_state == READ_M3) begin
				// update d_dataBank, d_tagBank, d_valid from D-memory
				d_dataBank[d_idx] <= d_dataM; 
				d_tagBank[d_idx] <= d_addressC[15:4];
				d_valid[d_idx] <= 1'd1;
			end
			if (d_state == WRITE_READY) begin // write D-cache(SWD) & dirty = 1
				case (d_blockOffset)
					2'd0 : d_dataBank[d_idx][7:0] <= d_dataC;
					2'd1 : d_dataBank[d_idx][15:8] <= d_dataC;
					2'd2 : d_dataBank[d_idx][23:16] <= d_dataC;
					2'd3 : d_dataBank[d_idx][31:24] <= d_dataC;
				endcase
				d_dirty[d_idx] <= 1'd1;
			end
		end
	end

	// asynchronous
	always @(*) begin
		if (!reset_n) begin
			{i_cache_hit, d_cache_hit} <= 2'b11;
			{i_readM, i_writeM, d_readM, d_writeM} <= 4'd0;
			{i_ready, d_ready} <= 2'b00;
			{next_i_accessCnt, next_d_accessCnt, next_i_hitCnt, next_d_hitCnt} <= {`WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0, `WORD_SIZE'd0}; // reset next_counter
			{i_outputDataC, d_outputDataC, i_outputDataM, d_outputDataM} <= {`WORD_SIZE'dz, `WORD_SIZE'dz, `FETCH_SIZE'dz, `FETCH_SIZE'dz};
		end
		else begin
			case (i_state)
				RESET : begin
					{i_readM, i_writeM} <= 2'b00;
					i_ready <= 1'd0;
					if (i_readC) begin
						next_i_accessCnt <= i_accessCnt + `WORD_SIZE'd1;
						// I-cache hit -> return i_dataC
						if (i_tagBank[i_idx] == i_tag && i_valid[i_idx]) begin
							i_cache_hit <= 1'd1; next_i_hitCnt <= i_hitCnt + `WORD_SIZE'd1;
							case (i_blockOffset)
								2'd0 : i_outputDataC <= i_dataBank[i_idx][15:0];
								2'd1 : i_outputDataC <= i_dataBank[i_idx][31:16];
								2'd2 : i_outputDataC <= i_dataBank[i_idx][47:32];
								2'd3 : i_outputDataC <= i_dataBank[i_idx][63:48];  
							endcase
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
				FETCH_READY : begin
					i_ready <= 1'd1; // i_cache_hit <= 1'd1;
					case (i_blockOffset)
						2'd0 : i_outputDataC <= i_dataBank[i_idx][15:0];
						2'd1 : i_outputDataC <= i_dataBank[i_idx][31:16];
						2'd2 : i_outputDataC <= i_dataBank[i_idx][47:32];
						2'd3 : i_outputDataC <= i_dataBank[i_idx][63:48];  
					endcase
				end
			endcase
			case (d_state)
				RESET : begin
					{d_readM, d_writeM} <= 2'b00;
					d_ready <= 1'd0;
					if (d_readC) begin
						next_d_accessCnt <= d_accessCnt + `WORD_SIZE'd1;
						// D-cache hit -> return d_dataC
						if (d_tagBank[d_idx] == d_tag && d_valid[d_idx]) begin
							d_cache_hit <= 1'd1; next_d_hitCnt <= d_hitCnt + `WORD_SIZE'd1;
							case (d_blockOffset)
								2'd0 : d_outputDataC <= d_dataBank[d_idx][15:0];
								2'd1 : d_outputDataC <= d_dataBank[d_idx][31:16];
								2'd2 : d_outputDataC <= d_dataBank[d_idx][47:32];
								2'd3 : d_outputDataC <= d_dataBank[d_idx][63:48];
							endcase
						end
						else begin
							d_cache_hit <= 1'd0;
							d_outputDataC <= `WORD_SIZE'dz;
						end
					end
					else if (d_writeC) begin
						next_d_accessCnt <= d_accessCnt + `WORD_SIZE'd1;
						d_outputDataC <= `WORD_SIZE'dz;
						// D-cache hit -> update d_dataBank, dirty = 1
						if (d_tagBank[d_idx] == d_tag && d_valid[d_idx]) begin
							case (d_blockOffset)
								2'd0 : d_dataBank[d_idx][7:0] <= d_dataC;
								2'd1 : d_dataBank[d_idx][15:8] <= d_dataC;
								2'd2 : d_dataBank[d_idx][23:16] <= d_dataC;
								2'd3 : d_dataBank[d_idx][31:24] <= d_dataC;
							endcase
							d_dirty[d_idx] <= 1'b1;
							d_cache_hit <= 1'b1; next_d_hitCnt <= d_hitCnt + `WORD_SIZE'd1;
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
				FETCH_READY : begin
					d_ready <= 1'd1; d_cache_hit <= 1'd1;
					// fetch data to datapath
					case (d_blockOffset)
						2'd0 : d_outputDataC <= d_dataBank[d_idx][15:0];
						2'd1 : d_outputDataC <= d_dataBank[d_idx][31:16];
						2'd2 : d_outputDataC <= d_dataBank[d_idx][47:32];
						2'd3 : d_outputDataC <= d_dataBank[d_idx][63:48];
					endcase
				end 
				WRITE_M0 : d_writeM <= 1'd1;
				WRITE_M3 : d_outputDataM <= d_dataBank[d_idx]; // transfer data to memory
				WRITE_READY : begin
					d_ready <= 1'd1; d_cache_hit <= 1'd1;
				end
			endcase
		end
	end



endmodule