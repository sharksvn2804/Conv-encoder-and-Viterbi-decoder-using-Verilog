// ----------------------------------------------------------------------------------------------------------
// CONVOLUTIONAL ENCODER (STAND-ALONE, FPGA FRIENDLY)
// ----------------------------------------------------------------------------------------------------------
module convolutional_encoder(
    Clk,
    Rst_n,
    encode_flag,
    data_in,
    symbol_data,
    encode_done
);

    // --- I/Os ---
    input Clk;
    input Rst_n;
    input encode_flag;
    input [DATA_WIDTH-1:0] data_in;
    output reg [2*DATA_WIDTH-1:0] symbol_data;
    output reg encode_done;

    // --- Internal regs, wires and parameter for encoder ---
    reg [DATA_WIDTH-1:0] idx;
    reg ff1;
	reg ff2;
    reg upper_xor_r;
	reg lower_xor_r;
    reg upper_xor_next;
	reg lower_xor_next;
    reg encoding;
    parameter DATA_WIDTH = 16;

	// --- Internal regs and wires for interleaving ---
    integer r;
	integer c;
    reg [2*DATA_WIDTH-1:0] interleave_temp;     // combinational buffer

    // combinational XOR:
    always @(*) begin
        if (encoding && idx < DATA_WIDTH) begin
            upper_xor_next = data_in[DATA_WIDTH-1-idx] ^ ff1 ^ ff2;
            lower_xor_next = data_in[DATA_WIDTH-1-idx] ^ ff2;
        end 
        else begin
            upper_xor_next = 1'b0;
            lower_xor_next = 1'b0;
        end
    end

    // combinational interleave (blocking assignment, synth-friendly)
    always @(*) begin
        interleave_temp = 0;
        for (r=0; r<8; r=r+1) begin
            for (c=0; c<4; c=c+1) begin
                interleave_temp[2*DATA_WIDTH-1-(r+c*8)] = symbol_data[2*DATA_WIDTH-1-(4*r+c)];
            end
        end
    end

    // sequential logic
    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            idx <= 0;
            ff1 <= 0;
            ff2 <= 0;
            upper_xor_r <= 0;
            lower_xor_r <= 0;
            encoding <= 0;
            symbol_data <= 0;
            encode_done <= 0;
        end 
        else begin
            if (encode_flag && !encoding) begin
                encoding <= 1;
                idx <= 0;
                ff1 <= 0;
				ff2 <= 0;
                upper_xor_r <= 0; 
				lower_xor_r <= 0;
                symbol_data <= 0;
                encode_done <= 0;
            end
            if (encoding) begin
                if (idx <= DATA_WIDTH) begin
                    symbol_data <= {symbol_data[2*DATA_WIDTH-3:0], {upper_xor_r, lower_xor_r}};
                    upper_xor_r <= upper_xor_next;
                    lower_xor_r <= lower_xor_next;
                    ff2 <= ff1;
                    if (idx < DATA_WIDTH) begin 
						ff1 <= data_in[DATA_WIDTH-1-idx]; 
					end 
					else begin
						ff1 <= 1'b0;
					end
                    idx <= idx + 16'd1;
                    encode_done <= 0;
                end 
                else begin
                    // gán kết quả interleave (non-blocking)
                    symbol_data <= interleave_temp;
                    // Đánh trống phất cờ
                    encode_done <= 1;
                    encoding <= 0;
                    idx <= 0;
                    ff1 <= 0; 
					ff2 <= 0;
                    upper_xor_r <= 0; 
					lower_xor_r <= 0;
                end
            end 
            else if (!encode_flag) begin
                encode_done <= 0;
            end
        end
    end

endmodule