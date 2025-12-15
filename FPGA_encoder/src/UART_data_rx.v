//----------------------------------------------------------------------------------------------------------
// UART RECEIVER (MULTIPLE BYTES, INCLUDING ENCODE FLAG BEFORE ENCODING)
//----------------------------------------------------------------------------------------------------------
module uart_data_rx (
	Clk,
	Rst_n,
  	uart_rx,
	data,
	Rx_Done,
	timeout_flag,
	Baud_Set,
	encode_flag
);
	// I/Os:
	input 		Clk;
	input 		Rst_n;
	input 		uart_rx;
    input 		[2:0] Baud_Set;
	output reg 	[DATA_WIDTH-1:0] data;
	output reg 	Rx_Done;
	output reg 	timeout_flag;
  	output reg 	encode_flag;
  	
    // Internal registers, wires, and parameters:
    parameter 	DATA_WIDTH = 32;
	parameter 	MSB_FIRST = 1;
    localparam 	S0 = 0;	                    // Wait for a single byte to receive a complete signal
	localparam 	S1 = 1;	                    // Determine whether the receipt timed out
	localparam 	S2 = 2;	                    // Check if all data has been received
  	reg 		[DATA_WIDTH-1:0] data_r;
	wire 		[7:0] data_byte;
	wire 		byte_rx_done;
	wire 		[19:0] TIMEOUT;
  	reg 		[8:0] cnt;
  	reg 		[1:0] state;
	reg 		[31:0] timeout_cnt;
	reg 		to_state;
	
    // Module instance (single byte receiver side):
	uart_byte_rx uart_byte_rx (
        .clk(Clk),
        .reset_n(Rst_n),	
      	.baud_set(Baud_Set),
		.uart_rx(uart_rx),
		.data_byte(data_byte),
		.rx_done(byte_rx_done)
	);
	
	// Automatically set a timeout based on the baud rate:
    assign TIMEOUT = (Baud_Set == 3'd0) ? 20'd182291 :
                     (Baud_Set == 3'd1) ? 20'd91145  :
                     (Baud_Set == 3'd2) ? 20'd45572  :
                     (Baud_Set == 3'd3) ? 20'd30381  :
                                          20'd15190;

	// Timeout flag management:
	always @(posedge Clk or negedge Rst_n) begin
		if (!Rst_n) begin
			timeout_flag <= 1'd0;
		end 
		else if (timeout_cnt >= TIMEOUT) begin
			timeout_flag <= 1'd1;
		end
		else if (state == S0) begin
			timeout_flag <= 1'd0;
		end
	end
	
	// Timeout state machine
	always @(posedge Clk or negedge Rst_n) begin
		if (!Rst_n)
			to_state <= 0;
		else if (!uart_rx)
			to_state <= 1;
		else if (byte_rx_done)
			to_state <= 0; 
	end
	
	// Timeout counter
	always @(posedge Clk or negedge Rst_n) begin
		if (!Rst_n)
			timeout_cnt <= 32'd0;
		else if (to_state) begin
			if (byte_rx_done)
				timeout_cnt <= 32'd0;
			else if (timeout_cnt >= TIMEOUT)
				timeout_cnt <= TIMEOUT;
			else
				timeout_cnt <= timeout_cnt + 1'd1;
		end
	end
    
	// Main FSM for data reception (important):
	always @(posedge Clk or negedge Rst_n) begin
		if (!Rst_n) begin
			data_r <= 0;
			state <= S0;
			cnt <= 0;
			data <= 0;
			encode_flag <= 1'b0;
		end
		else begin
			case (state)
                // Idle (received the first byte):
				S0: 
					begin
						Rx_Done <= 1'b0;
						data_r <= 0;
						encode_flag <= 1'b0;
                        // 1 byte => assign
						if (DATA_WIDTH == 8) begin
							data <= data_byte;
							Rx_Done <= byte_rx_done;
						end
						else if (byte_rx_done) begin
                            // Turning to S1 and wait for the next byte:
							state <= S1;
							cnt <= cnt + 9'd8;
                            // Merging bytes:
							if (MSB_FIRST == 1) begin
								data_r <= {data_r[DATA_WIDTH - 1 - 8 : 0], data_byte};
                            end
							else begin
								data_r <= {data_byte, data_r[DATA_WIDTH - 1 : 8]};
                            end
						end
					end
				// Received next byte:	
				S1:
                    // Timeout => no encoding and return to IDLE state:
					if (timeout_flag) begin
						state <= S0;
						Rx_Done <= 1'b1;	
						encode_flag <= 1'b0;
					end
					else if (byte_rx_done) begin
                        // Turning to S2 and wait for the next bytes:
						state <= S2;
						cnt <= cnt + 9'd8;
                        // Merging bytes:
						if (MSB_FIRST == 1)
							data_r <= {data_r[DATA_WIDTH - 1 - 8 : 0], data_byte};
						else
							data_r <= {data_byte, data_r[DATA_WIDTH - 1 : 8]};
					end
				// Received additional bytes:	
				S2:
                    // Full bit width => Allowed to encode and then reset:
					if (cnt >= DATA_WIDTH) begin
						encode_flag <= 1'b1;
						state <= S0;
						data <= data_r; 
						cnt <= 0;
						Rx_Done <= 1'b1;
					end
					else begin
                        // Continue receiving bytes:
						state <= S1;
						Rx_Done <= 1'b0;
					end
                // Default: return to IDLE state:
				default: state <= S0;
			endcase	
		end
	end

endmodule