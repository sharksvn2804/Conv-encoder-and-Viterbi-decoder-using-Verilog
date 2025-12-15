//----------------------------------------------------------------------------------------------------------
// UART TRANSMITTER (MULTIPLE BYTES)
//----------------------------------------------------------------------------------------------------------
module uart_data_tx (
	Clk,
	Rst_n,
	data,
	send_en,   
	Baud_Set,  
	uart_tx,  
	Tx_Done,   
	uart_state
);

    // I/Os:
	input       Clk;
	input       Rst_n;
	input       [DATA_WIDTH-1:0]data;
	input       send_en;
	input       [2:0]Baud_Set;
	output      uart_tx;
	output      reg Tx_Done;
	output      uart_state;
	
    // Internal registers, wires, and parameters:
	reg         [DATA_WIDTH-1:0]data_r;
	reg         [7:0] data_byte;
	reg         byte_send_en;
    reg         [8:0] cnt;
	reg         [1:0] state;
	wire        byte_tx_done;
    parameter   DATA_WIDTH = 8;
	parameter   MSB_FIRST = 1;
	localparam  S0 = 0;	                //Wait for the request to be sent
	localparam  S1 = 1;	                //Initiates a single-byte data send
	localparam  S2 = 2;	                //Wait for the single-byte data to be sent
	localparam  S3 = 3;	                //Check that all data has been sent
	
    // Module instance (single byte transmitter side):
	uart_byte_tx uart_byte_tx (
		.clk(Clk),
		.reset_n(Rst_n),
		.data_byte(data_byte),
		.send_en(byte_send_en),   
		.baud_set(Baud_Set),  
		.uart_tx(uart_tx),  
		.tx_done(byte_tx_done),   
		.uart_state(uart_state) 
	);
	
    // FSM for transmit:
	always @(posedge Clk or negedge Rst_n) begin
		if (!Rst_n) begin
			data_byte <= 0;
			byte_send_en <= 0;
			state <= S0;
			cnt <= 0;
		end
		else begin
			case (state)
                // IDLE state:
				S0: begin
                    data_byte <= 0;
                    cnt <= 0;
                    Tx_Done <= 0;
                    // Enable sending => Turning to S1 (sending a byte):
                    if (send_en) begin
                        state <= S1;
                        data_r <= data;
                    end
                    else begin
                        state <= S0;
                        data_r <= data_r;
                    end
                end
				// Sending 1 byte:
				S1: begin
                    byte_send_en <= 1;
                    // Sending the first MSB (most significant byte):
                    if (MSB_FIRST == 1) begin
                        data_byte <= data_r[DATA_WIDTH-1:DATA_WIDTH - 8];
                        data_r <= data_r << 8;
                    end
                    else begin
                        data_byte <= data_r[7:0];
                        data_r <= data_r >> 8;
                    end
                    state <= S2;
                end
                // Inspect:
				S2: begin
                    byte_send_en <= 0;
                    if (byte_tx_done) begin
                        state <= S3;
                        cnt <= cnt + 9'd8;
                    end
                    else begin
                        state <= S2;
                    end
                end
				// Full bit width => returning to IDLE and reset. If not, returning back to S1 and send another byte:
				S3: begin
					if (cnt >= DATA_WIDTH) begin
						state <= S0;
						cnt <= 0;
						Tx_Done <= 1;
					end
					else begin
						state <= S1;
						Tx_Done <= 0;
					end
                end
                // Default: IDLE state:
				default: state <= S0;
			endcase	
		end
	end

endmodule