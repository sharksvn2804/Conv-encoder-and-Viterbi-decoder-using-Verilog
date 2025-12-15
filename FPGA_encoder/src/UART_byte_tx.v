//----------------------------------------------------------------------------------------------------------
// UART TRANSMITTER (SINGLE BYTE)
//----------------------------------------------------------------------------------------------------------
module uart_byte_tx (
	clk,
	reset_n,
	data_byte,
	send_en,   
	baud_set, 
	uart_tx,  
	tx_done,   
	uart_state 
);

    // I/Os:
	input 		clk;    						 // Global clock input for the module, 50MHz
	input 		reset_n;    					 // Reset signal input, active low
  	input 		[7:0] data_byte;  				 // 8-bit data to be transmitted
	input 		send_en;    					 // Send enable signal
  	input 		[2:0] baud_set;   				 // Baud rate selection
	output reg 		uart_tx;    				 // UART transmission output signal
	output reg 		tx_done;    				 // Flag indicating completion of 1-byte data transmission
	output reg 		uart_state; 				 // Data transmission state
	
    // Internal registers, local parameters and wires:
	wire 			reset=~reset_n;
	localparam 		START_BIT = 1'b0;
	localparam 		STOP_BIT = 1'b1; 
	reg 	bps_clk;	     					 // Baud rate clock
    reg 	[15:0] div_cnt;      				 // Frequency division counter
    reg 	[15:0] bps_DR;       				 // Maximum count value for frequency division
  	reg 	[3:0] bps_cnt;      				 // Baud rate clock counter
  	reg 	[7:0] data_byte_reg;				 // Stored data after receiving data_byte
	
	// UART state control: Indicates whether transmission is ongoing:
    // 0: IDLE, 1: BUSY
    always @(posedge clk or posedge reset) begin
      	if (reset) begin 
            uart_state <= 1'b0;          		 
        end
      	else begin
          	if (uart_state && (bps_cnt == 4'd11)) begin
            	uart_state <= 1'b0;          	 
            end
        	else begin
              	if (send_en) begin
            		uart_state <= 1'b1;          
                end
                else begin
            		uart_state <= uart_state;   
                end
            end
        end
    end
	
	// Store the data to be transmitted:
    always @(posedge clk or posedge reset) begin
        if (reset) begin
          	data_byte_reg <= 8'b0;
        end
        else begin
            if (send_en) begin
              	data_byte_reg <= data_byte;
            end
            else begin
              	data_byte_reg <= data_byte_reg;
            end
        end
    end
	
	// Baud rate selection based on baud_set input:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
            bps_DR <= 16'd2811; 				// Default baud rate setting
        end
        else begin
            case (baud_set)
              	0: bps_DR <= 16'd2811; 			// Baud rate option 0: 2811
                1: bps_DR <= 16'd1405; 			// Baud rate option 1: 19200
                2: bps_DR <= 16'd702; 			// Baud rate option 2: 38400
                3: bps_DR <= 16'd467;  			// Baud rate option 3: 57600
                4: bps_DR <= 16'd233;  			// Baud rate option 4: 115200
                default: bps_DR <= 16'd2811; 	// Default value		
            endcase
        end	
  	end
	
	// Frequency division counter for baud rate clock:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
			div_cnt <= 16'd0; 					// Reset counter
        end
		else begin
          	if (uart_state == 1) begin
            	if (div_cnt == bps_DR) begin
					div_cnt <= 16'd0; 			// Reset when reaching the baud rate threshold
                end
				else begin
					div_cnt <= div_cnt + 1'b1; 	// Increment counter
				end
            end 
          	else begin
				div_cnt <= 16'd0; 				// Reset counter when not transmitting
            end
        end
    end
	
	// Generate baud rate clock signal:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
            bps_clk <= 1'b0; 					// Reset to low
        end
        else begin 
          	if (div_cnt == 16'd1) begin
           	 	bps_clk <= 1'b1; 				// Generate a pulse when the counter reaches 1
            end
            else begin
            	bps_clk <= 1'b0;
            end
        end
    end
	
	// Baud rate counter to track transmission progress:
  	always @(posedge clk or posedge reset) begin
        if (reset) begin	
            bps_cnt <= 4'd0; 					// Reset counter
        end 
        else begin
            if (bps_cnt == 4'd11) begin
                bps_cnt <= 4'd0; 				// Reset after full transmission cycle
            end
            else begin
              	if (bps_clk) begin
                    bps_cnt <= bps_cnt + 1'b1; 	// Increment on baud rate clock pulse
                end
                else begin
                  	bps_cnt <= bps_cnt;
                end
            end
        end
    end
		
	// Indicate when transmission is complete:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
            tx_done <= 1'b0; 					// Reset to low
        end
      	else begin
          	if (bps_cnt == 4'd11) begin
            	tx_done <= 1'b1; 				// Set to high when transmission completes
            end
        	else begin
				tx_done <= 1'b0;
            end
        end
    end
		
	// UART transmission logic:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
            uart_tx <= 1'b1; 					// Default line state is idle (high)
        end
        else begin
            case (bps_cnt)
                0: uart_tx <= 1'b1;	 			// Idle state
                1: uart_tx <= START_BIT; 		// Start bit
                2: uart_tx <= data_byte_reg[0]; // Transmit bit 0
                3: uart_tx <= data_byte_reg[1]; // Transmit bit 1
                4: uart_tx <= data_byte_reg[2]; // Transmit bit 2
                5: uart_tx <= data_byte_reg[3]; // Transmit bit 3
                6: uart_tx <= data_byte_reg[4]; // Transmit bit 4
                7: uart_tx <= data_byte_reg[5]; // Transmit bit 5
                8: uart_tx <= data_byte_reg[6]; // Transmit bit 6
                9: uart_tx <= data_byte_reg[7]; // Transmit bit 7
                10: uart_tx <= STOP_BIT; 		// Stop bit
                default: uart_tx <= 1'b1; 		// Default to idle
            endcase
        end
    end

endmodule