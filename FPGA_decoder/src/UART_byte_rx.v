//----------------------------------------------------------------------------------------------------------
// UART RECEIVER FOR A SINGLE BYTE:
//----------------------------------------------------------------------------------------------------------
module uart_byte_rx (
	clk,
	reset_n,
	baud_set,
	uart_rx,
	data_byte,
	rx_done
);
  	// I/Os:
	input 		clk;    
	input 		reset_n;   
 	input 		[2:0] baud_set;  
	input 		uart_rx; 
  	output 		[7:0] data_byte; 
	output 		rx_done;   
  
  	// Registers and wires:
  	wire 		reset = ~reset_n;
  	reg 		[7:0] data_byte;
	reg 		rx_done;
	reg 		uart_rx_sync1;   
	reg 		uart_rx_sync2;   
	reg 		uart_rx_reg1;   
	reg 		uart_rx_reg2;   
  	reg 		[15:0] bps_DR;    
  	reg 		[15:0] div_cnt; 
	reg 		bps_clk;   
	reg 		[7:0] bps_cnt; 
	reg 		uart_state; 
	wire 		uart_rx_nedge;  
    reg 		[2:0] START_BIT;
    reg 		[2:0] STOP_BIT;
    reg 		[2:0] data_byte_pre [7:0];
  	reg 		[3:0] idx;
    reg 		ff1;
 	reg 		ff2;
    reg 		upper_xor_r;
    reg 		lower_xor_r;
  
	// Synchronize bit:
  	always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_rx_sync1 <= 1'b0;
            uart_rx_sync2 <= 1'b0;	
        end
        else begin
            uart_rx_sync1 <= uart_rx;
            uart_rx_sync2 <= uart_rx_sync1;	
        end
    end
	
	// Data register:
  	always@ (posedge clk or posedge reset) begin
      	if (reset) begin
            uart_rx_reg1 <= 1'b0;
            uart_rx_reg2 <= 1'b0;	
        end
        else begin
            uart_rx_reg1 <= uart_rx_sync2;
            uart_rx_reg2 <= uart_rx_reg1;	
        end
    end
	
    // Falling edge detection:
	assign uart_rx_nedge = !uart_rx_reg1 & uart_rx_reg2;
		
    // Choosing baud rate:
  	always@(posedge clk or posedge reset) begin
      	if (reset) begin
            bps_DR <= 16'd174;
      	end	
        else begin
            case (baud_set)
                0: bps_DR <= 16'd174;		//Baud rate option 0(27MHZ, so baurate 9600:27*10^6/(9600*16) -1=2)
                1: bps_DR <= 16'd86;		//19200
                2: bps_DR <= 16'd42;		//38400
                3: bps_DR <= 16'd28;		//57600
                4: bps_DR <= 16'd13;		//115200
                default: bps_DR <= 16'd174;			
            endcase
        end
    end
  
  	// Counter from baud rate:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
            div_cnt <= 16'd0;
        end
        else if (uart_state) begin
            if (div_cnt == bps_DR)
                div_cnt <= 16'd0;
            else
                div_cnt <= div_cnt + 1'b1;
        end
        else begin
            div_cnt <= 16'd0;
        end
    end
		
	// Generate bps_clk:
  	always @(posedge clk or posedge reset) begin
        if (reset) begin
            bps_clk <= 1'b0;
        end
        else if (div_cnt == 16'd1) begin
            bps_clk <= 1'b1;
        end
        else begin
            bps_clk <= 1'b0;
        end
    end

  	// Baud rate counter:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
            bps_cnt <= 8'd0;
        end
      	else if (bps_cnt == 8'd159 || (bps_cnt == 8'd12 && (START_BIT > 2))) begin
            bps_cnt <= 8'd0;
      	end
      	else if (bps_clk) begin
            bps_cnt <= bps_cnt + 1'b1;
        end
        else begin
            bps_cnt <= bps_cnt;
        end
    end

    // Stating that the receiving byte activity is done:
  	always @(posedge clk or posedge reset) begin
        if (reset) begin
            rx_done <= 1'b0;
        end
      	else if (bps_cnt == 8'd159) begin
            rx_done <= 1'b1;
      	end
        else begin
            rx_done <= 1'b0;		
        end
  	end
		
    // Sampling:
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            START_BIT <= 3'd0;
            data_byte_pre[0] <= 3'd0;
            data_byte_pre[1] <= 3'd0;
            data_byte_pre[2] <= 3'd0;
            data_byte_pre[3] <= 3'd0;
            data_byte_pre[4] <= 3'd0;
            data_byte_pre[5] <= 3'd0;
            data_byte_pre[6] <= 3'd0;
            data_byte_pre[7] <= 3'd0;
            STOP_BIT <= 3'd0;
        end
        else if (bps_clk) begin
            case (bps_cnt)
                0: begin
                    START_BIT <= 3'd0;
                    data_byte_pre[0] <= 3'd0;
                    data_byte_pre[1] <= 3'd0;
                    data_byte_pre[2] <= 3'd0;
                    data_byte_pre[3] <= 3'd0;
                    data_byte_pre[4] <= 3'd0;
                    data_byte_pre[5] <= 3'd0;
                    data_byte_pre[6] <= 3'd0;
                    data_byte_pre[7] <= 3'd0;
                    STOP_BIT <= 3'd0;			
                end
                6, 7, 8, 9, 10, 11: START_BIT <= START_BIT + uart_rx_sync2;
                22, 23, 24, 25, 26, 27: data_byte_pre[0] <= data_byte_pre[0] + uart_rx_sync2;
                38, 39, 40, 41, 42, 43: data_byte_pre[1] <= data_byte_pre[1] + uart_rx_sync2;
                54, 55, 56, 57, 58, 59: data_byte_pre[2] <= data_byte_pre[2] + uart_rx_sync2;
                70, 71, 72, 73, 74, 75: data_byte_pre[3] <= data_byte_pre[3] + uart_rx_sync2;
                86, 87, 88, 89, 90, 91: data_byte_pre[4] <= data_byte_pre[4] + uart_rx_sync2;
                102, 103, 104, 105, 106, 107: data_byte_pre[5] <= data_byte_pre[5] + uart_rx_sync2;
                118, 119, 120, 121, 122, 123: data_byte_pre[6] <= data_byte_pre[6] + uart_rx_sync2;
                134, 135, 136, 137, 138, 139: data_byte_pre[7] <= data_byte_pre[7] + uart_rx_sync2;
                150, 151, 152, 153, 154, 155: STOP_BIT <= STOP_BIT + uart_rx_sync2;
                default: begin
                    START_BIT <= START_BIT;
                    data_byte_pre[0] <= data_byte_pre[0];
                    data_byte_pre[1] <= data_byte_pre[1];
                    data_byte_pre[2] <= data_byte_pre[2];
                    data_byte_pre[3] <= data_byte_pre[3];
                    data_byte_pre[4] <= data_byte_pre[4];
                    data_byte_pre[5] <= data_byte_pre[5];
                    data_byte_pre[6] <= data_byte_pre[6];
                    data_byte_pre[7] <= data_byte_pre[7];
                    STOP_BIT <= STOP_BIT;
                end
            endcase
        end
    end
     
    // Assign the data byte:
  	always @(posedge clk or posedge reset) begin
      	if (reset) begin
            data_byte <= 8'd0;
        end
        else if (bps_cnt == 8'd159) begin
            data_byte[0] <= data_byte_pre[0][2];
            data_byte[1] <= data_byte_pre[1][2];
            data_byte[2] <= data_byte_pre[2][2];
            data_byte[3] <= data_byte_pre[3][2];
            data_byte[4] <= data_byte_pre[4][2];
            data_byte[5] <= data_byte_pre[5][2];
            data_byte[6] <= data_byte_pre[6][2];
            data_byte[7] <= data_byte_pre[7][2];
        end
    end
	
    // UART state:
  	always @(posedge clk or posedge reset) begin
        if (reset) begin
            uart_state <= 1'b0;
        end
        else if (uart_rx_nedge) begin
            uart_state <= 1'b1;
        end
        else if (rx_done || (bps_cnt == 8'd12 && (START_BIT > 2)) || (bps_cnt == 8'd155 && (STOP_BIT < 3))) begin
            uart_state <= 1'b0;
        end
        else begin
            uart_state <= uart_state;	
        end
    end
  	
endmodule