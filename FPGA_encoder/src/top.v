//----------------------------------------------------------------------------------------------------------
// MAIN MODULE:
//----------------------------------------------------------------------------------------------------------
module top (
    input           clk,
    input           rst_n,
    input           uart_rx,
    output          [1:0] led,
    output          uart_tx
);

    // Internal regs, wires, and parameters:
    wire [15:0] rx_data;
    wire        rx_done;
    wire        timeout_flag;
    wire        encode_flag;
    wire [31:0] symbol_data;
    wire        encode_done;
    reg  [31:0] tx_shift_reg;
    reg         tx_send;
    wire        tx_done;
    wire        uart_state;
    wire  [2:0] set_baud_rate;
    reg         encode_done_d;
    wire        encode_done_pulse;
    reg [1:0]   tx_state;   
    localparam  TX_IDLE     = 2'd0;  // Wait for finishing encoding
    localparam  TX_LOADED   = 2'd1;  // Wait before transmitting
    localparam  TX_SEND     = 2'd2;  // Allowed to send
    
    // Assign the baud rate:
    assign      set_baud_rate = 3'd4;

    // Module instance (UART data receiver - received 16 bit data)
    uart_data_rx #(
        .DATA_WIDTH(16),
        .MSB_FIRST(1)
    ) u_rx (
        .Clk(clk),
        .Rst_n(rst_n),
        .uart_rx(uart_rx),
        .data(rx_data),
        .Rx_Done(rx_done),
        .timeout_flag(timeout_flag),
        .Baud_Set(set_baud_rate),
        .encode_flag(encode_flag)
    );

    // Module instance for convolutional encoder:
    convolutional_encoder #(
        .DATA_WIDTH(16)
    ) u_conv (
        .Clk(clk),
        .Rst_n(rst_n),
        .encode_flag(encode_flag),
        .data_in(rx_data),
        .symbol_data(symbol_data),
        .encode_done(encode_done)
    );

    // Creating a pulse for encoding:
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            encode_done_d <= 1'b0;
        else
            encode_done_d <= encode_done;
    end
    assign encode_done_pulse = encode_done & ~encode_done_d;
    
    // FSM before sending or transmitting:
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift_reg <= 32'd0;
            tx_send      <= 1'b0;
            tx_state     <= TX_IDLE;
        end 
        else begin
            case (tx_state)
                // State 0: IDLE
                TX_IDLE: begin
                    tx_send <= 1'b0;
                    if (encode_done_pulse) begin
                        tx_shift_reg <= symbol_data;  
                        tx_state     <= TX_LOADED;    
                    end
                end
                // State 1: LOADED - Wait before transmitting
                TX_LOADED: begin
                    tx_send <= 1'b0;
                    if (!uart_state) begin            
                        tx_state <= TX_SEND;          
                    end
                end
                // State 2: Allowed to send
                TX_SEND: begin
                    tx_send  <= 1'b1;               
                    tx_state <= TX_IDLE;              
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
    
    // Module instance (UART data receiver - send 32-bit symbol after interleaving)
    uart_data_tx #(
        .DATA_WIDTH(32),
        .MSB_FIRST(1)
    ) u_tx (
        .Clk(clk),
        .Rst_n(rst_n),
        .data(tx_shift_reg),
        .send_en(tx_send),
        .Baud_Set(set_baud_rate),
        .uart_tx(uart_tx),
        .Tx_Done(tx_done),
        .uart_state(uart_state)
    );

    // Debug LEDs: [1] = uart_state (busy), [0] = rx_done
    assign led = { uart_state, rx_done };

endmodule