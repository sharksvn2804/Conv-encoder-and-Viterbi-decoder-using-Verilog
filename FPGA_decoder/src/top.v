//----------------------------------------------------------------------------------------------------------
// MAIN MODULE – DECODE VERSION (FIXED v6 - Decoder Reset Added)
//----------------------------------------------------------------------------------------------------------
module top (
    input           clk,
    input           rst_n,
    input           uart_rx,
    output          [1:0] led,
    output          uart_tx
);
    // ==============================
    // Internal signals
    // ==============================
    wire [31:0] rx_data;            // 32-bit symbol from MCU
    wire        rx_done;
    wire        timeout_flag;
    wire        decode_flag;
    wire [15:0] decoded_data;
    wire        decode_done;
    reg  [15:0] tx_shift_reg;
    reg         tx_send;
    wire        tx_done;
    wire        uart_state;
    wire [2:0]  set_baud_rate = 3'd4;  // 115200 bps
    
    // ==============================
    // Decoder control signals - KEY FIX
    // ==============================
    reg         decoder_reset_n;     // Active-low reset for decoder
    reg         decoder_start;       // Start pulse for decoder
    
    // ==============================
    // TX FSM
    // ==============================
    reg         rx_done_d;
    reg         decode_done_d;
    wire        rx_done_pulse;
    wire        decode_done_pulse;
    
    reg  [2:0]  tx_state;
    localparam  TX_IDLE     = 3'd0;
    localparam  TX_RESET_DEC = 3'd1;  // NEW: Reset decoder
    localparam  TX_START_DEC = 3'd2;  // NEW: Start decoder
    localparam  TX_WAIT_DEC = 3'd3;
    localparam  TX_SEND     = 3'd4;
    
    // ==============================
    // UART RX (receive 32-bit encoded data)
    // ==============================
    uart_data_rx #(
        .DATA_WIDTH(32),
        .MSB_FIRST(1)
    ) u_rx (
        .Clk(clk),
        .Rst_n(rst_n),
        .uart_rx(uart_rx),
        .data(rx_data),
        .Rx_Done(rx_done),
        .timeout_flag(timeout_flag),
        .Baud_Set(set_baud_rate),
        .decode_flag(decode_flag)
    );
    
    // ==============================
    // CONVOLUTIONAL DECODER - with controlled reset
    // ==============================
    convolutional_decoder u_dec (
        .clk(clk),
        .reset_n(decoder_reset_n),      // Controlled by FSM
        .decode_flag(decoder_start),    // Use FSM-controlled start
        .symbol_data(rx_data),
        .decoded_output(decoded_data),
        .decode_done(decode_done)
    );
    
    // ==============================
    // RX-done pulse detector
    // ==============================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rx_done_d <= 1'b0;
        else
            rx_done_d <= rx_done;
    end
    
    assign rx_done_pulse = rx_done & ~rx_done_d;
    
    // ==============================
    // Decode-done pulse detector
    // ==============================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            decode_done_d <= 1'b0;
        else
            decode_done_d <= decode_done;
    end
    
    assign decode_done_pulse = decode_done & ~decode_done_d;

    // ==============================
    // TX FSM - NOW WITH DECODER RESET CONTROL
    // ==============================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift_reg    <= 16'd0;
            tx_send         <= 1'b0;
            decoder_reset_n <= 1'b1;     // Keep decoder in reset initially
            decoder_start   <= 1'b0;
            tx_state        <= TX_IDLE;
        end
        else begin
            case (tx_state)
                // --------------------------
                TX_IDLE: begin
                    tx_send         <= 1'b0;
                    decoder_start   <= 1'b0;
                    decoder_reset_n <= 1'b1;  // Decoder in normal operation
                    
                    // Trigger on RX done pulse (data received)
                    if (rx_done_pulse) begin
                        tx_state <= TX_RESET_DEC;
                    end
                end
                
                // --------------------------
                // NEW STATE: Reset decoder for fresh start
                // --------------------------
                TX_RESET_DEC: begin
                    decoder_reset_n <= 1'b0;  // Assert reset (active low)
                    decoder_start   <= 1'b0;
                    tx_send         <= 1'b0;
                    tx_state        <= TX_START_DEC;
                end
                
                // --------------------------
                // NEW STATE: Release reset and start decoding
                // --------------------------
                TX_START_DEC: begin
                    decoder_reset_n <= 1'b1;  // Release reset
                    decoder_start   <= 1'b1;  // Start decode (1-cycle pulse)
                    tx_send         <= 1'b0;
                    tx_state        <= TX_WAIT_DEC;
                end
                
                // --------------------------
                TX_WAIT_DEC: begin
                    decoder_start <= 1'b0;    // Clear start pulse
                    tx_send       <= 1'b0;
                    
                    // Wait for decoder to finish
                    if (decode_done_pulse) begin
                        tx_shift_reg <= decoded_data;  // Capture decoded data
                        tx_state     <= TX_SEND;
                    end
                end
                
                // --------------------------
                TX_SEND: begin
                    tx_send  <= 1'b1;      // 1-cycle pulse to start transmission
                    tx_state <= TX_IDLE;   // Return to idle
                end
                
                default:
                    tx_state <= TX_IDLE;
            endcase
        end
    end
    
    // ==============================
    // UART TX (send 16-bit decoded data)
    // ==============================
    uart_data_tx #(
        .DATA_WIDTH(16),
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
    
    // ==============================
    // DEBUG LED
    // ==============================
    assign led[1] = uart_state;      // TX busy
    assign led[0] = decode_done;     // Decoder done (better visibility)

endmodule