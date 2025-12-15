module convolutional_decoder (
    clk,
    reset_n,
    decode_flag,
    symbol_data,
    decoded_output,
    decode_done
);

    // I/Os:
    input           clk;
    input           reset_n;
    input           decode_flag;
    input           [2*DATA_WIDTH-1:0] symbol_data;
    output reg      [DATA_WIDTH-1:0] decoded_output;
    output reg      decode_done;
    
    // Local parameters:
    localparam IDLE         = 0;
    localparam RUN          = 1;
    localparam DONE_STATE   = 2;
    localparam MAX_VAL      = 16'd999;
    parameter  DATA_WIDTH   = 16;

    // Registers - OPTIMIZED: Reduced memory footprint
    reg        [1:0] state;
    reg        [DATA_WIDTH:0] cost [0:3][0:DATA_WIDTH+1];    // Keep 2D but smaller
    reg        [1:0] pred [0:3][0:DATA_WIDTH+1];
    reg        [1:0] best_st;
    reg        [1:0] rx [0:DATA_WIDTH-1];
    reg        [4:0] time_step;
    reg        [1:0] path [0:DATA_WIDTH];
    reg        [31:0] deinterleave_temp;

    // Integers:
    integer     k, i, j, r, c;
    
    // Table 1: The actual bit from the states chosen - OPTIMIZED: LUT structure
    function [1:0] out_bits(input [1:0] s, input b);
        case ({s, b})
            3'b000: out_bits = 2'b00;
            3'b001: out_bits = 2'b11;
            3'b010: out_bits = 2'b11;
            3'b011: out_bits = 2'b00;
            3'b100: out_bits = 2'b10;
            3'b101: out_bits = 2'b01;
            3'b110: out_bits = 2'b01;
            3'b111: out_bits = 2'b10;
        endcase
    endfunction
    
    // Calculating the Hamming distance - OPTIMIZED: Inline
    function [DATA_WIDTH:0] hamm(input [1:0] rx_val, input [1:0] exp);
        hamm = (rx_val[1] ^ exp[1]) + (rx_val[0] ^ exp[0]);
    endfunction
    
    // Next state given the current state and the input:
    function input_from_transition(input [1:0] from, input [1:0] to);
        case ({from, to})
            4'b0000: input_from_transition = 1'b0;
            4'b0010: input_from_transition = 1'b1;
            4'b0100: input_from_transition = 1'b0;
            4'b0110: input_from_transition = 1'b1;
            4'b1001: input_from_transition = 1'b0;
            4'b1011: input_from_transition = 1'b1;
            4'b1101: input_from_transition = 1'b0;
            4'b1111: input_from_transition = 1'b1;
            default: input_from_transition = 1'bx;
        endcase
    endfunction  
  
    // Combinational deinterleave (synth-friendly)
    always @(*) begin
        deinterleave_temp = 0;
        for (r=0; r<8; r=r+1) begin
            for (c=0; c<4; c=c+1) begin
                deinterleave_temp[31-(4*r+c)] = symbol_data[31-(r+c*8)];
            end
        end
    end
    
    // OPTIMIZED: Inline Viterbi step without task overhead
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            time_step <= 0;
            best_st <= 0;
            decoded_output <= 0;
            decode_done <= 0;
            for (i = 0; i<4; i=i+1) begin
                for (j=0; j<DATA_WIDTH+2; j=j+1) begin
                    cost[i][j] <= MAX_VAL;
                    pred[i][j] <= 0;
                end
            end
            cost[0][0] <= 0;
        end
        else begin
            case (state)
                // IDLE: extract symbols and prepare
                IDLE: begin
                    if (decode_flag) begin
                        state <= RUN;
                        time_step <= 1;
                        // Parallel extract all 16 symbols from deinterleaved data
                        rx[0]  <= deinterleave_temp[31:30];
                        rx[1]  <= deinterleave_temp[29:28];
                        rx[2]  <= deinterleave_temp[27:26];
                        rx[3]  <= deinterleave_temp[25:24];
                        rx[4]  <= deinterleave_temp[23:22];
                        rx[5]  <= deinterleave_temp[21:20];
                        rx[6]  <= deinterleave_temp[19:18];
                        rx[7]  <= deinterleave_temp[17:16];
                        rx[8]  <= deinterleave_temp[15:14];
                        rx[9]  <= deinterleave_temp[13:12];
                        rx[10] <= deinterleave_temp[11:10];
                        rx[11] <= deinterleave_temp[9:8];
                        rx[12] <= deinterleave_temp[7:6];
                        rx[13] <= deinterleave_temp[5:4];
                        rx[14] <= deinterleave_temp[3:2];
                      	rx[15] <= deinterleave_temp[1:0];
                        // Reset metrics
                        for (i=0; i<4; i=i+1) begin
                            for (j=0; j<DATA_WIDTH+2; j=j+1) begin
                                cost[i][j] <= MAX_VAL;
                                pred[i][j] <= 0;
                            end
                        end
                        cost[0][0] <= 0;
                    end
                end
                // RUN: Viterbi decoding
                RUN: begin
                    if (time_step == 1) begin
                        // First step: only S0 is reachable
                        cost[0][1] <= hamm(rx[0], out_bits(2'b00, 1'b0));
                        pred[0][1] <= 0;
                        cost[2][1] <= hamm(rx[0], out_bits(2'b00, 1'b1));
                        pred[2][1] <= 0;
                        cost[1][1] <= MAX_VAL;
                        cost[3][1] <= MAX_VAL;
                        time_step <= 2;
                    end
                    else if (time_step >= 2 && time_step <= DATA_WIDTH) begin
                        // INLINE Viterbi step - S00
                        if (cost[0][time_step-1] <= cost[1][time_step-1]) begin
                            cost[0][time_step] <= cost[0][time_step-1] + hamm(rx[time_step-1], out_bits(2'b00, 1'b0));
                            pred[0][time_step] <= 0;
                        end else begin
                            cost[0][time_step] <= cost[1][time_step-1] + hamm(rx[time_step-1], out_bits(2'b01, 1'b0));
                            pred[0][time_step] <= 1;
                        end
                        // S01
                        if (cost[2][time_step-1] <= cost[3][time_step-1]) begin
                            cost[1][time_step] <= cost[2][time_step-1] + hamm(rx[time_step-1], out_bits(2'b10, 1'b0));
                            pred[1][time_step] <= 2;
                        end else begin
                            cost[1][time_step] <= cost[3][time_step-1] + hamm(rx[time_step-1], out_bits(2'b11, 1'b0));
                            pred[1][time_step] <= 3;
                        end
                        // S10
                        if (cost[0][time_step-1] <= cost[1][time_step-1]) begin
                            cost[2][time_step] <= cost[0][time_step-1] + hamm(rx[time_step-1], out_bits(2'b00, 1'b1));
                            pred[2][time_step] <= 0;
                        end else begin
                            cost[2][time_step] <= cost[1][time_step-1] + hamm(rx[time_step-1], out_bits(2'b01, 1'b1));
                            pred[2][time_step] <= 1;
                        end
                        // S11
                        if (cost[2][time_step-1] <= cost[3][time_step-1]) begin
                            cost[3][time_step] <= cost[2][time_step-1] + hamm(rx[time_step-1], out_bits(2'b10, 1'b1));
                            pred[3][time_step] <= 2;
                        end else begin
                            cost[3][time_step] <= cost[3][time_step-1] + hamm(rx[time_step-1], out_bits(2'b11, 1'b1));
                            pred[3][time_step] <= 3;
                        end
                        time_step <= time_step + 5'd1;
                    end
                    else if (time_step == DATA_WIDTH+1) begin
                        // Find best final state with priority encoding
                        best_st = 0;
                        if (cost[1][DATA_WIDTH] < cost[0][DATA_WIDTH]) best_st = 1;
                        if (cost[2][DATA_WIDTH] < cost[best_st][DATA_WIDTH]) best_st = 2;
                        if (cost[3][DATA_WIDTH] < cost[best_st][DATA_WIDTH]) best_st = 3;
                        state <= DONE_STATE;
                        time_step <= 0;
                    end
                end
                // DONE_STATE: Backtrack and extract decoded bits
                DONE_STATE: begin
                    path[DATA_WIDTH] = best_st;
                    for (k = DATA_WIDTH-1; k >= 1; k = k - 1) begin
                        path[k] = pred[path[k+1]][k+1];
                    end
                    path[0] = 0;
                    // Extract bits MSB first
                    for (k = 0; k < DATA_WIDTH; k = k + 1) begin
                        decoded_output[DATA_WIDTH-1-k] <= input_from_transition(path[k], path[k+1]);
                    end
                    decode_done <= 1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule