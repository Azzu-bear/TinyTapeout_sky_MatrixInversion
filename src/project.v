// =============================================================================
// tt_um_solve2x2_q16.v
// 2x2 Linear System Solver with Q16.16 fixed-point output
// Solves Ax = b  where A=[a b; c d], b=[e; f]
//
// Solution:
//   det  = a*d - b*c
//   x0   = (d*e - b*f) / det        (Q16.16)
//   x1   = (-c*e + a*f) / det       (Q16.16)
//
// Q16.16 format: bit[31:16]=integer part, bit[15:0]=fractional part
//
// Input streaming (12 bytes, LSB first):
//   After start pulse:
//   byte 0-1  : a (signed 16-bit, lo then hi)
//   byte 2-3  : b
//   byte 4-5  : c
//   byte 6-7  : d
//   byte 8-9  : e
//   byte 10-11: f
//
// Control (uio_in):
//   [0]   start     - 1-cycle pulse to begin loading
//   [1]   sel       - 0=output x0, 1=output x1
//   [3:2] byte_sel  - which byte of the 32-bit result to output
//
// Output:
//   uo_out      - selected byte of selected result
//   uio_out[0]  - valid (stays high until next start)
//   uio_out[1]  - singular (det==0; outputs are 0)
//   uio_out[2]  - overflow (result saturated)
// =============================================================================

`default_nettype none

module tt_um_solve2x2_q16 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // -------------------------------------------------------------------------
    // Control signal decode
    // -------------------------------------------------------------------------
    wire        start    = uio_in[0];
    wire        sel      = uio_in[1];   // 0=x0, 1=x1
    wire [1:0]  byte_sel = uio_in[3:2]; // byte of 32-bit result

    // -------------------------------------------------------------------------
    // FSM states
    // -------------------------------------------------------------------------
    localparam [2:0]
        S_IDLE     = 3'd0,
        S_LOAD     = 3'd1,
        S_COMPUTE  = 3'd2,
        S_DIV_X0   = 3'd3,
        S_DIV_X1   = 3'd4,
        S_DONE     = 3'd5;

    reg [2:0] state;

    // -------------------------------------------------------------------------
    // Input registers (signed 16-bit)
    // -------------------------------------------------------------------------
    reg signed [15:0] a, b, c, d, e, f;
    reg [3:0]         load_cnt;   // counts 0..11 (12 bytes)

    // -------------------------------------------------------------------------
    // Datapath registers
    // -------------------------------------------------------------------------
    reg signed [32:0] det;    // a*d - b*c  (33 bits: 32-bit product diff + sign)
    reg signed [32:0] num0;   // d*e - b*f
    reg signed [32:0] num1;   // -c*e + a*f

    // Shifted numerators for Q16.16: num << 16 = 49 bits
    reg signed [48:0] num0_q;
    reg signed [48:0] num1_q;

    // Results (Q16.16 = 32 bits)
    reg signed [31:0] x0, x1;

    // Status flags
    reg valid, singular, overflow_flag;

    // -------------------------------------------------------------------------
    // Divider interface
    // -------------------------------------------------------------------------
    reg         div_start;
    wire        div_done;
    wire        div_busy;
    reg  signed [48:0] div_num;
    reg  signed [32:0] div_den;
    wire signed [48:0] div_quot;
    wire signed [48:0] div_rem;

    restoring_div #(
        .NUM_W(49),
        .DEN_W(33)
    ) u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .numerator  (div_num),
        .denominator(div_den),
        .done       (div_done),
        .busy       (div_busy),
        .quotient   (div_quot),
        .remainder  (div_rem)
    );

    // -------------------------------------------------------------------------
    // Saturation: clamp 49-bit signed quotient to 32-bit signed Q16.16
    // Overflow if quotient doesn't fit in [-2^31, 2^31-1]
    // Computed as wires so they are valid combinatorially — no automatic vars.
    // -------------------------------------------------------------------------
    // div_quot is 49-bit signed. It overflows Q16.16 if the upper 17 bits are
    // not all equal to the sign bit (i.e., not all 0s or all 1s).
    wire quot_pos_ovf = (!div_quot[48]) && (|div_quot[48:31]);   // > 0x7FFFFFFF
    wire quot_neg_ovf = ( div_quot[48]) && (!(&div_quot[48:31])); // < 0x80000000

    wire [31:0] sat_result = quot_pos_ovf ? 32'h7FFF_FFFF :
                             quot_neg_ovf ? 32'h8000_0000 :
                                            div_quot[31:0];
    wire        sat_ovf    = quot_pos_ovf | quot_neg_ovf;

    // -------------------------------------------------------------------------
    // Output mux
    // -------------------------------------------------------------------------
    wire [31:0] result_out = sel ? x1 : x0;

    assign uo_out = byte_sel[1]
                  ? (byte_sel[0] ? result_out[31:24] : result_out[23:16])
                  : (byte_sel[0] ? result_out[15:8]  : result_out[7:0]);

    assign uio_out = {5'b0, overflow_flag, singular, valid};
    assign uio_oe  = 8'b0000_0111;  // drive [2:0] only

    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            load_cnt     <= 4'd0;
            a <= 0; b <= 0; c <= 0; d <= 0; e <= 0; f <= 0;
            det  <= 0; num0 <= 0; num1 <= 0;
            num0_q <= 0; num1_q <= 0;
            x0 <= 0; x1 <= 0;
            valid        <= 1'b0;
            singular     <= 1'b0;
            overflow_flag<= 1'b0;
            div_start    <= 1'b0;
            div_num      <= 0;
            div_den      <= 1;
        end else begin
            div_start <= 1'b0;  // default: deassert each cycle

            case (state)

                // -----------------------------------------------------------------
                // Sample byte 0 (a_lo) on the SAME cycle as start so we don't
                // lose it when the FSM transitions to S_LOAD on the next edge.
                S_IDLE: begin
                    if (start) begin
                        valid         <= 1'b0;
                        singular      <= 1'b0;
                        overflow_flag <= 1'b0;
                        a[7:0]        <= ui_in;   // byte 0: a_lo captured now
                        load_cnt      <= 4'd1;    // next byte to load is byte 1
                        state         <= S_LOAD;
                    end
                end

                // -----------------------------------------------------------------
                // Stream in remaining 11 bytes (bytes 1..11)
                // -----------------------------------------------------------------
                S_LOAD: begin
                    case (load_cnt)
                        4'd1:  a[15:8] <= ui_in;
                        4'd2:  b[7:0]  <= ui_in;
                        4'd3:  b[15:8] <= ui_in;
                        4'd4:  c[7:0]  <= ui_in;
                        4'd5:  c[15:8] <= ui_in;
                        4'd6:  d[7:0]  <= ui_in;
                        4'd7:  d[15:8] <= ui_in;
                        4'd8:  e[7:0]  <= ui_in;
                        4'd9:  e[15:8] <= ui_in;
                        4'd10: f[7:0]  <= ui_in;
                        4'd11: f[15:8] <= ui_in;
                        default: ;
                    endcase

                    if (load_cnt == 4'd11)
                        state <= S_COMPUTE;
                    else
                        load_cnt <= load_cnt + 1'b1;
                end

                // -----------------------------------------------------------------
                // Compute det, num0, num1, then shift for Q16.16
                // One cycle for multiply-accumulate, next cycle for shift
                // -----------------------------------------------------------------
                S_COMPUTE: begin
                    det    <= (a * d) - (b * c);
                    num0   <= (d * e) - (b * f);
                    num1   <= (-(c * e)) + (a * f);
                    state  <= S_DIV_X0;
                end

                // -----------------------------------------------------------------
                // Divide num0<<16 / det  -> x0
                // -----------------------------------------------------------------
                S_DIV_X0: begin
                    if (det == 0) begin
                        singular <= 1'b1;
                        x0       <= 32'sd0;
                        x1       <= 32'sd0;
                        valid    <= 1'b1;
                        state    <= S_DONE;
                    end else if (!div_busy) begin
                        div_num   <= {num0, 16'b0};   // num0 << 16
                        div_den   <= det;
                        div_start <= 1'b1;
                        state     <= S_DIV_X1;
                    end
                end

                // -----------------------------------------------------------------
                // Wait for x0 result, then divide num1<<16 / det -> x1
                // -----------------------------------------------------------------
                S_DIV_X1: begin
                    if (div_done) begin
                        // Capture x0 with saturation (wires sat_result/sat_ovf)
                        x0            <= sat_result;
                        overflow_flag <= sat_ovf;
                        // Kick off x1 division
                        div_num   <= {num1, 16'b0};
                        div_den   <= det;
                        div_start <= 1'b1;
                        state     <= S_DONE;
                    end
                end

                // -----------------------------------------------------------------
                // Wait for x1 result, assert valid
                // -----------------------------------------------------------------
                S_DONE: begin
                    if (div_done) begin
                        x1            <= sat_result;
                        overflow_flag <= overflow_flag | sat_ovf;
                        valid         <= 1'b1;
                    end
                // Stay in DONE (valid held high) until next start pulse.
                // Priority: capture x1 first if div_done, then handle start.
                    if (start && !div_done) begin
                        valid         <= 1'b0;
                        singular      <= 1'b0;
                        overflow_flag <= 1'b0;
                        a[7:0]        <= ui_in;
                        load_cnt      <= 4'd1;
                        state         <= S_LOAD;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    wire _unused = &{ena, uio_in[7:4], div_rem, 1'b0};

endmodule


// =============================================================================
// restoring_div
// Signed restoring division: quotient = numerator / denominator (trunc to zero)
// Latency: NUM_W cycles after start. done pulses for 1 cycle.
// =============================================================================
module restoring_div #(
    parameter int NUM_W = 49,
    parameter int DEN_W = 33
) (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      start,
    input  wire signed [NUM_W-1:0]   numerator,
    input  wire signed [DEN_W-1:0]   denominator,
    output reg                       done,
    output reg                       busy,
    output reg  signed [NUM_W-1:0]   quotient,
    output reg  signed [NUM_W-1:0]   remainder
);
    // Working registers (unsigned)
    reg [NUM_W-1:0]   num_abs;
    reg [DEN_W-1:0]   den_abs;
    reg               num_neg;
    reg               den_neg;

    reg [NUM_W-1:0]   q_reg;
    reg [NUM_W:0]     r_reg;    // one extra bit for comparison

    reg [$clog2(NUM_W):0] step;  // counts 0..NUM_W-1

    localparam [1:0] D_IDLE = 2'd0, D_RUN = 2'd1, D_FINISH = 2'd2;
    reg [1:0] dstate;

    // Current bit of numerator being shifted in
    wire cur_bit = num_abs[NUM_W-1-step];

    // Partial remainder with new bit shifted in
    wire [NUM_W:0] r_shifted = {r_reg[NUM_W-1:0], cur_bit};

    // Zero-extend denominator to match r_shifted width
    wire [NUM_W:0] den_ext = {{(NUM_W+1-DEN_W){1'b0}}, den_abs};

    wire           sub_ok  = (r_shifted >= den_ext);
    wire [NUM_W:0] r_next  = sub_ok ? (r_shifted - den_ext) : r_shifted;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done      <= 1'b0;
            busy      <= 1'b0;
            quotient  <= '0;
            remainder <= '0;
            num_abs   <= '0;
            den_abs   <= '0;
            num_neg   <= 1'b0;
            den_neg   <= 1'b0;
            q_reg     <= '0;
            r_reg     <= '0;
            step      <= '0;
            dstate    <= D_IDLE;
        end else begin
            done <= 1'b0;

            case (dstate)
                D_IDLE: begin
                    if (start) begin
                        num_neg <= numerator[NUM_W-1];
                        den_neg <= denominator[DEN_W-1];
                        num_abs <= numerator[NUM_W-1]   ? -numerator   : numerator;
                        den_abs <= denominator[DEN_W-1] ? -denominator : denominator;
                        q_reg   <= '0;
                        r_reg   <= '0;
                        step    <= '0;
                        busy    <= 1'b1;
                        dstate  <= D_RUN;
                    end
                end

                D_RUN: begin
                    // Shift quotient bit in
                    q_reg <= {q_reg[NUM_W-2:0], sub_ok};
                    r_reg <= r_next;

                    if (step == NUM_W-1) begin
                        dstate <= D_FINISH;
                    end else begin
                        step <= step + 1'b1;
                    end
                end

                D_FINISH: begin
                    busy <= 1'b0;
                    done <= 1'b1;

                    // Apply sign (truncate toward zero)
                    quotient  <= (num_neg ^ den_neg) ? -$signed(q_reg) : $signed(q_reg);
                    remainder <= num_neg             ? -$signed(r_reg[NUM_W-1:0])
                                                     :  $signed(r_reg[NUM_W-1:0]);
                    dstate <= D_IDLE;
                end

                default: dstate <= D_IDLE;
            endcase
        end
    end
endmodule
