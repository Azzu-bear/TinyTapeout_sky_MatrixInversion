`default_nettype none

module tt_um_solve2x2 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire start = uio_in[0];
    wire sel   = uio_in[1];

    localparam S_IDLE  = 4'd0, S_LOAD  = 4'd1, S_DET_A = 4'd2, S_DET_B = 4'd3,
               S_NUM0A = 4'd4, S_NUM0B = 4'd5, S_NUM1A = 4'd6, S_NUM1B = 4'd7,
               S_DIV0  = 4'd8, S_DIV1  = 4'd9, S_LATCH = 4'd10, S_DONE = 4'd11;

    reg [3:0] state;
    reg       div_phase; 

    reg signed [3:0] a, b, c, d, e, f;
    reg [2:0] load_cnt;

    reg signed [12:0] acc; 
    reg signed [8:0]  det_save; 

    reg  signed [3:0] m1, m2;
    wire signed [7:0] mprod = m1 * m2;

    always @(*) begin
        m1 = 4'sd0; m2 = 4'sd0;
        case (state)
            S_DET_A:  begin m1 = a; m2 = d; end
            S_DET_B:  begin m1 = b; m2 = c; end
            S_NUM0A:  begin m1 = d; m2 = e; end
            S_NUM0B:  begin m1 = b; m2 = f; end
            S_NUM1A:  begin m1 = a; m2 = f; end
            S_NUM1B:  begin m1 = c; m2 = e; end
            default: ;
        endcase
    end

    reg              div_start;
    wire             div_done;
    wire signed [12:0] div_quotient;

    signed_div13by9 u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .dividend(acc), .divisor(det_save),
        .done(div_done), .quotient(div_quotient)
    );

    reg [7:0] x0_r, x1_r;
    reg valid_r, singular_r, overflow_r;

    assign uo_out  = sel ? x1_r : x0_r;
    assign uio_out = {5'b0, overflow_r, singular_r, valid_r};
    assign uio_oe  = 8'b0000_0111;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; load_cnt <= 0; div_phase <= 0;
            a <= 0; b <= 0; c <= 0; d <= 0; e <= 0; f <= 0;
            acc <= 0; det_save <= 0; x0_r <= 0; x1_r <= 0;
            valid_r <= 0; singular_r <= 0; overflow_r <= 0;
        end else begin
            div_start <= 1'b0;
            case (state)
                S_IDLE: if (start) begin
                    valid_r <= 0; singular_r <= 0; overflow_r <= 0;
                    a <= $signed(ui_in[3:0]); load_cnt <= 3'd1; state <= S_LOAD;
                end
                S_LOAD: begin
                    case (load_cnt)
                        3'd1: b <= $signed(ui_in[3:0]);
                        3'd2: c <= $signed(ui_in[3:0]);
                        3'd3: d <= $signed(ui_in[3:0]);
                        3'd4: e <= $signed(ui_in[3:0]);
                        3'd5: f <= $signed(ui_in[3:0]);
                    endcase
                    if (load_cnt == 3'd5) state <= S_DET_A;
                    else load_cnt <= load_cnt + 1'b1;
                end
                S_DET_A: begin acc <= { {5{mprod[7]}}, mprod }; state <= S_DET_B; end
                S_DET_B: begin det_save <= acc[8:0] - {mprod[7], mprod}; state <= S_NUM0A; end
                S_NUM0A: begin acc <= { {5{mprod[7]}}, mprod }; state <= S_NUM0B; end
                S_NUM0B: begin x0_r <= (acc[7:0] - mprod); state <= S_NUM1A; end
                S_NUM1A: begin acc <= { {5{mprod[7]}}, mprod }; state <= S_NUM1B; end
                S_NUM1B: begin x1_r <= (acc[7:0] - mprod); state <= S_DIV0; end
                
                S_DIV0: begin
                    if (det_save == 9'sd0) begin
                        singular_r <= 1'b1; x0_r <= 0; x1_r <= 0;
                        valid_r <= 1'b1; state <= S_DONE;
                    end else begin
                        acc <= {{5{x0_r[7]}}, x0_r} <<< 4;
                        div_start <= 1'b1; div_phase <= 1'b0; state <= S_DIV1;
                    end
                end
                S_DIV1: if (div_done) state <= S_LATCH;
                S_LATCH: begin
                    if (div_phase == 1'b0) begin
                        x0_r <= (div_quotient > 127) ? 8'h7F : (div_quotient < -128) ? 8'h80 : div_quotient[7:0];
                        overflow_r <= (div_quotient > 127 || div_quotient < -128);
                        acc <= {{5{x1_r[7]}}, x1_r} <<< 4;
                        div_start <= 1'b1; div_phase <= 1'b1; state <= S_DIV1;
                    end else begin
                        x1_r <= (div_quotient > 127) ? 8'h7F : (div_quotient < -128) ? 8'h80 : div_quotient[7:0];
                        overflow_r <= overflow_r | (div_quotient > 127 || div_quotient < -128);
                        valid_r <= 1'b1; state <= S_DONE;
                    end
                end
                S_DONE: if (!start) state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
    wire _unused = &{ena, uio_in[7:2], ui_in[7:4], 1'b0};
endmodule

module signed_div13by9 (
    input  wire clk, rst_n, start,
    input  wire signed [12:0] dividend,
    input  wire signed [8:0]  divisor,
    output reg done,
    output reg signed [12:0] quotient
);
    reg [1:0] dstate;
    reg [12:0] dvd_m, q;
    reg [8:0] dvs_m;
    reg [13:0] rem, n_rem; // Explicitly declared n_rem
    reg [3:0] idx;
    reg sign;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin 
            dstate <= 0; done <= 0; quotient <= 0;
            dvd_m <= 0; q <= 0; dvs_m <= 0; rem <= 0; n_rem <= 0; idx <= 0; sign <= 0;
        end else begin
            case (dstate)
                0: begin
                    done <= 0;
                    if (start) begin
                        sign <= dividend[12] ^ divisor[8];
                        dvd_m <= dividend[12] ? -dividend : dividend;
                        dvs_m <= divisor[8] ? -divisor : divisor;
                        rem <= 0; q <= 0; idx <= 12; dstate <= 1;
                    end
                end
                1: begin
                    n_rem = {rem[12:0], dvd_m[idx]};
                    if (n_rem >= {5'b0, dvs_m}) begin 
                        rem <= n_rem - {5'b0, dvs_m}; 
                        q[idx] <= 1'b1; 
                    end else begin 
                        rem <= n_rem; 
                        q[idx] <= 1'b0; 
                    end
                    if (idx == 0) dstate <= 2;
                    else idx <= idx - 1'b1;
                end
                2: begin 
                    quotient <= sign ? -q : q; 
                    done <= 1'b1; 
                    dstate <= 0; 
                end
                default: dstate <= 0;
            endcase
        end
    end
endmodule
