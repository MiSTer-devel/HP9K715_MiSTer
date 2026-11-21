// One FP operation at a time through the pipelined datapath (rtl/cpu/pa_fpdp.sv): the harness the
// unit bench drives (verilator/tb/tb_fpu.cpp, `make -C verilator tb_fpdp`; the module is named
// pa_fpu so that the bench of the sequential engine, rtl/cpu/pa_fpu.sv, drives both).
// Not part of the core: pa_fpunit instantiates pa_fpdp itself and moves it
// with the pipeline.
//
// `start` presents the operation as the instruction leaving D; it then moves
// through the datapath's stages (E, M, W), waits in W while a divide or
// square root iterates, and `done` pulses with the registered outputs. The
// inputs must hold until then.
module pa_fpu
    import pa_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  fpu_op_e     op,
    input  logic        sgl,            // source format single (dest too, except the converts)
    input  logic        dsgl,           // FCNV: destination format single
    input  logic [63:0] a, b,
    input  logic [1:0]  rm,             // FPSR rounding mode
    input  logic        dmode,          // FPSR D (hardware underflow mode)
    input  logic [4:0]  enables,        // FPSR trap enables {V, Z, O, U, I}
    input  logic [4:0]  cond,           // FCMP condition
    output logic        busy,
    output logic        done,           // one cycle, with the outputs below
    output logic [63:0] result,
    output logic        result_we,
    output logic [4:0]  flags,          // {V, Z, O, U, I} to set
    output logic        unimpl,         // unimplemented exception (delayed trap)
    output logic        cmp_c           // FCMP result
);
    typedef enum logic [1:0] { P_IDLE, P_E, P_M, P_W } pos_e;
    pos_e        pos;
    logic        w_hold;
    logic [63:0] dp_result;
    logic        dp_we, dp_unimpl, dp_cmp, dp_m_slow, dp_slow_xfer;
    logic [4:0]  dp_flags;
    wire adv = !(pos == P_W && w_hold);
    assign busy = pos != P_IDLE;
    // single operands as pa_fpunit hands them over: nothing in the low half
    wire [63:0] oa = sgl ? {a[63:32], 32'd0} : a;
    wire [63:0] ob = sgl ? {b[63:32], 32'd0} : b;
    pa_fpdp u_dp (
        .clk(clk), .rst_n(rst_n), .adv(adv), .kill(1'b0),
        .d_op(op), .d_sgl(sgl), .d_dsgl(dsgl), .d_cond(cond), .d_a(oa), .d_b(ob),
        .rm(rm), .dmode(dmode), .enables(enables),
        .e_live(pos == P_E),
        .result(dp_result), .result_we(dp_we), .flags(dp_flags), .unimpl(dp_unimpl), .cmp_c(dp_cmp),
        .m_cmp_c(), .m_cmp_un(), .w_cmp_un(),
        .m_slow(dp_m_slow), .slow_xfer(dp_slow_xfer));
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pos <= P_IDLE; w_hold <= 1'b0; done <= 1'b0;
            result <= 64'd0; result_we <= 1'b0; flags <= 5'd0; unimpl <= 1'b0; cmp_c <= 1'b0;
        end else begin
            done <= 1'b0;
            case (pos)
                P_IDLE: if (start) pos <= P_E;
                P_E: pos <= P_M;
                P_M: begin pos <= P_W; w_hold <= dp_m_slow; end
                default: begin
                    if (w_hold) w_hold <= !dp_slow_xfer;
                    else begin
                        pos <= P_IDLE; done <= 1'b1;
                        result <= dp_result; result_we <= dp_we; flags <= dp_flags; unimpl <= dp_unimpl; cmp_c <= dp_cmp;
                    end
                end
            endcase
        end
    end
endmodule
