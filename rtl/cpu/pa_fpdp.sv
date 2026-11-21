// PA-7100LC floating-point datapath (docs/design/cpu.md 3.5, 6.11; ERS ch. 7;
// ACD ch. 6), in four stages that move with the core's pipeline
// (cpu-timing.md 10):
//
//   D  the operands as read from the FR file (pa_fpunit): their classes, the
//      magnitude compare, the exponent difference -> the alignment distance;
//      the multiplier's operand registers
//   E  align and add / subtract; the 53 x 53 multiply; FCNVFX's shift; the
//      decisions that need no datapath (ERS s.7.3); FDIV / FSQRT start
//   M  count the leading zeroes and normalise; FCNVFX's rounding and negation
//   W  round, the PCX-L exception rules -> result, flags, the compare bit or
//      the unimplemented exception, combinational, for the commit of the
//      instruction in W
//
// One operation per stage; an operation is in the first stage while its
// instruction is in E, and so on (the multiply of a pair instruction runs one
// stage ahead of its instruction, pa_fpunit).
//
// FDIV and FSQRT are iterative (3 quotient bits or 2 root bits a cycle): the
// recurrence starts while the instruction is in E and runs on its own; when
// it is done and the instruction is in W, its result goes through the M
// stage's normaliser into the W registers (`slow_xfer`), and W, which
// pa_fpunit holds from a flip-flop meanwhile, rounds it the cycle after.
//
// The PCX-L model (ERS s.7.3): the only exception the hardware raises is
// the unimplemented exception (a delayed trap): any NaN operand, a denormal
// operand or a tiny result with FPSR.D = 0, any IEEE exception whose trap
// is enabled, FCNVFX overflow, FRND, quad formats and reserved sub-ops. The
// datapath therefore never computes with denormals: with D = 1 they are
// signed zeroes (and tiny results flush to zero with the inexact flag).
//
// Formats: single operands arrive left-justified in bits 63:32 (bits 31:0
// zero); a single result returns in both halves (so the register file's
// write needs no select behind the rounding). Internally every number is {sign,
// true exponent E, 53-bit significand with the leading 1 at bit 52}; the
// rounding step takes {significand, guard, round} plus a sticky bit. NaNs:
// PA-RISC marks a signalling NaN with fraction bit 1 = 1 (the opposite of the
// usual convention); the default quiet NaN is 0x7FF4000000000000 / 0x7FA00000.
module pa_fpdp
    import pa_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        adv,            // the pipeline moves
    input  logic        kill,           // everything in flight is dropped
    // ---- D: the operation that enters the first stage when the pipeline moves
    input  fpu_op_e     d_op,
    input  logic        d_sgl,          // source format single (dest too, except the converts)
    input  logic        d_dsgl,         // FCNV: destination format single
    input  logic [4:0]  d_cond,         // FCMP condition
    input  logic [63:0] d_a, d_b,
    input  logic [1:0]  rm,             // FPSR rounding mode
    input  logic        dmode,          // FPSR D (hardware underflow mode)
    input  logic [4:0]  enables,        // FPSR trap enables {V, Z, O, U, I}
    // ---- E
    input  logic        e_live,         // the operation in the first stage is a valid, non-nullified instruction's
    // ---- W
    output logic [63:0] result,
    output logic        result_we,
    output logic [4:0]  flags,          // {V, Z, O, U, I} to set
    output logic        unimpl,         // unimplemented exception (delayed trap)
    output logic        cmp_c,          // FCMP result
    // ---- the compare bit of the operations in M and W, for an FTEST behind them
    output logic        m_cmp_c, m_cmp_un,  // M: the bit, or the compare is unimplemented (no bit)
    output logic        w_cmp_un,           // W (the bit is cmp_c)
    // ---- the iterative operations
    output logic        m_slow,         // the operation in M is a divide / square root that iterates
    output logic        slow_xfer       // its result enters the W registers at the end of this cycle
);
    typedef enum logic [2:0] { C_ZERO, C_DENORM, C_NORM, C_INF, C_QNAN, C_SNAN } cls_e;

    // the IEEE default results
    function automatic logic [63:0] pack_inf(input logic s, input logic single);
        return single ? {s, 8'hFF, 23'd0, 32'd0} : {s, 11'h7FF, 52'd0};
    endfunction
    function automatic logic [63:0] pack_max(input logic s, input logic single);
        return single ? {s, 8'hFE, {23{1'b1}}, 32'd0} : {s, 11'h7FE, {52{1'b1}}};
    endfunction
    function automatic logic [63:0] pack_zero(input logic s, input logic single);
        return single ? {s, 31'd0, 32'd0} : {s, 63'd0};
    endfunction
    localparam logic [63:0] QNAN_D = 64'h7FF4_0000_0000_0000;
    localparam logic [63:0] QNAN_S = 64'h7FA0_0000_0000_0000;

    // leading zeroes of 64 bits as a tree: nibbles, groups of four nibbles, the
    // word -- three levels of logic (a priority chain over the bits is ten)
    function automatic logic [6:0] lzc64(input logic [63:0] v);
        logic [15:0] nz;                // nibble i (0 = the top one) is not zero
        logic [31:0] np;                // its leading zeroes, 2 bits each
        logic [3:0]  gz;                // group g of four nibbles is not zero
        logic [15:0] gp;                // its leading zeroes, 4 bits each
        logic [3:0]  n;
        for (int i = 0; i < 16; i++) begin
            n = v[63 - 4*i -: 4];
            nz[i] = |n;
            np[2*i +: 2] = n[3] ? 2'd0 : n[2] ? 2'd1 : n[1] ? 2'd2 : 2'd3;
        end
        for (int k = 0; k < 4; k++) begin
            gz[k] = |nz[4*k +: 4];
            gp[4*k +: 4] = nz[4*k]     ? {2'd0, np[2*(4*k)     +: 2]} :
                           nz[4*k + 1] ? {2'd1, np[2*(4*k + 1) +: 2]} :
                           nz[4*k + 2] ? {2'd2, np[2*(4*k + 2) +: 2]} :
                                         {2'd3, np[2*(4*k + 3) +: 2]};
        end
        lzc64 = gz[0] ? {3'd0, gp[3:0]} : gz[1] ? {3'd1, gp[7:4]} : gz[2] ? {3'd2, gp[11:8]} :
                gz[3] ? {3'd3, gp[15:12]} : 7'd64;
    endfunction
    function automatic logic [5:0] lzc58(input logic [57:0] v);     // 58 for zero
        logic [6:0] c;
        c = lzc64({v, 6'b111111});
        return c[5:0];
    endfunction
    wire en_v = enables[4], en_z = enables[3], en_o = enables[2], en_i = enables[0];

    // ================================================================ D: classes, compare, distances
    function automatic cls_e classify(input logic [63:0] v, input logic single, input logic dm);
        logic emax, ez, fz, f_msb;
        if (single) begin emax = &v[62:55]; ez = v[62:55] == 8'd0;  fz = v[54:32] == 23'd0; f_msb = v[54]; end
        else        begin emax = &v[62:52]; ez = v[62:52] == 11'd0; fz = v[51:0] == 52'd0;  f_msb = v[51]; end
        if (emax)    classify = fz ? C_INF : f_msb ? C_SNAN : C_QNAN;
        else if (ez) classify = (fz || dm) ? C_ZERO : C_DENORM;      // hardware underflow mode: denormals are signed zeroes
        else         classify = C_NORM;
    endfunction
    wire [10:0] d_ea = d_sgl ? {3'd0, d_a[62:55]} : d_a[62:52];     // biased exponents
    wire [10:0] d_eb = d_sgl ? {3'd0, d_b[62:55]} : d_b[62:52];
    wire [11:0] d_bias52 = d_sgl ? 12'd179 : 12'd1075;              // bias + 52
    // the magnitudes order as their bit patterns do (a zero or a denormal below every normal)
    wire        d_age = d_a[62:0] >= d_b[62:0];
    wire        d_aeq = d_a[62:0] == d_b[62:0];
    logic [5:0]  d_sh;                  // the right shift: the alignment, or FCNVFX's 52 - E
    logic [3:0]  d_lsh;                 // FCNVFX with E >= 52: the left shift E - 52
    logic        d_ege52, d_egt63;
    always_comb begin
        logic [11:0] dab, dba, cr;
        logic [5:0]  sh_ab, sh_ba, sh_cr;
        // both exponent differences beside the magnitude compare that picks one; a
        // distance of 64 or more saturates to 63 (one level: everything is shifted
        // out from 58 on, and the operands' top bits are zero)
        dab = {1'b0, d_ea} - {1'b0, d_eb};
        dba = {1'b0, d_eb} - {1'b0, d_ea};
        cr = d_bias52 - {1'b0, d_ea};                               // 52 - E
        sh_ab = dab[5:0] | {6{|dab[11:6]}};
        sh_ba = dba[5:0] | {6{|dba[11:6]}};
        sh_cr = cr[5:0] | {6{|cr[11:6]}};
        d_ege52 = {1'b0, d_ea} >= d_bias52;
        d_egt63 = {1'b0, d_ea} > d_bias52 + 12'd11;
        d_lsh = 4'({1'b0, d_ea} - d_bias52);
        if (d_op == F_ADD || d_op == F_SUB) d_sh = d_age ? sh_ab : sh_ba;
        else                                d_sh = sh_cr;
    end

    // ================================================================ stage registers
    // ---- first stage (the instruction in E)
    fpu_op_e     x1_op;
    logic        x1_sgl, x1_dsgl;
    logic [4:0]  x1_cond;
    logic [63:0] x1_a, x1_b;
    cls_e        x1_ca, x1_cb;
    logic        x1_age, x1_aeq, x1_ege52, x1_egt63;
    logic [5:0]  x1_sh;
    logic [3:0]  x1_lsh;
    logic [52:0] mul_a, mul_b;          // the significands (the integers of XMPYU)
    // ---- second stage (M)
    fpu_op_e     x2_op;
    logic        x2_sgl, x2_dsgl, x2_osgl, x2_as;
    logic [57:0] x2_v;
    logic        x2_sticky, x2_rs;
    logic signed [13:0] x2_re;
    logic [63:0] x2_x;
    logic        x2_cg, x2_cr, x2_cs;
    logic        x2_early, x2_eun, x2_ewe, x2_efv, x2_efz, x2_ecmp, x2_slow, x2_dup;
    logic [105:0] prod;
    // ---- third stage (W)
    logic [57:0] x3_nv;
    logic        x3_st, x3_rs, x3_osgl;         // x3_st: everything below the round bit
    logic signed [13:0] x3_ne;
    logic        x3_ovf0, x3_ovf1, x3_tiny0, x3_tiny1;
    logic        x3_byp, x3_bwe, x3_bun, x3_bcmp;
    logic [4:0]  x3_bfl;
    logic [63:0] x3_x;
    logic        x3_slow;
    assign m_slow = x2_slow;
    assign m_cmp_c = x2_ecmp; assign m_cmp_un = x2_eun; assign w_cmp_un = x3_bun;

    // ================================================================ E: unpack, the early decisions
    wire        sa = x1_a[63], sb = x1_b[63];
    wire [10:0] ea_r = x1_sgl ? {3'd0, x1_a[62:55]} : x1_a[62:52];
    wire [10:0] eb_r = x1_sgl ? {3'd0, x1_b[62:55]} : x1_b[62:52];
    wire signed [13:0] bias = x1_sgl ? 14'sd127 : 14'sd1023;
    wire signed [13:0] ea = 14'(signed'({3'd0, ea_r})) - bias;      // true exponents
    wire signed [13:0] eb = 14'(signed'({3'd0, eb_r})) - bias;
    // a zero (or a flushed denormal) that reaches the add/subtract datapath
    // (x + 0) must weigh nothing: no implicit leading one
    wire [52:0] sig_a = (x1_ca == C_ZERO) ? 53'd0 : mul_a;
    wire [52:0] sig_b = (x1_cb == C_ZERO) ? 53'd0 : mul_b;
    wire a_nan = x1_ca == C_QNAN || x1_ca == C_SNAN, b_nan = x1_cb == C_QNAN || x1_cb == C_SNAN;
    wire a_den = x1_ca == C_DENORM, b_den = x1_cb == C_DENORM;
    wire a_zero = x1_ca == C_ZERO, b_zero = x1_cb == C_ZERO, a_inf = x1_ca == C_INF, b_inf = x1_cb == C_INF;
    wire a_norm = x1_ca == C_NORM, b_norm = x1_cb == C_NORM;
    wire b_sign_eff = (x1_op == F_SUB) ? ~sb : sb;
    wire e_osgl = (x1_op == F_CNVFF || x1_op == F_CNVXF) ? x1_dsgl : x1_sgl;

    logic        early_done, early_unimpl, early_we, early_cmp;
    logic [63:0] early_result;
    logic        early_fv, early_fz;                 // the V and Z flags
    logic cmp_lt, cmp_eq, cmp_gt, cmp_un;
    always_comb begin
        early_done = 1'b0; early_unimpl = 1'b0; early_we = 1'b0; early_result = 64'd0;
        early_fv = 1'b0; early_fz = 1'b0; early_cmp = 1'b0;
        cmp_lt = 0; cmp_eq = 0; cmp_gt = 0; cmp_un = a_nan || b_nan;
        case (x1_op)
            F_CPY: begin early_done = 1'b1; early_we = 1'b1; early_result = x1_a; end
            F_ABS: begin early_done = 1'b1; early_we = 1'b1; early_result = {1'b0, x1_a[62:0]}; end
            F_RND, F_RESERVED: begin early_done = 1'b1; early_unimpl = 1'b1; end
            F_XMPYU: ;                              // the multiplier
            F_CMP: begin
                early_done = 1'b1;
                if (x1_ca == C_SNAN || x1_cb == C_SNAN || ((a_nan || b_nan) && x1_cond[0])) early_unimpl = 1'b1;
                else begin
                    // sign-magnitude; denormals count by their bits
                    if (cmp_un) ;
                    else if (a_zero && b_zero) cmp_eq = 1;
                    else if (sa != sb) begin cmp_lt = sa; cmp_gt = sb; end
                    else if (x1_aeq) cmp_eq = 1;
                    else begin cmp_lt = ~x1_age ^ sa; cmp_gt = ~cmp_lt; end
                    early_cmp = (cmp_gt & x1_cond[4]) | (cmp_lt & x1_cond[3]) | (cmp_eq & x1_cond[2]) | (cmp_un & x1_cond[1]);
                end
            end
            F_ADD, F_SUB: begin
                if (a_nan || b_nan) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if ((a_den && !b_inf) || (b_den && !a_inf)) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if (a_inf && b_inf) begin
                    early_done = 1'b1;
                    if (sa != b_sign_eff) begin                 // inf - inf
                        if (en_v) early_unimpl = 1'b1;
                        else begin early_we = 1'b1; early_result = x1_sgl ? QNAN_S : QNAN_D; early_fv = 1'b1; end
                    end else begin early_we = 1'b1; early_result = pack_inf(sa, x1_sgl); end
                end else if (a_inf) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_inf(sa, x1_sgl); end
                else if (b_inf) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_inf(b_sign_eff, x1_sgl); end
                else if (a_zero && b_zero) begin
                    early_done = 1'b1; early_we = 1'b1;
                    // +0 + -0 = +0 except in round-to-minus-infinity
                    early_result = pack_zero((sa == b_sign_eff) ? sa : (rm == 2'd3), x1_sgl);
                end
            end
            F_MPY: begin
                if (a_nan || b_nan) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if ((a_den && !(b_zero || b_inf)) || (b_den && !(a_zero || a_inf))) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if ((a_inf && b_zero) || (a_zero && b_inf)) begin
                    early_done = 1'b1;
                    if (en_v) early_unimpl = 1'b1;
                    else begin early_we = 1'b1; early_result = x1_sgl ? QNAN_S : QNAN_D; early_fv = 1'b1; end
                end else if (a_inf || b_inf) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_inf(sa ^ sb, x1_sgl); end
                else if (a_zero || b_zero) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_zero(sa ^ sb, x1_sgl); end
            end
            F_DIV: begin
                // ERS s.7.3 lists denorm/denorm, denorm/norm and norm/denorm (D = 0) as
                // unimplemented; denorm/zero is a division by zero, and the trivial
                // zero/denorm, inf/denorm, denorm/inf complete (STUBS S-013)
                if (a_nan || b_nan) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if ((a_den && (b_den || b_norm)) || (b_den && a_norm)) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if ((a_inf && b_inf) || (a_zero && b_zero)) begin
                    early_done = 1'b1;
                    if (en_v) early_unimpl = 1'b1;
                    else begin early_we = 1'b1; early_result = x1_sgl ? QNAN_S : QNAN_D; early_fv = 1'b1; end
                end else if (b_zero) begin                      // x / 0
                    early_done = 1'b1;
                    if (en_z) early_unimpl = 1'b1;
                    else begin early_we = 1'b1; early_result = pack_inf(sa ^ sb, x1_sgl); early_fz = 1'b1; end
                end else if (a_inf) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_inf(sa ^ sb, x1_sgl); end
                else if (b_inf || a_zero) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_zero(sa ^ sb, x1_sgl); end
            end
            F_SQRT: begin
                // ERS s.7.3: a negative denormal (D = 0) is an invalid operation like a
                // negative normal; only a positive denormal is unimplemented
                if (a_nan) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if (a_zero) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_zero(sa, x1_sgl); end
                else if (sa) begin                               // negative: invalid
                    early_done = 1'b1;
                    if (en_v) early_unimpl = 1'b1;
                    else begin early_we = 1'b1; early_result = x1_sgl ? QNAN_S : QNAN_D; early_fv = 1'b1; end
                end else if (a_den) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if (a_inf) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_inf(1'b0, x1_sgl); end
            end
            F_CNVFF: begin
                if (a_nan || a_den) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if (a_inf) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_inf(sa, x1_dsgl); end
                else if (a_zero) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_zero(sa, x1_dsgl); end
            end
            F_CNVFX, F_CNVFXT: begin
                if (a_nan || a_inf || a_den) begin early_done = 1'b1; early_unimpl = 1'b1; end
                else if (a_zero) begin early_done = 1'b1; early_we = 1'b1; early_result = 64'd0; end
                else if (x1_egt63) begin early_done = 1'b1; early_unimpl = 1'b1; end   // beyond any integer
            end
            F_CNVXF: begin
                if (x1_a == 64'd0) begin early_done = 1'b1; early_we = 1'b1; early_result = pack_zero(1'b0, x1_dsgl); end
            end
            default: ;
        endcase
    end

    // ================================================================ E: align, add / subtract, FCNVFX's shift
    // the one right shifter with a sticky bit, shared by the add/subtract
    // alignment and the float-to-integer conversion
    wire is_addsub = x1_op == F_ADD || x1_op == F_SUB;
    wire [52:0] sig_big   = x1_age ? sig_a : sig_b;
    wire [52:0] sig_small = x1_age ? sig_b : sig_a;
    logic [57:0] sh_in, sh_out;
    logic        sh_lost;
    always_comb begin
        // add/subtract: operands as {sig, 3'b0} in 56 bits, two guard bits of headroom at the top;
        // FCNVFX(T): the units bit is bit 52 of sig, shifted right by 52 - E with two guard bits
        sh_in = is_addsub ? {2'b00, sig_small, 3'd0} : {3'd0, sig_a, 2'b00};
        sh_out = sh_in >> x1_sh;
        sh_lost = (x1_sh != 0) && ((sh_in & ((58'd1 << x1_sh) - 58'd1)) != 0);
    end
    logic [57:0] s1_big, s1_small;
    logic        s1_sticky, s1_effsub, s1_rs;
    logic signed [13:0] s1_re;
    logic [63:0] s1_x;
    logic        s1_cg, s1_cr, s1_cs;
    logic [57:0] s1_sum;
    always_comb begin
        logic [63:0] xv, mag;
        s1_big = 58'd0; s1_small = 58'd0; s1_sticky = 1'b0; s1_effsub = 1'b0; s1_rs = sa; s1_re = ea;
        s1_x = early_result; s1_cg = 1'b0; s1_cr = 1'b0; s1_cs = 1'b0;
        xv = x1_sgl ? {{32{x1_a[63]}}, x1_a[63:32]} : x1_a;
        mag = xv[63] ? -xv : xv;
        case (x1_op)
            F_ADD, F_SUB: begin
                s1_big = {2'b00, sig_big, 3'd0};
                s1_small = sh_out; s1_sticky = sh_lost;
                s1_effsub = sa != b_sign_eff;
                s1_rs = x1_age ? sa : b_sign_eff;
                s1_re = (x1_age ? ea : eb) + 14'sd2;           // the leading one sits at bit 55
            end
            F_MPY: begin s1_rs = sa ^ sb; s1_re = ea + eb + 14'sd1; end
            F_CNVFF: begin                                     // a normal in: rounded to the other format
                s1_big = {2'b00, sig_a, 3'd0}; s1_re = ea + 14'sd2;
            end
            F_CNVXF: begin                                     // the magnitude, normalised in M
                s1_rs = xv[63];
                if (|mag[63:58]) begin s1_big = mag[63:6]; s1_sticky = |mag[5:0]; s1_re = 14'sd63; end
                else             begin s1_big = mag[57:0];                         s1_re = 14'sd57; end
            end
            F_CNVFX, F_CNVFXT: if (!early_done) begin          // the magnitude shifted to the units position
                if (x1_ege52) s1_x = {11'd0, sig_a} << x1_lsh;
                else begin s1_x = {11'd0, sh_out[54:2]}; s1_cg = sh_out[1]; s1_cr = sh_out[0]; s1_cs = sh_lost; end
            end
            F_DIV:  begin s1_rs = sa ^ sb; s1_re = ea - eb; end              // the quotient's units bit
            F_SQRT: begin s1_rs = 1'b0; s1_re = (ea - 14'(signed'({13'd0, ea[0]}))) >>> 1; end   // E odd -> (E-1)/2
            default: ;
        endcase
        if (s1_effsub) s1_sum = s1_big - s1_small - {57'd0, s1_sticky};      // the sticky bit borrows
        else           s1_sum = s1_big + s1_small;
    end
    wire e_slow_op = x1_op == F_DIV || x1_op == F_SQRT;
    wire e_slow = e_slow_op && !early_done;

    // ================================================================ the iterative divide / square root
    // divide: restoring, three quotient bits a cycle against 2b, so the first
    // digit is the units digit (a/b in (0.5, 2)): q = floor(a * 2^(N-1) / b),
    // N = 57 (double) or 27 (single) -- the significand, guard and round bits
    // after a one-bit normalisation; the remainder is the sticky bit.
    // square root: digit by digit, two root bits a cycle: 56 (double) or 28
    // (single) bits of sqrt(m), m = sig (E even) or 2 sig (E odd).
    typedef enum logic [1:0] { Q_IDLE, Q_RUN, Q_FIN, Q_DONE } qst_e;
    qst_e        qst;
    logic        q_sqrt, q_sgl, q_rs, e_started, q_abort;
    logic signed [13:0] q_re;
`ifndef PA_FP_DIVB
`define PA_FP_DIVB 2
`endif
`ifndef PA_FP_SQB
`define PA_FP_SQB 1
`endif
    // quotient bits (1, 2 or 3) and root bits (1 or 2) a cycle: 57 / 27 quotient bits and 56 / 28
    // root bits in all. 3 quotient bits or 2 root bits chain that many 57-bit subtracters, and
    // the whole machine's fit then misses 64 MHz by 2-3 ns in the *core* (cpu-timing.md 11).
    // 2 quotient bits are a radix-4 step with the three candidate remainders subtracted side
    // by side (the one-bit step's depth); the odd first bit is a one-bit step.
    localparam int DIVB = `PA_FP_DIVB, SQB = `PA_FP_SQB;
    logic [5:0]  q_iter;
    logic [55:0] div_r;
    logic [56:0] div_q;
    logic [52:0] div_b;
    logic [59:0] sq_rem;
    logic [57:0] sq_root;
    logic [111:0] sq_rad;
    localparam int DIV_N_D = (DIVB == 2) ? 29 : 57 / DIVB, DIV_N_S = (DIVB == 2) ? 14 : 27 / DIVB;
    wire q_last = q_sqrt ? (q_iter == (q_sgl ? 6'(28 / SQB - 1) : 6'(56 / SQB - 1)))
                         : (q_iter == (q_sgl ? 6'(DIV_N_S - 1) : 6'(DIV_N_D - 1)));
    logic [54:0] div_b3;                // 3b, added up during the first (one-bit) step
    logic [57:0] r4_d1, r4_d2, r4_d3;
    logic [55:0] r4_r;
    logic [1:0]  r4_q;
    always_comb begin
        r4_d1 = {div_r, 2'b00} - {4'd0, div_b, 1'b0};      // 4r - 2b
        r4_d2 = {div_r, 2'b00} - {3'd0, div_b, 2'b00};     // 4r - 4b
        r4_d3 = {div_r, 2'b00} - {2'd0, div_b3, 1'b0};     // 4r - 6b
        if (!r4_d3[57])      begin r4_q = 2'd3; r4_r = r4_d3[55:0]; end
        else if (!r4_d2[57]) begin r4_q = 2'd2; r4_r = r4_d2[55:0]; end
        else if (!r4_d1[57]) begin r4_q = 2'd1; r4_r = r4_d1[55:0]; end
        else                 begin r4_q = 2'd0; r4_r = {div_r[53:0], 2'b00}; end
    end
    // started on the operation alone (the early decisions come late in the cycle);
    // dropped the cycle after if the operation needed no recurrence
    wire q_start = e_live && e_slow_op && !e_started && qst == Q_IDLE;
    assign slow_xfer = qst == Q_FIN && x3_slow;        // the instruction waits in W: hand the result over

    logic [55:0] dv_r [4];
    logic [56:0] dv_q [4];
    logic [59:0] sr_rem [3];
    logic [57:0] sr_root [3];
    always_comb begin
        dv_r[0] = div_r; dv_q[0] = div_q;
        for (int k = 0; k < 3; k++) begin                   // the steps past DIVB / SQB are not used
            logic [56:0] t;
            t = {dv_r[k], 1'b0} - {3'd0, div_b, 1'b0};
            if (!t[56]) begin dv_r[k+1] = t[55:0];              dv_q[k+1] = {dv_q[k][55:0], 1'b1}; end
            else        begin dv_r[k+1] = {dv_r[k][54:0], 1'b0}; dv_q[k+1] = {dv_q[k][55:0], 1'b0}; end
        end
        sr_rem[0] = sq_rem; sr_root[0] = sq_root;
        for (int k = 0; k < 2; k++) begin
            logic [59:0] rem2, trial;
            rem2 = {sr_rem[k][57:0], (k == 0) ? sq_rad[111:110] : sq_rad[109:108]};
            trial = {2'b00, sr_root[k][55:0], 2'b01};
            if (rem2 >= trial) begin sr_rem[k+1] = rem2 - trial; sr_root[k+1] = {sr_root[k][56:0], 1'b1}; end
            else               begin sr_rem[k+1] = rem2;         sr_root[k+1] = {sr_root[k][56:0], 1'b0}; end
        end
    end

    // ================================================================ M: normalise
    // The sum needs the leading-zero count and the shifter; the product and the
    // finished recurrence at most one place. Everything below the round bit of
    // the result's format becomes one sticky bit here: for the sum from its
    // trailing-zero count (beside the shift), for the others from their bits.
    function automatic logic [5:0] tzc58(input logic [57:0] v);     // trailing zeroes, 58 for zero
        logic [57:0] rv;
        logic [6:0]  c;
        for (int i = 0; i < 58; i++) rv[i] = v[57 - i];
        c = lzc64({rv, 6'b111111});
        return c[5:0];
    endfunction
    function automatic logic low_d(input logic [57:0] nv); return |nv[2:0];  endfunction   // double: below guard, round
    function automatic logic low_s(input logic [57:0] nv); return |nv[31:0]; endfunction   // single
    logic [57:0] s2_nv;
    logic [5:0]  s2_lz;
    logic        s2_st, s2_rs, s2_osgl;
    logic signed [13:0] s2_re, s2_ne;
    always_comb begin
        logic [57:0] vq, vm, nq, nm;
        logic [5:0]  lza, tza;
        logic [6:0]  zsum;
        logic        stq;
        // the sum (and what else comes through the adder: FCNVFF, FCNVXF)
        lza = lzc58(x2_v); tza = tzc58(x2_v);
        zsum = {1'b0, lza} + {1'b0, tza};               // the normalised value's trailing zeroes (116 for zero)
        // the product: [1, 4)
        vm = prod[105:48];
        nm = vm[57] ? vm : {vm[56:0], 1'b0};
        // the recurrence: a quotient in (0.5, 2), a root in [1, 2)
        if (q_sqrt) begin vq = q_sgl ? {sq_root[27:0], 30'd0} : {sq_root[55:0], 2'b00}; stq = |sq_rem; end
        else        begin vq = q_sgl ? {div_q[26:0], 31'd0} : {div_q, 1'b0};            stq = |div_r;  end
        nq = vq[57] ? vq : {vq[56:0], 1'b0};
        if (slow_xfer) begin
            s2_nv = nq; s2_lz = {5'd0, ~vq[57]};
            s2_re = q_re; s2_rs = q_rs; s2_osgl = q_sgl;
            s2_st = stq | (q_sgl ? low_s(nq) : low_d(nq));
        end else if (x2_op == F_MPY) begin
            s2_nv = nm; s2_lz = {5'd0, ~vm[57]};
            s2_re = x2_re; s2_rs = x2_rs; s2_osgl = x2_osgl;
            s2_st = (|prod[47:0]) | (x2_osgl ? low_s(nm) : low_d(nm));
        end else begin
            s2_nv = x2_v << lza; s2_lz = lza;
            s2_re = x2_re; s2_rs = x2_rs; s2_osgl = x2_osgl;
            s2_st = x2_sticky | (x2_osgl ? (zsum < 7'd32) : (zsum < 7'd3));
        end
        s2_ne = s2_re - 14'(signed'({8'd0, s2_lz}));
    end
    // FCNVFX / FCNVFXT: round, negate, range
    logic [63:0] s2_int;
    logic        s2_iovf;
    always_comb begin
        logic [63:0] ival;
        logic up;
        if (x2_op == F_CNVFXT) up = 1'b0;
        else case (rm)
            2'd0: up = x2_cg & (x2_cr | x2_cs | x2_x[0]);
            2'd1: up = 1'b0;
            2'd2: up = ~x2_as & (x2_cg | x2_cr | x2_cs);
            default: up = x2_as & (x2_cg | x2_cr | x2_cs);
        endcase
        ival = x2_x + {63'd0, up};
        s2_iovf = 1'b0;
        if (x2_dsgl) begin                                 // 32-bit integer
            if (x2_as ? (ival > 64'h8000_0000) : (ival > 64'h7FFF_FFFF)) s2_iovf = 1'b1;
            // ERS s.7.3 **: FCNVFX(T),dbl,sgl of -2^31 raises the unimplemented
            // exception "even though this is not strictly necessary"
            if (!x2_sgl && x2_as && ival == 64'h8000_0000) s2_iovf = 1'b1;
            s2_int = {(x2_as ? -ival[31:0] : ival[31:0]), 32'd0};
        end else begin
            if (x2_as ? (ival > 64'h8000_0000_0000_0000) : ival[63]) s2_iovf = 1'b1;
            s2_int = x2_as ? -ival : ival;
        end
    end
    // the results that bypass the rounding: the early decisions, the integer
    // of FCNVFX, XMPYU's product, an exact cancellation
    logic        s2_byp, s2_bwe, s2_bun;
    logic [4:0]  s2_bfl;
    logic [63:0] s2_x;
    always_comb begin
        logic inx;
        inx = x2_cg | x2_cr | x2_cs;
        s2_byp = 1'b1; s2_bwe = 1'b0; s2_bun = 1'b0; s2_bfl = 5'd0; s2_x = x2_x;
        if (x2_op == F_XMPYU) begin s2_x = prod[63:0]; s2_bwe = 1'b1; end
        else if (x2_early) begin s2_bwe = x2_ewe; s2_bun = x2_eun; s2_bfl[4] = x2_efv; s2_bfl[3] = x2_efz; end
        else if (x2_op == F_CNVFX || x2_op == F_CNVFXT) begin
            if (s2_iovf) s2_bun = 1'b1;
            else if (inx && en_i) s2_bun = 1'b1;
            else begin s2_x = s2_int; s2_bwe = 1'b1; s2_bfl[0] = inx; end
        end else if ((x2_op == F_ADD || x2_op == F_SUB) && x2_v == 58'd0 && !x2_sticky) begin
            // an exact cancellation gives +0 (or -0 in round-to-minus-infinity)
            s2_x = {rm == 2'd3, 63'd0}; s2_bwe = 1'b1;
        end else s2_byp = 1'b0;
        if (x2_dup) s2_x[31:0] = s2_x[63:32];
    end

    // ================================================================ W: round, the PCX-L rules
    logic [52:0] kept;
    logic        g, r, inexact, round_up, lsb, carry, ovf, tiny;
    logic [52:0] mant;
    logic signed [13:0] e_final;
    logic [63:0] pk, pk_inf, pk_max, pk_zero;
    always_comb begin
        logic [52:0] kept1;
        logic [31:0] hi;
        logic to_inf;
        // x3_nv[57] is the leading one; keep 53 (double) or 24 (single) bits
        if (x3_osgl) begin kept = {29'd0, x3_nv[57:34]}; g = x3_nv[33]; r = x3_nv[32]; lsb = x3_nv[34]; end
        else         begin kept = x3_nv[57:5];           g = x3_nv[4];  r = x3_nv[3];  lsb = x3_nv[5];  end
        inexact = g | r | x3_st;
        case (rm)
            2'd0: round_up = g & (r | x3_st | lsb);
            2'd1: round_up = 1'b0;
            2'd2: round_up = ~x3_rs & (g | r | x3_st);
            default: round_up = x3_rs & (g | r | x3_st);
        endcase
        // the increment beside the decision; a carry out of the significand (all
        // ones rounded up) leaves 1.0 with the exponent one up
        kept1 = kept + 53'd1;
        carry = round_up && (x3_osgl ? (&x3_nv[57:34]) : (&x3_nv[57:5]));
        mant = round_up ? kept1 : kept;
        e_final = carry ? x3_ne + 14'sd1 : x3_ne;
        ovf  = carry ? x3_ovf1 : x3_ovf0;
        tiny = carry ? x3_tiny1 : x3_tiny0;
        // packed; a single in both halves
        hi = {x3_rs, 8'(e_final + 14'sd127), carry ? 23'd0 : mant[22:0]};
        if (x3_osgl) pk = {hi, hi};
        else         pk = {x3_rs, 11'(e_final + 14'sd1023), carry ? 52'd0 : mant[51:0]};
        pk_inf  = x3_osgl ? {2{x3_rs, 8'hFF, 23'd0}}       : {x3_rs, 11'h7FF, 52'd0};
        pk_max  = x3_osgl ? {2{x3_rs, 8'hFE, {23{1'b1}}}}  : {x3_rs, 11'h7FE, {52{1'b1}}};
        pk_zero = x3_osgl ? {2{x3_rs, 31'd0}}              : {x3_rs, 63'd0};
        case (rm)
            2'd0: to_inf = 1'b1;
            2'd1: to_inf = 1'b0;
            2'd2: to_inf = ~x3_rs;
            default: to_inf = x3_rs;
        endcase
        result = 64'd0; result_we = 1'b0; flags = 5'd0; unimpl = 1'b0; cmp_c = x3_bcmp;
        if (x3_byp) begin
            result = x3_x; result_we = x3_bwe; unimpl = x3_bun; flags = x3_bfl;
        end else if (ovf) begin
            if (en_o) unimpl = 1'b1;
            else begin
                // overflow with the trap disabled: the IEEE default result, O and I flags
                result = to_inf ? pk_inf : pk_max;
                result_we = 1'b1; flags[2] = 1'b1; flags[0] = 1'b1;
            end
        end else if (tiny) begin
            if (!dmode) unimpl = 1'b1;                     // tiny result: the handler makes the denormal
            else if (en_i) unimpl = 1'b1;                  // flushed to zero with inexact enabled
            else begin result = pk_zero; result_we = 1'b1; flags[0] = 1'b1; end
        end else if (inexact && en_i) unimpl = 1'b1;
        else begin result = pk; result_we = 1'b1; flags[0] = inexact; end
    end

    // ================================================================ sequential
    always_ff @(posedge clk) begin
        if (adv) begin
            // ---- D -> first stage
            x1_op <= d_op; x1_sgl <= d_sgl; x1_dsgl <= d_dsgl; x1_cond <= d_cond;
            x1_a <= d_a; x1_b <= d_b;
            x1_ca <= classify(d_a, d_sgl, dmode); x1_cb <= classify(d_b, d_sgl, dmode);
            x1_age <= d_age; x1_aeq <= d_aeq; x1_sh <= d_sh; x1_lsh <= d_lsh;
            x1_ege52 <= d_ege52; x1_egt63 <= d_egt63;
            if (d_op == F_XMPYU) begin mul_a <= {21'd0, d_a[63:32]}; mul_b <= {21'd0, d_b[63:32]}; end
            else begin
                mul_a <= d_sgl ? {1'b1, d_a[54:32], 29'd0} : {1'b1, d_a[51:0]};
                mul_b <= d_sgl ? {1'b1, d_b[54:32], 29'd0} : {1'b1, d_b[51:0]};
            end
            // ---- first -> second
            x2_op <= x1_op; x2_sgl <= x1_sgl; x2_dsgl <= x1_dsgl; x2_osgl <= e_osgl; x2_as <= sa;
            x2_v <= s1_sum; x2_sticky <= s1_sticky; x2_rs <= s1_rs; x2_re <= s1_re;
            x2_x <= s1_x; x2_cg <= s1_cg; x2_cr <= s1_cr; x2_cs <= s1_cs;
            x2_early <= early_done; x2_eun <= early_unimpl; x2_ewe <= early_we; x2_efv <= early_fv;
            x2_efz <= early_fz; x2_ecmp <= early_cmp;
            // the destination is a single (so the result goes to both halves): XMPYU's is 64 bits
            x2_dup <= (x1_op == F_XMPYU) ? 1'b0 :
                      (x1_op == F_CNVFF || x1_op == F_CNVXF || x1_op == F_CNVFX || x1_op == F_CNVFXT) ? x1_dsgl : x1_sgl;
            prod <= mul_a * mul_b;
            // ---- second -> third: what bypasses the rounding
            x3_byp <= s2_byp; x3_bwe <= s2_bwe; x3_bun <= s2_bun; x3_bfl <= s2_bfl; x3_x <= s2_x;
            x3_bcmp <= x2_ecmp;
        end
        if (adv || slow_xfer) begin
            // ---- second -> third: the normalised significand (or the recurrence's)
            x3_nv <= s2_nv; x3_st <= s2_st; x3_ne <= s2_ne; x3_rs <= s2_rs; x3_osgl <= s2_osgl;
            // the range of the result, for the exponent as it is and one up (a rounding carry)
            x3_ovf0  <= s2_osgl ? (s2_ne > 14'sd127)   : (s2_ne > 14'sd1023);
            x3_ovf1  <= s2_osgl ? (s2_ne >= 14'sd127)  : (s2_ne >= 14'sd1023);
            x3_tiny0 <= s2_osgl ? (s2_ne < -14'sd126)  : (s2_ne < -14'sd1022);
            x3_tiny1 <= s2_osgl ? (s2_ne < -14'sd127)  : (s2_ne < -14'sd1023);
        end
        if (!rst_n || kill) begin
            x2_slow <= 1'b0; x3_slow <= 1'b0; e_started <= 1'b0; qst <= Q_IDLE; q_abort <= 1'b0;
        end else begin
            q_abort <= q_start && !e_slow;                      // decided early after all
            if (adv) begin
                x2_slow <= e_live && e_slow; x3_slow <= x2_slow; e_started <= 1'b0;
            end else if (q_start) e_started <= 1'b1;
            case (qst)
                Q_IDLE: if (q_start) begin
                    q_sqrt <= x1_op == F_SQRT; q_sgl <= x1_sgl; q_iter <= 6'd0; q_re <= s1_re; q_rs <= s1_rs;
                    div_r <= {3'd0, mul_a}; div_b <= mul_b; div_q <= 57'd0;
                    // radicand m = sig (E even) or sig << 1 (E odd), times 2^58
                    sq_rad <= ea[0] ? {mul_a, 1'b0, 58'd0} : {1'b0, mul_a, 58'd0};
                    sq_rem <= 60'd0; sq_root <= 58'd0;
                    qst <= Q_RUN;
                end
                Q_RUN: if (q_abort) qst <= Q_IDLE;
                else begin
                    if (q_sqrt) begin
                        sq_rem <= sr_rem[SQB]; sq_root <= sr_root[SQB]; sq_rad <= sq_rad << (2 * SQB);
                    end else begin
                        if (DIVB == 2) begin
                            div_b3 <= {1'b0, div_b, 1'b0} + {2'b00, div_b};
                            if (q_iter == 6'd0) begin div_r <= dv_r[1]; div_q <= dv_q[1]; end
                            else                begin div_r <= r4_r;    div_q <= {div_q[54:0], r4_q}; end
                        end else begin
                            div_r <= dv_r[DIVB]; div_q <= dv_q[DIVB];
                        end
                    end
                    q_iter <= q_iter + 6'd1;
                    if (q_last) qst <= Q_FIN;
                end
                Q_FIN: if (x3_slow) qst <= Q_DONE;              // handed to the W registers (slow_xfer)
                default: if (adv && x3_slow) qst <= Q_IDLE;     // Q_DONE: its instruction leaves W
            endcase
        end
    end
endmodule
