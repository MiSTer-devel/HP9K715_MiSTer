// PA-7100LC floating-point unit: the FP register file, the status and
// exception registers, the decode of the FP operation word and the sequencing
// of the arithmetic engine (pa_fpu) -- docs/design/cpu.md 3.5, 4.3.12, 6.11;
// ERS ch. 7; ACD ch. 6.
//
// An operation (FU_FP) runs beside the core's pipeline (cpu-timing.md 10):
// its operands are read from the register file while it is still in F2 and
// registered as it enters D (a register written since is taken from the last
// write), the datapath (pa_fpdp) classifies and compares them in D and works
// on while the instruction is in E, M and W, and the result, the
// flags and the compare bit are written when the instruction commits in W --
// in program order, so an instruction that traps or is killed leaves nothing
// behind. An FP load writes its target when it commits; an FP store reads
// its source in W. What the core's D stage must wait for is `d_wait`: a
// source register that an operation or a load ahead (in E, M or W) has yet
// to write, FTEST behind an operation, a second divide / square root, and the
// pair instructions' two steps. FDIV and FSQRT hold W from a flip-flop
// (`w_hold`) until their recurrence is done.
//
// The unimplemented exception sets FPSR.T at the commit; the core takes
// trap 14 on the next FP instruction, in W as well as in E, since that
// instruction may already be past E (pa_core `w_t_late`).
//
// The register file fr4..fr31 is MLAB LUT-RAM with asynchronous reads, one
// copy per read port (two operands, the store source), every write going to
// all three (the pa_regfile scheme; one array with three read ports would
// not infer a memory and cost 1.8k flip-flops plus the read muxes, fit 10).
//
// FPSR (fr0L) in Verilog bits: flags V Z O U I = [31:27], C = [26], the
// compare queue PA 10:20 = [21:11] (also the model/revision written by
// COPR,0,0), RM = [10:9], T = [6], D = [5], enables V Z O U I = [4:0].
// fr0R, fr1L..fr3R are exception registers 1..7; the hardware writes only
// exception register 2 (fr1L) = {code, instruction bits 6:31} on the
// unimplemented exception, which also sets T (sec. 3.5).
module pa_fpunit
    import pa_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        adv,            // the pipeline moves (E -> M -> W)
    input  logic        kill,           // W redirect: everything younger is dropped
    // D: the instruction there, and what it must wait for
    input  logic [31:0] d_insn,
    input  logic        d_fpop,         // a valid FP operation (FU_FP)
    input  logic        d_ftest,
    input  logic        d_fp,           // any FP instruction (operation, load, store)
    input  logic        d_adv,          // D's instruction moves to E this cycle (and F2's into D)
    input  logic [31:0] f2_insn,        // the instruction about to enter D
    output logic        d_wait,
    // E
    input  logic        e_fpop,         // a valid, non-nullified FP operation is in E
    input  logic        e_fpld,         // a valid, non-nullified FP load is in E
    input  logic [31:0] e_insn,
    // W
    input  logic        issue,          // the FP operation in W commits this cycle
    input  logic [31:0] insn,           // the instruction in W (operations, loads, stores)
    input  logic [31:0] m_insn,         // the instruction in M (an FP store's source is read there)
    input  logic        m_st_dbl,       // ... and it is a doubleword store (the core's decoded size)
    input  logic        ld_we,          // an FP load commits: write insn's target
    input  logic [63:0] ld_data,        // the doubleword (FLDD) or the word in [31:0] (FLDW)
    input  logic        st_commit,      // an FP store commits (FSTD fr0 clears T)
    output logic [63:0] st_data,        // the store source of the instruction in W (a flip-flop): a doubleword, or the word in [31:0]
    output logic        w_hold,         // the FP operation in W is not finished (a flip-flop)
    output logic        t_set,          // FPSR.T is being set by the operation committing now
    output logic        fpsr_t,         // FPSR.T: the next FP instruction traps 14
    output logic [31:0] fpsr_o
);
    // ---------------------------------------------------------------- registers
    logic [31:0] fpsr;
    logic [31:0] xr [7];                // exception registers 1..7 = fr0R, fr1L, fr1R, fr2L, fr2R, fr3L, fr3R
    // fr4..fr31 (entries 0..3 unused): left and right halves, one copy per read port
    (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] frl_a [32], frl_b [32], frl_c [32], frl_d [32], frl_s [32];
    (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] frr_a [32], frr_b [32], frr_c [32], frr_d [32], frr_s [32];
    assign fpsr_t = fpsr[6];

    function automatic logic [63:0] rd_ff(input logic [4:0] r);
        case (r[1:0])
            2'd0: rd_ff = {fpsr, xr[0]};
            2'd1: rd_ff = {xr[1], xr[2]};
            2'd2: rd_ff = {xr[3], xr[4]};
            default: rd_ff = {xr[5], xr[6]};
        endcase
    endfunction
    // five read ports: the operands of the instruction in F2 (a, b), the second
    // operation of a pair instruction in D (c, d), the store source (s). As an operand
    // of a non-load/store instruction fr0 "encodes a floating-point +0 or a
    // fixed-point 0" (ACD p.6-4, 6-9): HP's compilers zero registers with
    // FCPY fr0 and compare against fr0; only FSTD/FSTW fr0 read the status.
    // Found 2026-10-09: awk's `fcmp,dbl,< fr4,fr0` compared against the
    // status word, so its truth tests went wrong and its loops never ended.
    // fr1-fr3 as operands are undefined (ACD p.6-5); they read as themselves.
    logic [4:0]  rd_a_n, rd_b_n, rd_c_n, rd_d_n, rd_s_n;
    logic [63:0] rd_a, rd_b, rd_c, rd_d, rd_s;
    assign rd_a = (rd_a_n == 5'd0) ? 64'd0 : (rd_a_n < 5'd4) ? rd_ff(rd_a_n) : {frl_a[rd_a_n], frr_a[rd_a_n]};
    assign rd_b = (rd_b_n == 5'd0) ? 64'd0 : (rd_b_n < 5'd4) ? rd_ff(rd_b_n) : {frl_b[rd_b_n], frr_b[rd_b_n]};
    assign rd_c = (rd_c_n == 5'd0) ? 64'd0 : (rd_c_n < 5'd4) ? rd_ff(rd_c_n) : {frl_c[rd_c_n], frr_c[rd_c_n]};
    assign rd_d = (rd_d_n == 5'd0) ? 64'd0 : (rd_d_n < 5'd4) ? rd_ff(rd_d_n) : {frl_d[rd_d_n], frr_d[rd_d_n]};
    assign rd_s = (rd_s_n < 5'd4) ? rd_ff(rd_s_n) : {frl_s[rd_s_n], frr_s[rd_s_n]};
    // a single operand from the selected half, left-justified for the datapath
    // (the other half's bits must not reach it: its compares take all 63 bits)
    function automatic logic [63:0] opnd(input logic [63:0] r, input logic single, input logic half);
        return !single ? r : half ? {r[31:0], 32'd0} : {r[63:32], 32'd0};
    endfunction

    // ---------------------------------------------------------------- decode of the operation word (F2 -> D)
    // K_MULH: the multiply of a pair instruction, a stage ahead of it; K_NONE: nothing of the FPU's
    typedef enum logic [2:0] { K_UNIMPL, K_OP, K_FMA, K_FTEST, K_FID, K_MULH, K_NONE } kind_e;
    kind_e      d_kind;
    fpu_op_e    d_op;
    logic       d_sgl, d_dsgl, d_sdest;           // operand format, convert destination, single result
    logic [4:0] d_cond;
    logic [4:0] d_r1, d_r2, d_t;                  // register numbers
    logic       d_h1, d_h2, d_ht;                 // halves (0E singles)
    logic [4:0] d_tm, d_ra, d_ta;                 // FMPYADD/SUB/CFXT
    logic       d_htm, d_hra, d_hta, d_fsub, d_cfxt;
    logic [5:0] d_code;
    // the 5-bit single specifier of 06/26: 0-15 -> fr(16+s)L, 16-31 -> fr(s)R
    function automatic logic [5:0] fma_spec(input logic [4:0] s, input logic single);   // {half, reg}
        return !single ? {1'b0, s} : s[4] ? {1'b1, s} : {1'b0, 1'b1, s[3:0]};
    endfunction
    // The word is decoded while the instruction is still in F2 (the registers
    // are below), and the register file is read from F2's word too.
    kind_e      n_kind;
    fpu_op_e    n_op;
    logic       n_sgl, n_dsgl, n_sdest;
    logic [4:0] n_cond, n_r1, n_r2, n_t, n_tm, n_ra, n_ta;
    logic       n_h1, n_h2, n_ht, n_htm, n_hra, n_hta, n_fsub, n_cfxt;
    logic [5:0] n_code;
    always_comb begin
        logic [5:0] maj;
        logic [1:0] cls, fmt, sf, df;
        logic [2:0] sub;
        logic       is0e;
        maj = f2_insn[31:26]; cls = f2_insn[10:9]; is0e = maj == 6'h0E;
        n_kind = K_UNIMPL; n_op = F_RESERVED; n_sgl = 1'b0; n_dsgl = 1'b0; n_sdest = 1'b0; n_cond = f2_insn[4:0];
        n_r1 = f2_insn[25:21]; n_r2 = f2_insn[20:16]; n_t = f2_insn[4:0];
        n_h1 = is0e & f2_insn[7]; n_h2 = is0e & f2_insn[12]; n_ht = is0e & f2_insn[6];
        n_tm = 5'd0; n_ra = 5'd0; n_ta = 5'd0; n_htm = 1'b0; n_hra = 1'b0; n_hta = 1'b0;
        n_fsub = 1'b0; n_cfxt = 1'b0;
        fmt = is0e ? {1'b0, f2_insn[11]} : f2_insn[12:11];
        sub = f2_insn[15:13];
        sf = f2_insn[12:11]; df = f2_insn[14:13];
        case (maj)
            6'h0C, 6'h0E: begin
                n_code = is0e ? 6'b001011 : 6'b001001;
                n_sgl = fmt == 2'd0; n_sdest = n_sgl;
                case (cls)
                    2'd0: begin                                             // class 0
                        if (fmt[1]) n_kind = K_UNIMPL;                      // quad / undefined format
                        else case (sub)
                            3'd0: if (f2_insn == 32'h3000_0000) n_kind = K_FID;
                            3'd2: begin n_kind = K_OP; n_op = F_CPY; end
                            3'd3: begin n_kind = K_OP; n_op = F_ABS; end
                            3'd4: begin n_kind = K_OP; n_op = F_SQRT; end
                            3'd5: begin n_kind = K_OP; n_op = F_RND; end    // the engine reports it unimplemented
                            default: ;                                      // 1, 6, 7: PA 2.0 FNEG/FNEGABS, reserved
                        endcase
                    end
                    2'd1: begin                                             // class 1: conversions
                        n_sgl = sf == 2'd0; n_dsgl = df == 2'd0; n_sdest = n_dsgl;
                        if (sf[1] || df[1] || f2_insn[17]) n_kind = K_UNIMPL;  // quad / undefined, PA 2.0 unsigned
                        else case (f2_insn[16:15])
                            2'd0: begin                                     // FCNVFF: sgl,sgl = FCPY sgl; dbl,dbl = FABS dbl (ERS s.7.2.5)
                                n_kind = K_OP;
                                n_op = (sf == df) ? (sf[0] ? F_ABS : F_CPY) : F_CNVFF;
                            end
                            2'd1: begin n_kind = K_OP; n_op = F_CNVXF; end
                            2'd2: begin n_kind = K_OP; n_op = F_CNVFX; end
                            default: begin n_kind = K_OP; n_op = F_CNVFXT; end
                        endcase
                    end
                    2'd2: begin                                             // class 2: FCMP, FTEST
                        if (f2_insn[5]) begin                                  // the N bit: FTEST (0C, sub 1), else undefined
                            if (!is0e && sub == 3'd1 && ftest_valid(f2_insn[4:0])) n_kind = K_FTEST;
                        end else if (!fmt[1]) begin n_kind = K_OP; n_op = F_CMP; end
                    end
                    default: begin                                          // class 3: arithmetic, XMPYU
                        if (is0e && f2_insn[8]) begin                          // fixed-point: XMPYU only
                            if (sub == 3'd2 && !f2_insn[11]) begin n_kind = K_OP; n_op = F_XMPYU; n_sgl = 1'b1; n_sdest = 1'b0; end
                        end else if (!fmt[1]) case (sub)
                            3'd0: begin n_kind = K_OP; n_op = F_ADD; end
                            3'd1: begin n_kind = K_OP; n_op = F_SUB; end
                            3'd2: begin n_kind = K_OP; n_op = F_MPY; end
                            3'd3: begin n_kind = K_OP; n_op = F_DIV; end
                            default: ;
                        endcase
                    end
                endcase
            end
            6'h06, 6'h26: begin                                             // FMPYADD / FMPYSUB / FMPYCFXT
                logic single;
                single = f2_insn[5];
                n_code = (maj == 6'h26) ? 6'b100011 : 6'b000011;
                n_kind = K_FMA; n_fsub = maj == 6'h26; n_sgl = single; n_sdest = single;
                {n_h1, n_r1} = fma_spec(f2_insn[25:21], single);              // rm1
                {n_h2, n_r2} = fma_spec(f2_insn[20:16], single);              // rm2
                {n_hta, n_ta} = fma_spec(f2_insn[15:11], single);
                {n_hra, n_ra} = fma_spec(f2_insn[10:6], single);
                {n_htm, n_tm} = fma_spec(f2_insn[4:0], single);
                n_cfxt = f2_insn[10:6] == 5'd0;
            end
            default: n_code = 6'd0;
        endcase
    end

    // the pair instructions' second operation: ta + ra, ta - ra (FMPYSUB), or
    // FCNVFXT(ta) to a 32-bit integer (FMPYCFXT) -- its operands and operation
    logic [4:0] d_a2, d_b2;
    logic       d_ha2, d_hb2, d_dsgl2;
    fpu_op_e    d_op2;
    // Nothing here waits for the pipeline: `pf` takes F2's decode every cycle,
    // an instruction's first cycle in D (`d_first`, a flip-flop) uses it and
    // every later cycle the copy kept in `pd`. So the only thing the unit needs
    // from the pipeline's late "D moves" is that one flag.
    typedef struct packed {
        kind_e      kind;
        fpu_op_e    op, op2;
        logic       sgl, dsgl, sdest, dsgl2;
        logic [4:0] cond, r1, r2, t, tm, ra, ta, a2, b2;
        logic       h1, h2, ht, htm, hra, hta, ha2, hb2, fsub, cfxt;
        logic [5:0] code;
    } dec_t;
    dec_t       n_dec, pf, pd, dd;
    logic       d_first;
    always_comb begin
        n_dec.kind = n_kind; n_dec.op = (n_kind == K_FMA) ? F_MPY : n_op;       // a pair's first operation is its multiply
        n_dec.sgl = n_sgl; n_dec.dsgl = n_dsgl; n_dec.sdest = n_sdest; n_dec.cond = n_cond;
        n_dec.r1 = n_r1; n_dec.r2 = n_r2; n_dec.t = n_t; n_dec.h1 = n_h1; n_dec.h2 = n_h2; n_dec.ht = n_ht;
        n_dec.tm = n_tm; n_dec.ra = n_ra; n_dec.ta = n_ta; n_dec.htm = n_htm; n_dec.hra = n_hra; n_dec.hta = n_hta;
        n_dec.fsub = n_fsub; n_dec.cfxt = n_cfxt; n_dec.code = n_code;
        n_dec.a2 = (n_cfxt || n_fsub) ? n_ta : n_ra;  n_dec.ha2 = (n_cfxt || n_fsub) ? n_hta : n_hra;
        n_dec.b2 = n_fsub ? n_ra : n_ta;              n_dec.hb2 = n_fsub ? n_hra : n_hta;
        n_dec.op2 = n_cfxt ? F_CNVFXT : n_fsub ? F_SUB : F_ADD;
        n_dec.dsgl2 = n_cfxt ? 1'b1 : n_sgl;
    end
    assign dd = d_first ? pf : pd;
    always_ff @(posedge clk) begin pf <= n_dec; pd <= dd; d_first <= rst_n && d_adv; end
    assign d_kind = dd.kind; assign d_op = dd.op; assign d_op2 = dd.op2;
    assign d_sgl = dd.sgl; assign d_dsgl = dd.dsgl; assign d_sdest = dd.sdest; assign d_dsgl2 = dd.dsgl2;
    assign d_cond = dd.cond; assign d_r1 = dd.r1; assign d_r2 = dd.r2; assign d_t = dd.t;
    assign d_tm = dd.tm; assign d_ra = dd.ra; assign d_ta = dd.ta; assign d_a2 = dd.a2; assign d_b2 = dd.b2;
    assign d_h1 = dd.h1; assign d_h2 = dd.h2; assign d_ht = dd.ht; assign d_htm = dd.htm; assign d_hra = dd.hra;
    assign d_hta = dd.hta; assign d_ha2 = dd.ha2; assign d_hb2 = dd.hb2; assign d_fsub = dd.fsub; assign d_cfxt = dd.cfxt;
    assign d_code = dd.code;

    // ---------------------------------------------------------------- store source
    // read while the store is in M, registered as it enters W (below, with the
    // result that is written at that very edge)
    assign rd_s_n = m_insn[4:0];

    // ---------------------------------------------------------------- D: the operation, its operands
    // A pair instruction (FMPYADD / FMPYSUB / FMPYCFXT) is two operations: its
    // multiply enters the datapath first, while the instruction waits one cycle
    // in D (`d_half` then says it has), and runs one stage ahead of it; the
    // instruction itself carries the second operation (decoded above).
    wire         d_isfma = d_kind == K_FMA;
    logic        d_half;
    wire         d_ha = d_half ? d_ha2 : d_h1, d_hb = d_half ? d_hb2 : d_h2;
    fpu_op_e     g_op;
    assign g_op = d_half ? d_op2 : d_op;
    wire         g_dsgl = d_half ? d_dsgl2 : d_dsgl;

    // The operands. They are read while the instruction is in F2 (ports a, b:
    // the addresses straight from F2's word) into `xf`, every cycle; a pair's
    // second operands are read while it is in D (ports c, d) into `xc`, every
    // cycle. The instruction uses `xf` in its first cycle in D, `xc` in the
    // first cycle of a pair's second operation, and otherwise `x0`, the copy
    // kept of what it used the cycle before. A result goes into the register
    // file a cycle after its commit (from `lw`, below: the file is written from
    // flip-flops, not by the pipeline's commit), so a read misses the last two
    // commits: both are kept (`lw`, and `lw2` behind it) and replace an operand
    // they match, the newer last. So the register file is not
    // in D's path, nothing here is enabled by the pipeline, and the instruction
    // may leave D as soon as nothing ahead of it in E, M or W still has to write
    // one of its sources (`d_block_r` below), as before.
    assign rd_a_n = n_r1; assign rd_b_n = n_r2;
    assign rd_c_n = d_a2; assign rd_d_n = d_b2;
    logic [63:0] xf_a, xf_b, xc_a, xc_b, x0_a, x0_b;
    logic        d_hf;                  // the first cycle of a pair's second operation
    wire [4:0]   xa_n = d_half ? d_a2 : d_r1, xb_n = d_half ? d_b2 : d_r2;   // the sources (fr0 reads as zero: never replaced)
    logic        lw_v, lw_l, lw_r;      // the last commit wrote lw_n: its left / right half
    logic [4:0]  lw_n;
    logic [63:0] lw_data;
    logic        lw2_v, lw2_l, lw2_r;   // the one before it (in the file from this cycle on)
    logic [4:0]  lw2_n;
    logic [63:0] lw2_data;
    function automatic logic [63:0] patch(input logic [63:0] x, input logic [4:0] n);
        logic h2, h1;
        h2 = lw2_v && lw2_n == n && n != 5'd0;
        h1 = lw_v && lw_n == n && n != 5'd0;
        patch = x;
        if (h2 && lw2_l) patch[63:32] = lw2_data[63:32];
        if (h2 && lw2_r) patch[31:0]  = lw2_data[31:0];
        if (h1 && lw_l)  patch[63:32] = lw_data[63:32];
        if (h1 && lw_r)  patch[31:0]  = lw_data[31:0];
    endfunction
    wire [63:0]  pre_a = d_first ? xf_a : d_hf ? xc_a : x0_a;
    wire [63:0]  pre_b = d_first ? xf_b : d_hf ? xc_b : x0_b;
    wire [63:0]  cur_a = patch(pre_a, xa_n);
    wire [63:0]  cur_b = patch(pre_b, xb_n);
    // The W redirect reaches the unit a cycle late, from a flip-flop: nothing of
    // an instruction is in D, E, M or W in the cycle after a redirect, so what
    // the unit still moves then is dropped before anything could use it (and W's
    // trap logic is not in front of the unit's resets).
    logic        kill_q;
    always_ff @(posedge clk) kill_q <= rst_n && kill;
    logic [63:0] dp_result;
    logic        dp_we, dp_unimpl, dp_cmp, dp_m_slow, dp_slow_xfer, dp_m_cmp, dp_m_cun, dp_w_cun;
    logic [4:0]  dp_flags;
    pa_fpdp u_dp (
        .clk(clk), .rst_n(rst_n), .adv(adv), .kill(kill_q),
        .d_op(g_op), .d_sgl(d_sgl), .d_dsgl(g_dsgl), .d_cond(d_cond),
        .d_a(opnd(cur_a, d_sgl, d_ha)), .d_b(opnd(cur_b, d_sgl, d_hb)),
        .rm(fpsr[10:9]), .dmode(fpsr[5]), .enables(fpsr[4:0]),
        .e_live(e_fpop),
        .result(dp_result), .result_we(dp_we), .flags(dp_flags), .unimpl(dp_unimpl), .cmp_c(dp_cmp),
        .m_cmp_c(dp_m_cmp), .m_cmp_un(dp_m_cun), .w_cmp_un(dp_w_cun),
        .m_slow(dp_m_slow), .slow_xfer(dp_slow_xfer));

    // ---------------------------------------------------------------- the operations in E, M and W
    kind_e       e_kind, m_kind, w_kind;
    logic        m_live, w_live, m_isld, w_isld;
    logic [5:0]  e_code, m_code, w_code;
    logic [4:0]  e_t, m_t, w_t, e_ta, m_ta, w_ta, e_tm, m_tm, w_tm;
    logic        e_ht, m_ht, w_ht, e_hta, m_hta, w_hta, e_htm, m_htm, w_htm;
    logic        e_sdest, m_sdest, w_sdest, e_cmp, m_cmp, w_cmp, e_sgl, m_sgl, w_sgl, e_cfxt, m_cfxt, w_cfxt;
    logic        e_slow_r, e_ctl_r, e_tlow, m_tlow, w_tlow;
    logic        m_p1v, m_p2v, w_p1v, w_p2v, m_ctl, w_ctl, m_slowdec, w_slowdec;
    logic [4:0]  m_p1, w_p1;
    wire         e_fma = e_kind == K_FMA;
    // what the instruction in E will write, and what else the one in D must know
    wire         e_has_t = e_kind == K_OP && !e_cmp;
    wire         e_p1v = e_fpld || (e_fpop && (e_has_t || e_fma));
    wire [4:0]   e_p1  = e_fpld ? e_insn[4:0] : e_fma ? e_tm : e_t;
    wire         e_p2v = e_fpop && e_fma;
    wire         e_ctl = e_fpop && e_ctl_r;        // an operation that writes the status / exception registers
    wire         e_slowdec = e_fpop && e_slow_r;
    // the pair's multiply result, taken as it passes W (a stage ahead of the
    // instruction) and written the cycle after the instruction's commit
    logic [63:0] fma_res1;
    logic        fma_we1, fma_un1;
    logic [4:0]  fma_fl1;
    // ... as is the result of an operation whose target is fr0-fr3 (the status
    // and exception registers: flip-flops, kept off the datapath's result)
    logic        drain_v, dr_sgl, dr_half;
    logic [4:0]  dr_n;
    logic [63:0] dr_data;

    // ---------------------------------------------------------------- D: the interlock
    // The operands are read in the cycle the operation leaves D, so every
    // register it reads must have been written: nothing ahead in E, M or W
    // (a result lands at the end of W's cycle) that targets it.
    //
    // The verdict is a flip-flop (`d_block_r`): it stands in front of the
    // pipeline's stall and of the fetch, so it is worked out a cycle ahead, for
    // the instruction that will be in D and the stages as they will be -- three
    // cases, chosen by how the pipeline moves: (A) F2's instruction enters D
    // and D's moves on, (B) D's stays and E gets a bubble, (C) nothing moves.
    // An instruction leaving D counts as live in case A (whether it is nullified
    // is decided in the same cycle); if it was not, case B or C corrects the
    // verdict a cycle later.
    // Sources by register number (a half counts as its register), of the
    // instruction in D and of the one in F2:
    function automatic logic [20:0] srcs(input logic [31:0] i);    // {pair, s4, s3, s2, s1}
        logic fs, fss;
        fs = i[31:26] == 6'h06 || i[31:26] == 6'h26;
        fss = fs && i[5];                                           // the pair's single specifiers: fr16..fr31
        srcs = {fs, fss ? {1'b1, i[14:11]} : i[15:11], fss ? {1'b1, i[9:6]} : i[10:6],
                    fss ? {1'b1, i[19:16]} : i[20:16], fss ? {1'b1, i[24:21]} : i[25:21]};
    endfunction
    function automatic logic reads(input logic [20:0] s, input logic [4:0] t);
        reads = t == s[4:0] || t == s[9:5] || (s[20] && (t == s[14:10] || t == s[19:15]));
    endfunction
    function automatic logic slow_of(input logic [31:0] i);         // FDIV, FSQRT (0C / 0E)
        slow_of = (i[31:26] == 6'h0C || i[31:26] == 6'h0E) &&
                  ((i[10:9] == 2'd3 && i[15:13] == 3'd3 && !(i[27] && i[8])) || (i[10:9] == 2'd0 && i[15:13] == 3'd4));
    endfunction
    function automatic logic fpop_of(input logic [31:0] i);
        fpop_of = i[31:26] == 6'h0C || i[31:26] == 6'h0E || i[31:26] == 6'h06 || i[31:26] == 6'h26;
    endfunction
    function automatic logic ftest_of(input logic [31:0] i);
        ftest_of = i[31:26] == 6'h0C && i[8:6] == 3'b000 && i[10:9] == 2'b10 && i[5];
    endfunction
    wire [20:0]  d_src = srcs(d_insn), f_src = srcs(f2_insn);
    wire         d_fs = d_src[20];
    wire         d_slowdec = slow_of(d_insn);
    wire         f_fpop = fpop_of(f2_insn), f_ftest = ftest_of(f2_insn);
    wire         f_fp = f_fpop || f2_insn[31:26] == 6'h09 || f2_insn[31:26] == 6'h0B;
    // what the instruction in D will write once it is in E (taken as live)
    wire         d_ctl = (d_kind == K_OP && d_op != F_CMP && d_t < 5'd4) || (d_isfma && (d_tm < 5'd4 || d_ta < 5'd4));
    wire         d_fpld = d_fp && !d_fpop && !d_insn[9];
    wire         dq1v = d_fpld || (d_fpop && ((d_kind == K_OP && d_op != F_CMP) || d_isfma));
    wire [4:0]   dq1  = d_fpld ? d_insn[4:0] : d_isfma ? d_tm : d_t;
    wire         dq2v = d_fpop && d_isfma;
    // the write that follows W's commit by a cycle: a pair's multiply result, an fr0-fr3 target
    wire         wdv = w_kind == K_FMA || (w_kind == K_OP && w_tlow);
    wire [4:0]   wd  = (w_kind == K_FMA) ? w_tm : w_t;
    wire         e_fid = e_fpop && e_kind == K_FID, m_fid = m_kind == K_FID, w_fid = w_kind == K_FID;
    // the stages behind E as they are (E, M) or as they will be (W's late write)
    function automatic logic haz_em(input logic [20:0] s);
        haz_em = (e_p1v && reads(s, e_p1)) || (e_p2v && reads(s, e_ta)) ||
                 (m_p1v && reads(s, m_p1)) || (m_p2v && reads(s, m_ta));
    endfunction
    wire blockA = (f_fpop && !f_ftest && (haz_em(f_src) || (dq1v && reads(f_src, dq1)) || (dq2v && reads(f_src, d_ta)) ||
                                          (wdv && reads(f_src, wd)))) ||
                  (f_fpop && slow_of(f2_insn) && ((d_fpop && d_slowdec) || e_slowdec || m_slowdec)) ||
                  (f_ftest && ((d_fpop && d_kind == K_FID) || e_fid || m_fid)) ||
                  (f_fp && d_fpop && d_isfma) ||
                  (f_fp && ((d_fpop && d_ctl) || e_ctl || m_ctl));
    wire blockB = (d_fpop && !d_ftest && (haz_em(d_src) || (wdv && reads(d_src, wd)))) ||
                  (d_fpop && d_slowdec && (e_slowdec || m_slowdec)) ||                  // one recurrence at a time
                  (d_ftest && (e_fid || m_fid)) ||       // FTEST reads the status in E: compares are forwarded, COPR 0,0 is not
                  (d_fp && (e_ctl || m_ctl));
    wire blockC = (d_fpop && !d_ftest && (haz_em(d_src) || (w_p1v && reads(d_src, w_p1)) || (w_p2v && reads(d_src, w_ta)) ||
                                          (drain_v && reads(d_src, dr_n)))) ||
                  (d_fpop && d_slowdec && (e_slowdec || m_slowdec || w_slowdec)) ||
                  (d_ftest && (e_fid || m_fid || w_fid)) ||
                  (d_fp && e_fpop && e_fma) ||           // nothing of the FPU's directly behind a pair instruction
                  (d_fp && (e_ctl || m_ctl || w_ctl));
    logic d_block_r;
    always_ff @(posedge clk) d_block_r <= (!rst_n || kill_q) ? 1'b0 : d_adv ? blockA : adv ? blockB : blockC;
    assign d_wait = d_block_r || (d_fpop && d_fs && !d_half);
    wire d_mulh_go = adv && d_fpop && d_fs && !d_half && !d_block_r;   // the pair's multiply enters the datapath

    // ---------------------------------------------------------------- register writes (one per cycle)
    // The write of this cycle's commit (`wr_*`): the datapath's result (late in
    // the cycle: one select behind it), a load, or a delayed write. It goes to
    // `lw` and from there into fr4-fr31 a cycle later, so neither the result nor
    // the pipeline's commit is in front of the register file's write port.
    // A single is written from both halves of the data alike.
    wire         sel_dp = w_kind == K_OP || w_kind == K_FMA;        // an operation is in W (no load, no delayed write then)
    wire         dp_ok = !dp_unimpl && dp_we && !(w_kind == K_FMA && fma_un1);
    wire         ld_sgl = insn[31:26] == 6'h09;
    wire [63:0]  ld_wdata = ld_sgl ? {ld_data[31:0], ld_data[31:0]} : ld_data;
    wire [63:0]  alt_data = drain_v ? dr_data : ld_wdata;
    wire [4:0]   alt_n    = drain_v ? dr_n : insn[4:0];
    wire         alt_sgl  = drain_v ? dr_sgl : ld_sgl;
    wire         alt_half = drain_v ? dr_half : insn[6];
    wire         alt_en   = drain_v || ld_we;
    wire [63:0]  wr_data   = sel_dp ? dp_result : alt_data;
    wire [4:0]   wr_n      = sel_dp ? (w_kind == K_FMA ? w_ta : w_t) : alt_n;
    wire         wr_single = sel_dp ? (w_kind == K_FMA ? (w_sgl || w_cfxt) : w_sdest) : alt_sgl;   // dbl FMPYCFXT: the MSW of ta
    wire         wr_half   = sel_dp ? (w_kind == K_FMA ? w_hta : w_ht) : alt_half;
    wire         wr_en     = sel_dp ? (issue && dp_ok && !w_tlow) : alt_en;
    wire wr_l = wr_en && (!wr_single || !wr_half);
    wire wr_r = wr_en && (!wr_single || wr_half);

    always_ff @(posedge clk) begin
        if (lw_v && lw_l && lw_n >= 5'd4) begin
            frl_a[lw_n] <= lw_data[63:32]; frl_b[lw_n] <= lw_data[63:32]; frl_c[lw_n] <= lw_data[63:32]; frl_d[lw_n] <= lw_data[63:32];
            frl_s[lw_n] <= lw_data[63:32];
        end
        if (lw_v && lw_r && lw_n >= 5'd4) begin
            frr_a[lw_n] <= lw_data[31:0]; frr_b[lw_n] <= lw_data[31:0]; frr_c[lw_n] <= lw_data[31:0]; frr_d[lw_n] <= lw_data[31:0];
            frr_s[lw_n] <= lw_data[31:0];
        end
        // the last two commits' writes, and the operands of the instruction in D
        lw_v <= rst_n && wr_en; lw_l <= wr_l; lw_r <= wr_r; lw_n <= wr_n; lw_data <= wr_data;
        lw2_v <= rst_n && lw_v; lw2_l <= lw_l; lw2_r <= lw_r; lw2_n <= lw_n; lw2_data <= lw_data;
        xf_a <= rd_a; xf_b <= rd_b; xc_a <= rd_c; xc_b <= rd_d; x0_a <= cur_a; x0_b <= cur_b;
    end
    // An FP store's source is read while the store is in M and registered as it
    // enters W: the register file is then not in front of W's store data (the
    // store-to-load forward and what follows it), and need not sit beside the
    // D-cache. What the instruction in W writes at that same edge is taken from
    // the write itself (W commits when the pipeline moves, or everything behind
    // it is dropped). A store of fr0-fr3 waits in D for the operations ahead
    // (pa_core), so the status it reads is settled.
    wire         ww = sel_dp ? ((w_kind == K_FMA || !w_cmp) && !w_tlow) : (drain_v || w_isld);
    wire         st_hit = ww && wr_n == m_insn[4:0] && m_insn[4:0] >= 5'd4;
    wire         st_lw  = lw_v && lw_n == m_insn[4:0] && m_insn[4:0] >= 5'd4;      // on its way into the file
    wire [31:0]  st_l = (st_hit && (!wr_single || !wr_half)) ? wr_data[63:32] : (st_lw && lw_l) ? lw_data[63:32] : rd_s[63:32];
    wire [31:0]  st_r = (st_hit && (!wr_single || wr_half))  ? wr_data[31:0]  : (st_lw && lw_r) ? lw_data[31:0]  : rd_s[31:0];
    always_ff @(posedge clk) if (adv) st_data <= m_st_dbl ? {st_l, st_r} : {32'd0, m_insn[6] ? st_r : st_l};

    // fr0-fr3: from the loads and the delayed writes only
    wire xw_l = alt_en && !sel_dp && alt_n < 5'd4 && (!alt_sgl || !alt_half);
    wire xw_r = alt_en && !sel_dp && alt_n < 5'd4 && (!alt_sgl || alt_half);

    // ---------------------------------------------------------------- stage tracking
    always_ff @(posedge clk) begin
        if (adv) begin
            e_code <= d_code; e_t <= d_t; e_ht <= d_ht; e_sdest <= d_sdest;
            e_cmp <= d_op == F_CMP; e_ta <= d_ta; e_hta <= d_hta; e_tm <= d_tm; e_htm <= d_htm;
            e_sgl <= d_sgl; e_cfxt <= d_cfxt; e_slow_r <= d_slowdec; e_ctl_r <= d_ctl;
            e_tlow <= d_isfma ? d_ta < 5'd4 : d_t < 5'd4; m_tlow <= e_tlow; w_tlow <= m_tlow;
            m_code <= e_code; m_t <= e_t; m_ht <= e_ht; m_sdest <= e_sdest;
            m_cmp <= e_cmp; m_ta <= e_ta; m_hta <= e_hta; m_tm <= e_tm; m_htm <= e_htm;
            m_sgl <= e_sgl; m_cfxt <= e_cfxt; m_p1 <= e_p1;
            w_code <= m_code; w_t <= m_t; w_ht <= m_ht; w_sdest <= m_sdest;
            w_cmp <= m_cmp; w_ta <= m_ta; w_hta <= m_hta; w_tm <= m_tm; w_htm <= m_htm;
            w_sgl <= m_sgl; w_cfxt <= m_cfxt; w_p1 <= m_p1;
        end
        if (w_kind == K_MULH) begin
            fma_res1 <= dp_result; fma_we1 <= dp_we; fma_un1 <= dp_unimpl; fma_fl1 <= dp_flags;
        end
        // what a delayed write would carry (taken whenever an operation is in W: no commit in its enable)
        if (sel_dp) dr_data <= (w_kind == K_FMA) ? fma_res1 : dp_result;
        if (!rst_n || kill_q) begin
            e_kind <= K_NONE; m_kind <= K_NONE; w_kind <= K_NONE; d_half <= 1'b0; d_hf <= 1'b0;
            m_live <= 1'b0; w_live <= 1'b0; m_isld <= 1'b0; w_isld <= 1'b0; m_p1v <= 1'b0; m_p2v <= 1'b0; w_p1v <= 1'b0; w_p2v <= 1'b0; m_ctl <= 1'b0; w_ctl <= 1'b0;
            m_slowdec <= 1'b0; w_slowdec <= 1'b0; w_hold <= 1'b0;
        end else begin
            if (d_adv) d_half <= 1'b0;
            else if (d_mulh_go) d_half <= 1'b1;
            d_hf <= d_mulh_go;
            if (adv) begin
                // what enters E: the instruction leaving D, the multiply of a pair still in D, or nothing of ours
                e_kind <= (d_adv && d_fpop) ? d_kind : d_mulh_go ? K_MULH : K_NONE;
                m_kind <= (e_kind == K_MULH || e_fpop) ? e_kind : K_NONE;
                w_kind <= m_kind;
                m_live <= e_fpop; w_live <= m_live; m_isld <= e_fpld; w_isld <= m_isld; m_p1v <= e_p1v; m_p2v <= e_p2v; w_p1v <= m_p1v; w_p2v <= m_p2v;
                m_ctl <= e_ctl; w_ctl <= m_ctl; m_slowdec <= e_slowdec; w_slowdec <= m_slowdec;
                // a divide / square root entering W waits there for its recurrence
                w_hold <= dp_m_slow;
            end else if (w_hold) w_hold <= !dp_slow_xfer;
        end
    end

    // The status as FTEST in E must see it (ERS s.7.4.3): the C bit and the
    // compare queue with the compares still in M and W applied, oldest first.
    // (An unimplemented compare leaves them alone and sets T: the FTEST behind
    // it then traps 14 in W, whatever it decided.)
    function automatic logic [11:0] cq_push(input logic [11:0] q, input logic c);   // q = {C, FPSR[21:11]}
        cq_push = {c, q[11], q[10:1]};
    endfunction
    wire        w_fc = w_live && w_kind == K_OP && w_cmp && !dp_w_cun;
    wire        m_fc = m_live && m_kind == K_OP && m_cmp && !dp_m_cun;
    wire [11:0] cq0 = {fpsr[26], fpsr[21:11]};
    wire [11:0] cq1 = w_fc ? cq_push(cq0, dp_cmp) : cq0;
    wire [11:0] cq2 = m_fc ? cq_push(cq1, dp_m_cmp) : cq1;
    assign fpsr_o = {fpsr[31:27], cq2[11], fpsr[25:22], cq2[10:0], fpsr[10:0]};

    // FPSR.T is set at this edge (for the instruction that enters W with it)
    assign t_set = issue && (w_kind == K_UNIMPL || ((w_kind == K_OP || w_kind == K_FMA) && dp_unimpl) ||
                             (w_kind == K_FMA && fma_un1));

    // ---------------------------------------------------------------- the status and exception registers
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fpsr <= 32'd0;
            for (int i = 0; i < 7; i++) xr[i] <= 32'd0;
            drain_v <= 1'b0;
        end else begin
            drain_v <= 1'b0;
            // fr0..fr3 written by FP loads (and by an operation that targets them, a cycle late)
            case (alt_n[1:0])
                2'd0: begin if (xw_l) fpsr <= alt_data[63:32]; if (xw_r) xr[0] <= alt_data[31:0]; end
                2'd1: begin if (xw_l) xr[1] <= alt_data[63:32]; if (xw_r) xr[2] <= alt_data[31:0]; end
                2'd2: begin if (xw_l) xr[3] <= alt_data[63:32]; if (xw_r) xr[4] <= alt_data[31:0]; end
                default: begin if (xw_l) xr[5] <= alt_data[63:32]; if (xw_r) xr[6] <= alt_data[31:0]; end
            endcase
            // FSTD fr0 stored the status with T, then clears it (sec. 3.5)
            if (st_commit && insn[31:26] == 6'h0B && insn[4:0] == 5'd0) fpsr[6] <= 1'b0;

            // the operation in W commits
            if (issue) case (w_kind)
                K_FID: fpsr[21:11] <= {6'h0D, 5'h01};                       // model 0x0D, revision 1 (ERS s.7.1)
                K_FTEST: ;                                                   // nullification is decided in E
                K_UNIMPL: begin fpsr[6] <= 1'b1; xr[1] <= {w_code, insn[25:0]}; end
                K_OP: begin
                    if (dp_unimpl) begin fpsr[6] <= 1'b1; xr[1] <= {w_code, insn[25:0]}; end
                    else begin
                        fpsr[31:27] <= fpsr[31:27] | dp_flags;
                        if (w_cmp) begin                                     // the compare queue (ERS s.7.4.3)
                            fpsr[26] <= dp_cmp; fpsr[21] <= fpsr[26]; fpsr[20:11] <= fpsr[21:12];
                        end
                        // a target among fr0-fr3: written in the next cycle
                        if (w_tlow && dp_we) begin
                            drain_v <= 1'b1; dr_n <= w_t; dr_sgl <= w_sdest; dr_half <= w_ht;
                        end
                    end
                end
                K_FMA: begin
                    if (dp_unimpl || fma_un1) begin                          // neither result, no flag
                        fpsr[6] <= 1'b1; xr[1] <= {w_code, insn[25:0]};
                    end else begin
                        fpsr[31:27] <= fpsr[31:27] | dp_flags | fma_fl1;
                        // the multiply's result follows in the next cycle
                        drain_v <= fma_we1; dr_n <= w_tm; dr_sgl <= w_sgl; dr_half <= w_htm;
                    end
                end
                default: ;
            endcase
        end
    end
endmodule
