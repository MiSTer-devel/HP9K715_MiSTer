#!/bin/bash
# Run the pacputest suite ROMs (tests/cpu) on the RTL CPU (verilator/cputest)
# and check each log: against tests/cpu/model.py for the suites it covers,
# against QEMU's reference log for the rest (known QEMU bugs excused).
#   [CORE=seq|pipe|pipeseq] tests/run-cputest.sh [suite ...]   default: every suite (the fp suites need pipe or pipeseq)
# CORE selects the core: pipe (default) = the pipelined pa_cpu as it is built, pipeseq = the same with the
# sequential FP unit of build 65 (PA_FPU_SEQ), seq = pa_core_seq.
# Exits non-zero if any suite fails.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
V=$HERE/../verilator
CPUT=$HERE/cpu
SUITES=${*:-alu1 alu2 shift max1 mem branch trap sys cache tlbx fp fpedge fpseq link}
CORE=${CORE:-pipe}
export CORE
make -s -C "$CPUT" >/dev/null || exit 1
mkdir -p "$V/obj_dir_ct/$CORE"
make -s -C "$V" obj_dir_ct/$CORE/Vcputest_top >/dev/null || exit 1
fail=0
for s in $SUITES; do
    rm -f "$V/obj_dir_ct/$s.hex"
    make -s -C "$V" "obj_dir_ct/$s.hex" >/dev/null || { echo "$s: no ROM"; fail=1; continue; }
    run=$("$V/obj_dir_ct/$CORE/Vcputest_top" +rom="$V/obj_dir_ct/$s.hex" --log "$V/obj_dir_ct/$s.log" \
          --max-cycles 200000000 2>&1 | tail -1)
    # the test bench's last line when an address was retired twice in a row with no interruption
    # between (a delay slot run twice); the link suite's branches to their own slot do that by design
    case "$run" in *DUPLICATE*) [ "$s" = link ] || { echo "$s: $run"; fail=1; } ;; esac
    if [ "$s" = link ]; then
        # the B,L link suite (cpu-requests #10): its own checker, no reference log
        res=$(python3 "$CPUT/check_link.py" "$V/obj_dir_ct/$s.log" 2>&1)
        tot=$(echo "$res" | tail -1)
        if echo "$res" | tail -1 | grep -q ' 0 bad slots'; then st=PASS; else st=FAIL; fail=1; fi
        printf "%-7s %s  %s\n          (%s)\n" "$s" "$st" "$tot" "${run#cputest: }"
        continue
    fi
    case $s in
        alu1|alu2|shift|max1|cache|tlbx|fpedge) ref="--model" ;;
        fpseq)                ref="--ref $CPUT/ref/core65/$s.log" ;;      # random FP streams: the release core of 2026-10-18 is the reference
        *)                    ref="--ref $CPUT/ref/qemu/$s.log --known-qemu" ;;
    esac
    res=$(python3 "$CPUT/compare.py" $s "$V/obj_dir_ct/$s.log" $ref 2>&1)
    tot=$(echo "$res" | grep '^total' | sed 's/^total: //')
    if echo "$res" | grep -q '^result: PASS'; then st=PASS; else st=FAIL; fail=1; fi
    printf "%-7s %s  %s\n          (%s)\n" "$s" "$st" "$tot" "${run#cputest: }"
done
exit $fail
