#!/usr/bin/env python3
"""
run_regression.py: build and run the radar testbench, optionally with mutation testing.

    python run_regression.py                       # one clean run (Verilator if found, else Questa)
    python run_regression.py --sim questa          # Questa / ModelSim (vlib, vlog, vsim on PATH)
    python run_regression.py --mutations           # also build 12 buggy RTL variants; each must FAIL
    python run_regression.py --mutations --only TRUNC,NOSAT

Run it from the project root (the folder that contains rtl/, tb/ and vectors/).

Mutation testing: every MUT_* switch in the RTL injects one realistic bug (truncating instead of rounding,
missing saturation, wrong twiddle index, wrong bit-reversal, ...). A mutant is KILLED when the testbench
reports anything other than TESTBENCH PASSED. The mutation score is killed / total.
"""
import argparse, concurrent.futures as cf, os, re, shutil, subprocess, sys, tempfile, time

RTL = ["radar_pkg", "radar_delay", "radar_cmul", "radar_fft_stage", "radar_fft256", "radar_inbuf", "radar_refmul",
       "radar_bitrev_buf", "radar_cfar", "radar_regs", "radar_top"]
TB  = ["radar_tb_pkg", "radar_sva", "radar_bind", "radar_tb"]

MUTATIONS = {
    "TRUNC":   "FFT/multiply rounding replaced by truncation",
    "NOSAT":   "saturation replaced by wrap-around",
    "TWIDX":   "twiddle-factor index off by one",
    "CMULRND": "complex multiplier without rounding constant",
    "BITREV":  "bit-reversal wrong on one bit",
    "NOCONJ":  "conjugates for the inverse transform omitted",
    "HALF":    "keeps the wrong half of each overlap-save block",
    "GUARD":   "CFAR training window one cell too small",
    "PEAKWIN": "peak-report window +/-1 instead of +/-2",
    "MAG2RND": "|z|^2 truncated instead of rounded",
    "ALPHA":   "CFAR threshold scaled by the wrong power of two",
    "OVERRUN": "input back-pressure rule removed (bank overrun)",
}

VERILATOR_FLAGS = ["--binary", "--timing", "--assert", "-Wno-fatal", "-Wno-WIDTH", "-Wno-INITIALDLY", "-Wno-UNOPTFLAT",
                   "-Wno-CASEINCOMPLETE", "-Wno-STMTDLY", "-Wno-LATCH", "-Wno-BLKSEQ", "--top-module", "radar_tb"]


def run(cmd, cwd, timeout):
    try:
        p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
        return p.returncode, p.stdout
    except subprocess.TimeoutExpired as e:
        return 124, (e.stdout or "") + "\nTIMEOUT"


def build_and_run(sim, defines, vec_dir, tag, timeout):
    proj = os.getcwd()
    work = tempfile.mkdtemp(prefix=f"radar_{tag}_")
    t0 = time.time()
    try:
        if sim == "verilator":
            files = [f"rtl/{m}.sv" for m in RTL] + [f"tb/{m}.sv" for m in TB]
            cmd = ["verilator", *VERILATOR_FLAGS, "--Mdir", work, f"+incdir+{vec_dir}", f'-DVEC_DIR="{vec_dir}"',
                   *[f"-D{d}" for d in defines], *files]
            rc, out = run(cmd, proj, timeout)
            exe = os.path.join(work, "Vradar_tb")
            if rc != 0 or not os.path.exists(exe):
                return "BUILD-ERROR", out[-1500:], time.time() - t0
            rc, out = run([exe], proj, timeout)
        else:
            # Questa/ModelSim: run vlib/vlog/vsim FROM the temp folder with a plain relative library
            # name ("work"). Passing an absolute Windows path as -work's value breaks some Questa
            # versions' auto-optimize step (it concatenates the path with ".<unit>" into one garbled
            # token). Source files and the vectors folder are referenced by absolute path instead,
            # since the simulator's working directory is no longer the project root.
            abs_files = [os.path.join(proj, "rtl", f"{m}.sv") for m in RTL] + \
                        [os.path.join(proj, "tb", f"{m}.sv") for m in TB]
            abs_vec = vec_dir if os.path.isabs(vec_dir) else os.path.join(proj, vec_dir)
            vec_str = abs_vec.replace("\\", "/")   # goes into a Verilog string literal: force forward slashes
            rc, out = run(["vlib", "work"], work, timeout)
            if rc != 0:
                return "BUILD-ERROR", out[-1500:], time.time() - t0
            rc, out = run(["vlog", "-sv", "-work", "work", f"+incdir+{abs_vec}", f'+define+VEC_DIR="{vec_str}"',
                          *[f"+define+{d}" for d in defines], *abs_files], work, timeout)
            if rc != 0:
                return "BUILD-ERROR", out[-1500:], time.time() - t0
            rc, out = run(["vsim", "-c", "-work", "work", "radar_tb", "-do", "run -all; quit -f"], work, timeout)
        status = "PASSED" if "TESTBENCH PASSED" in out and rc == 0 else "FAILED"
        return status, out, time.time() - t0
    finally:
        shutil.rmtree(work, ignore_errors=True)


def first_failure(out):
    for line in out.splitlines():
        if re.search(r"FAIL|SVA FAIL|TIMEOUT|mismatch", line):
            return line.strip()[:110]
    return "(no message)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", choices=["verilator", "questa"], default=None)
    ap.add_argument("--vec-dir", default="vectors")
    ap.add_argument("--mutations", action="store_true")
    ap.add_argument("--only", default="", help="comma-separated mutation names")
    ap.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1))
    ap.add_argument("--timeout", type=int, default=600)
    a = ap.parse_args()

    sim = a.sim or ("verilator" if shutil.which("verilator") else "questa")
    if not shutil.which("verilator" if sim == "verilator" else "vsim"):
        sys.exit(f"simulator '{sim}' not found on PATH")
    vec_dir = os.path.abspath(a.vec_dir) if sim == "verilator" else a.vec_dir
    if not os.path.exists(os.path.join(a.vec_dir, "num_tests.hex")):
        sys.exit(f"{a.vec_dir}/num_tests.hex not found: run radar_make_vectors.py first")

    print(f"simulator: {sim}\n-- clean design")
    status, out, sec = build_and_run(sim, [], vec_dir, "clean", a.timeout)
    lines = [l for l in out.splitlines() if l.strip()]
    tail = lines if status != "PASSED" else lines[-14:]   # on failure, show everything (PASS/FAIL per test, mismatches)
    print("\n".join("   " + l for l in tail))
    print(f"clean design: {status} ({sec:.0f} s)")
    if status != "PASSED":
        sys.exit(1)
    if not a.mutations:
        return

    names = [n for n in (a.only.split(",") if a.only else MUTATIONS) if n]
    print(f"\n-- mutation testing: {len(names)} mutants, {a.jobs} in parallel")
    results = {}
    with cf.ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = {ex.submit(build_and_run, sim, [f"MUT_{n}"], vec_dir, n, a.timeout): n for n in names}
        for f in cf.as_completed(futs):
            n = futs[f]
            st, out, sec = f.result()
            results[n] = (st, out, sec)
            print(f"   done {n} ({sec:.0f} s)", flush=True)

    killed = 0
    print(f"\n{'mutant':9s} {'result':9s} bug / first failure message")
    for n in names:
        st, out, sec = results[n]
        if st == "FAILED":
            killed += 1
            print(f"{n:9s} KILLED    {MUTATIONS[n]}\n{'':19s}-> {first_failure(out)}")
        elif st == "BUILD-ERROR":
            print(f"{n:9s} BUILD-ERR  {MUTATIONS[n]}\n{out}")
        else:
            print(f"{n:9s} SURVIVED  {MUTATIONS[n]}   <-- testbench did not notice")
    print(f"\nmutation score: {killed}/{len(names)} killed ({100 * killed // len(names)}%)")
    sys.exit(0 if killed == len(names) else 2)


if __name__ == "__main__":
    main()
