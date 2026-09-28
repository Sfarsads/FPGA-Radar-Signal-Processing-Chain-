
import numpy as np
import matplotlib.pyplot as plt

import radar_common as rc
import radar_golden as g

# ---- load the 2a stream -------------------------------------------------
d = np.load("radar_stream.npz")
in_i, in_q = d["stream_i"].astype(np.int64), d["stream_q"].astype(np.int64)
ci, cq = d["chirp_i"].astype(np.int64), d["chirp_q"].astype(np.int64)
pos = int(d["target_pos"])

# ---- fixed-point golden model -------------------------------------------
cfg = g.Cfg()
res = g.run_pipeline(in_i, in_q, ci, cq, cfg)
info = res["info"]
fixed = g.to_float(res["out_i"], res["out_q"], cfg, info)

# ---- floating-point reference on the SAME quantized inputs ---------------
scale = float(1 << (rc.IN_BITS - 1))
ref = rc.compress_float((in_i + 1j * in_q) / scale, (ci + 1j * cq) / scale)
ref_flags, _ = rc.cfar_float(np.abs(ref) ** 2)

# ---- report -------------------------------------------------------------
err = fixed - ref
sqnr = 10 * np.log10(np.sum(np.abs(ref) ** 2) / np.sum(np.abs(err) ** 2))
pf_fixed = rc.peak_to_floor_db(np.abs(fixed), pos)[0]
pf_float = rc.peak_to_floor_db(np.abs(ref), pos)[0]

print(f"word lengths: data {cfg.data_bits}, twiddle {cfg.tw_bits}, reference {cfg.ref_bits} bits")
print(f"reference scale k = {info['k']}, alpha_q = {info['alpha_q']} (Q.{cfg.alpha_frac}), saturation events = {info['sat']}")
print(f"compression SQNR vs float: {sqnr:.1f} dB")
print(f"peak/noise floor: float {pf_float:.2f} dB, fixed {pf_fixed:.2f} dB (loss {pf_float - pf_fixed:.2f} dB)")
print(f"CFAR cells flagged  fixed: {np.flatnonzero(res['flags']).tolist()}")
print(f"CFAR cells flagged  float: {np.flatnonzero(ref_flags).tolist()}")
print(f"reported targets (fixed): {np.flatnonzero(res['reports']).tolist()}   true position: {pos}")
print(f"flag decisions that differ from float: {int(np.sum(res['flags'] != ref_flags))} of {len(ref_flags)}")

# ---- plot ---------------------------------------------------------------
fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax[0].plot(np.abs(ref), label="float", lw=1.5)
ax[0].plot(np.abs(fixed), "--", label=f"fixed ({cfg.data_bits}-bit)", lw=1)
ax[0].axvline(pos, color="r", ls=":", alpha=0.6)
ax[0].set_title("Compressed magnitude: float vs bit-accurate fixed point")
ax[0].legend()
ax[1].plot(np.abs(err), color="tab:red", lw=0.8)
ax[1].set_title(f"Error magnitude (SQNR {sqnr:.1f} dB)")
ax[1].set_xlabel("sample")
plt.tight_layout()
plt.savefig("fig_2d_fixed_vs_float.png", dpi=150)
plt.show()
