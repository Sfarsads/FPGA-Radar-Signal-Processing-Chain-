"""radar_detection_stats.py: measure detection performance by Monte Carlo.

1. Threshold check: measures the false-alarm rate on noise only (millions of cells) with
   (a) the textbook CFAR multiplier and (b) the calibrated one, and re-derives the multiplier
   that would give exactly the target Pfa.
2. Pd vs SNR: detection probability across input SNR for float and the bit-accurate model.

Takes about a minute. Writes detection_stats.csv and fig_detection_stats.png.
"""
import csv

import numpy as np
import matplotlib.pyplot as plt

import radar_common as rc
import radar_golden as g

L           = 2048
PFA_STREAMS = 2048           # x ~1900 cells = ~3.9 M noise-only cells
CHUNK       = 512
PD_TRIALS   = 500
SNRS        = list(range(-15, -3))          # input SNR per sample, dB

ci, cq = rc.chirp_int()
cf = (ci + 1j * cq) / 32768.0
lo = rc.GUARD + rc.TRAIN
hi = rc.valid_len(L) - lo
comp_gain_db = 10 * np.log10(rc.PULSE_LEN)   # coherent gain of the matched filter

cfg_cal = g.Cfg()                                          # calibrated multiplier
cfg_txt = g.Cfg(alpha=rc.cfar_alpha_closed_form())         # textbook multiplier

# ---------------------------------------------------------------- 1. false-alarm rate
cells = 0
fa = dict(float_txt=0, float_cal=0, fixed_txt=0, fixed_cal=0)
ratios = []
for c in range(PFA_STREAMS // CHUNK):
    z, _ = rc.make_streams(CHUNK, length=L, with_target=False, seed=100 + c)
    ii, qq = rc.quantize(z)
    ref = rc.compress_float((ii + 1j * qq) / 32768.0, cf)
    p = np.abs(ref) ** 2
    cells += CHUNK * (hi - lo)
    fa["float_txt"] += rc.cfar_float(p, alpha=cfg_txt.alpha)[0][:, lo:hi].sum()
    fa["float_cal"] += rc.cfar_float(p, alpha=cfg_cal.alpha)[0][:, lo:hi].sum()
    i, s = rc.train_sum(p)
    ratios.append((p[..., i] / s).ravel())

    oi, oq, _ = g.compress_fixed(ii, qq, ci, cq, cfg_cal)
    mag2 = g.rshift_round(oi ** 2 + oq ** 2, cfg_cal.mag2_sh)
    fa["fixed_txt"] += g.cfar_fixed(mag2, cfg_txt)[0][:, lo:hi].sum()
    fa["fixed_cal"] += g.cfar_fixed(mag2, cfg_cal)[0][:, lo:hi].sum()

alpha_needed = np.quantile(np.concatenate(ratios), 1 - rc.PFA)

print(f"noise-only cells tested: {cells:,}   target Pfa: {rc.PFA:.0e}\n")
print("multiplier          | float Pfa  | fixed Pfa  (fixed = bit-accurate, 18-bit)")
for name, key, a in [("textbook", "txt", cfg_txt.alpha), ("calibrated", "cal", cfg_cal.alpha)]:
    print(f"{name:10s} {a:6.3f}   | {fa['float_' + key] / cells:.2e}   | {fa['fixed_' + key] / cells:.2e}")
n = fa["fixed_cal"]
print(f"\ncalibrated fixed-point Pfa = {n / cells:.2e}  (95% interval {max(n - 1.96 * np.sqrt(n), 0) / cells:.2e} to {(n + 1.96 * np.sqrt(n)) / cells:.2e})")
print(f"multiplier that would hit {rc.PFA:.0e} exactly on this data: {alpha_needed:.3f}   (stored: {rc.ALPHA})")
if abs(alpha_needed / rc.ALPHA - 1) > 0.03:
    print("-> more than 3% off: consider updating ALPHA in radar_common.py")

# ---------------------------------------------------------------- 2. Pd vs SNR
print(f"\n SNR in | SNR after compression |  Pd float | Pd fixed")
rows = []
for snr in SNRS:
    z, pos = rc.make_streams(PD_TRIALS, length=L, snr_db=snr, seed=7)
    ii, qq = rc.quantize(z)
    ref = rc.compress_float((ii + 1j * qq) / 32768.0, cf)
    pd_fl = rc.detected(rc.cfar_float(np.abs(ref) ** 2)[0], pos).mean()
    pd_fx = rc.detected(g.run_pipeline(ii, qq, ci, cq, cfg_cal)["flags"], pos).mean()
    rows.append(dict(snr_in_db=snr, snr_out_db=snr + comp_gain_db, pd_float=pd_fl, pd_fixed=pd_fx))
    print(f" {snr:5d}  | {snr + comp_gain_db:20.1f}   |  {pd_fl:8.3f} | {pd_fx:8.3f}")

with open("detection_stats.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

snr_arr = np.array([r["snr_in_db"] for r in rows], dtype=float)
pd_arr = np.array([r["pd_fixed"] for r in rows])
if pd_arr.max() >= 0.9 and pd_arr.min() < 0.9:
    snr90 = np.interp(0.9, pd_arr, snr_arr)
    print(f"\nfixed-point Pd = 90% at about {snr90:.1f} dB input SNR ({snr90 + comp_gain_db:.1f} dB after compression), Pfa {n / cells:.1e}")

# ---------------------------------------------------------------- plot
fig, ax = plt.subplots(figsize=(7.5, 4.5))
ax.plot(snr_arr, [r["pd_float"] for r in rows], "o-", label="float")
ax.plot(snr_arr, pd_arr, "s--", label="bit-accurate fixed point (18-bit)")
ax.axhline(0.9, color="gray", ls=":", lw=1)
ax.set_xlabel("input SNR per sample (dB), before pulse compression")
ax.set_ylabel("probability of detection")
ax.set_title(f"Detection performance, Pfa ~ {n / cells:.1e}")
ax.grid(alpha=0.3)
ax.legend()
plt.tight_layout()
plt.savefig("fig_detection_stats.png", dpi=150)
plt.show()
