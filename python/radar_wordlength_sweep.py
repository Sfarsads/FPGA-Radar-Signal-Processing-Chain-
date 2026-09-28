"""radar_wordlength_sweep.py: how many bits does the datapath need?

Sweeps the internal word length (data, twiddle and stored-reference words all set to W bits;
the 16-bit input stays fixed) and compares the bit-accurate model against floating point:

  SQNR          how close the compressed output is to float (dB, higher is better)
  loss          drop in peak-to-noise-floor vs float (dB)
  Pd            detection probability at a marginal SNR
  Pfa           measured false-alarm rate on noise only (target 1e-4)
  sat           saturation events across the test streams

Takes about a minute. Writes wordlength_sweep.csv and fig_wordlength_sweep.png.
"""
import csv

import numpy as np
import matplotlib.pyplot as plt

import radar_common as rc
import radar_golden as g

WIDTHS     = [8, 10, 12, 14, 16, 18, 20, 24]
SNR_QUAL   = -5.0        # SNR for the SQNR / peak-to-floor measurements
SNR_MARG   = -10.0       # marginal SNR for the Pd measurement (float Pd is about 0.55 here)
N_QUAL     = 100
N_PD       = 1000
N_PFA      = 256         # noise-only streams (~0.5 M cells)

ci, cq = rc.chirp_int()
cf = (ci + 1j * cq) / 32768.0
lo = rc.GUARD + rc.TRAIN

# same streams for every word length, so the comparison is fair
zq, posq = rc.make_streams(N_QUAL, snr_db=SNR_QUAL, seed=1)
zp, posp = rc.make_streams(N_PD, snr_db=SNR_MARG, seed=2)
zn, _ = rc.make_streams(N_PFA, with_target=False, seed=3)
iq, qq_ = rc.quantize(zq)
ip, qp = rc.quantize(zp)
inn, qn = rc.quantize(zn)

ref_q = rc.compress_float((iq + 1j * qq_) / 32768.0, cf)
ref_p = rc.compress_float((ip + 1j * qp) / 32768.0, cf)
ref_n = rc.compress_float((inn + 1j * qn) / 32768.0, cf)
pf_float = rc.peak_to_floor_db(np.abs(ref_q), posq).mean()
pd_float = rc.detected(rc.cfar_float(np.abs(ref_p) ** 2)[0], posp).mean()
hi = rc.valid_len(zn.shape[-1]) - lo
fa_float = rc.cfar_float(np.abs(ref_n) ** 2)[0][:, lo:hi].mean()

print(f"float reference: peak/floor {pf_float:.2f} dB, Pd@{SNR_MARG:g} dB {pd_float:.3f}, Pfa {fa_float:.2e}\n")
print(" bits |  SQNR dB | loss dB |   Pd   |   Pfa    | sat")
print("------+----------+---------+--------+----------+-----")

rows = []
for W in WIDTHS:
    cfg = g.Cfg(data_bits=W, tw_bits=W, ref_bits=W)

    r = g.run_pipeline(iq, qq_, ci, cq, cfg)
    fx = g.to_float(r["out_i"], r["out_q"], cfg, r["info"])
    sqnr = 10 * np.log10(np.sum(np.abs(ref_q) ** 2) / np.sum(np.abs(fx - ref_q) ** 2))
    loss = pf_float - rc.peak_to_floor_db(np.abs(fx), posq).mean()
    sat = r["info"]["sat"]

    pd = rc.detected(g.run_pipeline(ip, qp, ci, cq, cfg)["flags"], posp).mean()
    fa = g.run_pipeline(inn, qn, ci, cq, cfg)["flags"][:, lo:hi].mean()

    rows.append(dict(bits=W, sqnr_db=sqnr, loss_db=loss, pd=pd, pfa=fa, sat=sat))
    print(f" {W:4d} | {sqnr:8.1f} | {loss:7.2f} | {pd:6.3f} | {fa:8.2e} | {sat}")

with open("wordlength_sweep.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

# smallest width that costs (almost) nothing
good = [r["bits"] for r in rows if r["loss_db"] < 0.1 and abs(r["pd"] - pd_float) < 0.03 and r["sat"] == 0]
if good:
    print(f"\nsmallest width with <0.1 dB loss, Pd within 0.03 of float, no saturation: {min(good)} bits")

# ---- plot ---------------------------------------------------------------
bits = [r["bits"] for r in rows]
fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
ax[0].plot(bits, [r["sqnr_db"] for r in rows], "o-")
ax[0].set_title("SQNR vs float"); ax[0].set_ylabel("dB")
ax[1].plot(bits, [r["loss_db"] for r in rows], "o-", color="tab:red")
ax[1].set_title("Peak-to-floor loss vs float"); ax[1].set_ylabel("dB")
ax[2].plot(bits, [r["pd"] for r in rows], "o-", color="tab:green", label="fixed")
ax[2].axhline(pd_float, color="gray", ls="--", label="float")
ax[2].set_title(f"Pd at {SNR_MARG:g} dB SNR"); ax[2].legend()
for a in ax:
    a.set_xlabel("internal word length (bits)")
    a.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("fig_wordlength_sweep.png", dpi=150)
plt.show()
