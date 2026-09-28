
import numpy as np
import matplotlib.pyplot as plt

import radar_common as rc

# ---- load ---------------------------------------------------------------
d = np.load("radar_stream.npz")
TARGET_POS = int(d["target_pos"])
out = np.load("compressed_float.npy")          # written by radar_2b.py
power = np.abs(out) ** 2                        # square-law detector: |I|^2 + |Q|^2

# ---- CFAR ---------------------------------------------------------------
flags, thr = rc.cfar_float(power)
reports = rc.peak_reports(flags, power)

alpha = rc.ALPHA
print(f"target Pfa {rc.PFA:g}, {2 * rc.TRAIN} training cells, {rc.GUARD} guard cells each side")
print(f"threshold = {alpha:.3f} x (sum of training cells) = {alpha * 2 * rc.TRAIN:.2f} x (mean noise power)")
print(f"(textbook formula would say {rc.cfar_alpha_closed_form():.3f}; radar_detection_stats.py explains why we don't use it)")

flag_idx = np.flatnonzero(flags)
report_idx = np.flatnonzero(reports)
print(f"cells above threshold: {flag_idx.tolist()}")
print(f"reported targets (local peaks): {report_idx.tolist()}   true position: {TARGET_POS}")

false_cells = [i for i in flag_idx if abs(i - TARGET_POS) > 2]
print(f"false alarms away from the target: {len(false_cells)}")

# ---- plot ---------------------------------------------------------------
fig, ax = plt.subplots(2, 1, figsize=(10, 7))

ax[0].plot(power, label="compressed power |y|^2", lw=0.8)
ax[0].plot(thr, color="orange", label="CFAR threshold", lw=1)
ax[0].axvline(TARGET_POS, color="r", ls="--", alpha=0.6, label="true echo position")
ax[0].plot(report_idx, power[report_idx], "gv", ms=9, label="reported target")
ax[0].set_yscale("log")
ax[0].set_title("CA-CFAR: the threshold follows the noise level")
ax[0].set_xlabel("sample")
ax[0].legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=4, frameon=False)

lo_z, hi_z = max(TARGET_POS - 40, 0), TARGET_POS + 41
xs = np.arange(lo_z, hi_z)
ax[1].plot(xs, power[lo_z:hi_z], "o-", ms=3, label="compressed power")
ax[1].plot(xs, thr[lo_z:hi_z], color="orange", label="CFAR threshold")
ax[1].plot(report_idx[(report_idx >= lo_z) & (report_idx < hi_z)],
           power[report_idx[(report_idx >= lo_z) & (report_idx < hi_z)]], "gv", ms=10, label="reported target")
ax[1].set_title("Zoom on the echo: the peak crosses the threshold, the noise does not")
ax[1].set_xlabel("sample")
ax[1].legend()

plt.tight_layout()
plt.savefig("fig_2c_cfar.png", dpi=150)
plt.show()
