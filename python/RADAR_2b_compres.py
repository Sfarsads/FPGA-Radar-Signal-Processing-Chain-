    
import numpy as np
import matplotlib.pyplot as plt

# ld 2a
d = np.load("radar_stream.npz")
stream = (d["stream_i"] + 1j * d["stream_q"]) / 32768
chirp = (d["chirp_i"]  + 1j * d["chirp_q"])  / 32768
TARGET_POS = int(d["target_pos"])
N_FFT = 256               
P = len(chirp)        
HOP = N_FFT - P        

# ref chrip
H = np.conj(np.fft.fft(chirp, N_FFT))

# strmdat
n_blocks = int(np.ceil(len(stream) / HOP))
padded   = np.concatenate([stream, np.zeros(N_FFT, dtype=complex)])
out      = np.zeros(n_blocks * HOP, dtype=complex)

for b in range(n_blocks):
    block   = padded[b * HOP : b * HOP + N_FFT]
    y = np.fft.ifft(np.fft.fft(block) * H)
    out[b * HOP : (b + 1) * HOP] = y[:HOP]  

out = out[:len(stream)]

# check
direct = np.correlate(stream, chirp, mode="full")[P - 1 : P - 1 + len(stream)]
print(f"max difference FFT vs direct: {np.max(np.abs(out - direct)):.2e} (want ~1e-15)")
mag = np.abs(out)
peak = int(np.argmax(mag))
print(f"peak found at sample {peak}, true position {TARGET_POS}")
full = mag[: len(stream) - P + 1]                                
mask = np.ones(len(full), dtype=bool)
mask[max(0, peak - 20) : peak + 21] = False                      
floor_rms = np.sqrt(np.mean(full[mask] ** 2))
print(f"peak / noise floor: {20 * np.log10(mag[peak] / floor_rms):.1f} dB (expect ~16 dB)")

np.save("compressed_float.npy", out)

# plot
fig, ax = plt.subplots(2, 1, figsize=(10, 6))
ax[0].plot(np.abs(stream))
ax[0].set_title("Before compression: |signal| (echo invisible)")
ax[1].plot(mag)
ax[1].axvline(TARGET_POS, color="r", linestyle="--", label="true echo position")
ax[1].set_title("After pulse compression: |output| (echo spikes out of the noise)")
ax[1].legend()
plt.tight_layout()
plt.savefig("fig_2b_compression.png", dpi=150)
plt.show()
