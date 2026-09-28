import numpy as np
import matplotlib.pyplot as plt

PULSE_LEN = 128
BANDWIDTH = 0.5
STREAM_LEN = 2048
TARGET_POS = 700
SNR_DB = -5
AMP = 0.1
SEED = 42
rng = np.random.default_rng(SEED)

#1_chirp_complex
n = np.arange(PULSE_LEN)
chirp = AMP * np.exp(1j * np.pi * (BANDWIDTH / PULSE_LEN) * (n - PULSE_LEN / 2) ** 2)

#2_noise+echo
noise_power = AMP**2 / 10 ** (SNR_DB / 10)
sigma = np.sqrt(noise_power / 2)
noise = sigma * (rng.standard_normal(STREAM_LEN) + 1j * rng.standard_normal(STREAM_LEN))
stream = noise.copy()
stream[TARGET_POS:TARGET_POS + PULSE_LEN] += chirp

#3_quant16int
def to_q15(x):
    return np.clip(np.round(x * 32768), -32768, 32767).astype(np.int16)
chirp_q   = to_q15(chirp.real)  + 1j * to_q15(chirp.imag)
stream_i  = to_q15(stream.real)
stream_q  = to_q15(stream.imag)
clipped = np.sum((np.abs(stream.real) >= 1) | (np.abs(stream.imag) >= 1))

print(f"stream: {STREAM_LEN} samples, echo at {TARGET_POS}..{TARGET_POS + PULSE_LEN - 1}")
print(f"int16 range used: I [{stream_i.min()}, {stream_i.max()}]  Q [{stream_q.min()}, {stream_q.max()}]")
print(f"clipped samples: {clipped} (want 0)")

#ltr
np.savez("radar_stream.npz",
         stream_i=stream_i, stream_q=stream_q,
         chirp_i=to_q15(chirp.real), chirp_q=to_q15(chirp.imag),
         target_pos=TARGET_POS)

#plot 
fig, ax = plt.subplots(2, 1, figsize=(10, 6))
ax[0].plot(n, chirp.real)
ax[0].set_title("The chirp we're hiding (real part)")
ax[1].plot(stream_i)
ax[1].axvspan(TARGET_POS, TARGET_POS + PULSE_LEN, color="r", alpha=0.2, label="where the echo is")
ax[1].set_title("What the radar 'sees' (real part, 16-bit ints): echo is invisible")
ax[1].legend()
plt.tight_layout()
plt.show()