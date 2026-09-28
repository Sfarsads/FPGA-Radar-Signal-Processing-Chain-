"""radar_common.py: shared settings + helpers for the radar project.

Every other radar_*.py script imports this, so keep all the scripts in one folder.
"""
import numpy as np

# ---- signal / architecture settings (same values as radar_2a.py, radar_2b.py) ----
PULSE_LEN = 128            # chirp length in samples
BANDWIDTH = 0.5            # chirp sweep, as a fraction of the sample rate
AMP       = 0.1            # chirp amplitude, fraction of full scale
IN_BITS   = 16             # input I/Q word length (Q1.15)
N_FFT     = 256
HOP       = N_FFT - PULSE_LEN   # 128 clean outputs per block (overlap-save)

# ---- CFAR settings ----
GUARD     = 4              # guard cells each side of the cell under test
TRAIN     = 8              # training cells each side (16 total)
PFA       = 1e-4           # target false-alarm probability per cell
ALPHA     = 0.96           # CFAR threshold multiplier on the SUM of the 16 training cells,
                           # calibrated by Monte Carlo (radar_detection_stats.py re-checks it)
PEAK_HALF = 2              # report a flagged sample only if it is the max within +/- this many samples


def cfar_alpha_closed_form(pfa=PFA, train=TRAIN):
    return pfa ** (-1.0 / (2 * train)) - 1.0


# ---------------------------------------------------------------- stimulus
def make_chirp():
    n = np.arange(PULSE_LEN)
    return AMP * np.exp(1j * np.pi * (BANDWIDTH / PULSE_LEN) * (n - PULSE_LEN / 2) ** 2)


def to_q(x, bits=IN_BITS):
    """Float (fraction of full scale) -> saturating signed integer."""
    s = 1 << (bits - 1)
    return np.clip(np.round(x * s), -s, s - 1).astype(np.int64)


def quantize(z, bits=IN_BITS):
    """Complex float array -> (I ints, Q ints)."""
    return to_q(z.real, bits), to_q(z.imag, bits)


def chirp_int():
    """The reference chirp as 16-bit ints (what gets stored on the FPGA)."""
    return quantize(make_chirp())


def make_streams(trials, length=2048, snr_db=-5.0, target_pos=None,
                 with_target=True, gain=1.0, seed=0):
    """Batch of fake radar returns, complex float, shape (trials, length).

    target_pos : None -> random position per trial, int or array -> fixed position(s)
    snr_db     : echo power vs noise power per sample (same definition as radar_2a.py)
    gain       : scales the whole stream (used for overflow / saturation stress tests)
    Returns (streams, target_positions).
    """
    rng = np.random.default_rng(seed)
    chirp = make_chirp()
    sigma = np.sqrt(AMP ** 2 / 10 ** (snr_db / 10) / 2)      # per I and per Q component
    z = sigma * (rng.standard_normal((trials, length)) + 1j * rng.standard_normal((trials, length)))
    rand_pos = rng.integers(100, length - PULSE_LEN - 100, size=trials)
    if target_pos is None:
        pos = rand_pos
    else:
        pos = np.broadcast_to(np.asarray(target_pos), (trials,)).astype(int)
    if with_target:
        for t in range(trials):
            z[t, pos[t]:pos[t] + PULSE_LEN] += chirp
    return gain * z, pos


# ---------------------------------------------------------------- float reference
def compress_float(z, chirp):
    """Exact linear correlation with the chirp (one big FFT). Works on (..., L) arrays."""
    L = z.shape[-1]
    m = 1 << int(np.ceil(np.log2(L + len(chirp))))
    H = np.conj(np.fft.fft(chirp, m))
    return np.fft.ifft(np.fft.fft(z, m, axis=-1) * H, axis=-1)[..., :L]


# ---------------------------------------------------------------- CFAR helpers
def valid_len(L):
    """Only the first L-127 compressed outputs see the whole chirp; the last 127 are partial
    overlap (a finite-stream flush artifact, noise level falls off), so CFAR skips them."""
    return L - PULSE_LEN + 1


def train_sum(p, guard=GUARD, train=TRAIN):
    """Sum of the 2*train training cells around each sample (guard cells skipped).

    Returns (indices, sums); valid for i in [guard+train, valid_len-guard-train), so every
    training cell lies in the full-overlap region.
    Works on any dtype (float or int) and on (..., L) arrays.
    """
    L = valid_len(p.shape[-1])
    c = np.concatenate([np.zeros(p.shape[:-1] + (1,), dtype=p.dtype), np.cumsum(p, axis=-1)], axis=-1)
    i = np.arange(guard + train, L - guard - train)
    lag = c[..., i - guard] - c[..., i - guard - train]
    lead = c[..., i + guard + train + 1] - c[..., i + guard + 1]
    return i, lag + lead


def cfar_float(power, alpha=None, guard=GUARD, train=TRAIN):
    """Floating-point CA-CFAR. Returns (flags, threshold)."""
    alpha = ALPHA if alpha is None else alpha
    i, s = train_sum(power, guard, train)
    thr = np.full(power.shape, np.nan)
    thr[..., i] = alpha * s
    flags = np.zeros(power.shape, dtype=bool)
    flags[..., i] = power[..., i] > thr[..., i]
    return flags, thr


def peak_reports(flags, p, half=PEAK_HALF):
    """Keep a flagged sample only if it is the largest within +/- half samples."""
    m = p.copy()
    for s in range(1, half + 1):
        m[..., s:] = np.maximum(m[..., s:], p[..., :-s])
        m[..., :-s] = np.maximum(m[..., :-s], p[..., s:])
    return flags & (p >= m)


def detected(flags, pos, tol=1):
    """True per trial if any flag lands within +/- tol of the true target position."""
    flags = np.atleast_2d(flags)
    pos = np.atleast_1d(pos)
    idx = pos[:, None] + np.arange(-tol, tol + 1)[None, :]
    return flags[np.arange(flags.shape[0])[:, None], idx].any(axis=1)


def peak_to_floor_db(mag, pos, excl=20):
    """Peak (within +/-1 of true position) over the RMS noise floor, in dB. mag shape (T, L)."""
    mag = np.atleast_2d(mag)
    pos = np.atleast_1d(pos)
    valid = mag.shape[-1] - PULSE_LEN + 1          # region with full overlap
    out = np.zeros(mag.shape[0])
    for t in range(mag.shape[0]):
        m = mag[t, :valid].astype(float)
        mask = np.ones(valid, dtype=bool)
        mask[max(0, pos[t] - excl):pos[t] + excl + 1] = False
        floor = np.sqrt(np.mean(m[mask] ** 2))
        peak = m[pos[t] - 1:pos[t] + 2].max()
        out[t] = 20 * np.log10(peak / floor)
    return out
