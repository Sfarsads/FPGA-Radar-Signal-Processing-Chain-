
from dataclasses import dataclass

import numpy as np

import radar_common as rc


@dataclass
class Cfg:
    in_bits: int = rc.IN_BITS          # input I/Q word (Q1.15)
    data_bits: int = 18                # internal datapath word (Q1.(data_bits-1)); 18 = one MAX 10 multiplier
    tw_bits: int = 18                  # twiddle ROM word
    ref_bits: int = 18                 # stored chirp-spectrum word
    fwd_shifts: tuple = (1, 1, 1, 1, 0, 0, 0, 0)   # divide-by-2 at these forward FFT stages (1 = scale)
    inv_shifts: tuple = (1, 1, 1, 1, 0, 0, 0, 0)   # same for the inverse FFT
    mag2_shift: int = None             # None -> auto (keeps ~26 fractional bits in |.|^2)
    alpha_frac: int = 12               # fractional bits of the CFAR threshold multiplier
    alpha: float = rc.ALPHA            # CFAR multiplier on the training-cell sum
    guard: int = rc.GUARD
    train: int = rc.TRAIN
    peak_half: int = rc.PEAK_HALF

    @property
    def mag2_sh(self):
        if self.mag2_shift is not None:
            return self.mag2_shift
        return max(0, 2 * (self.data_bits - 1) - 26)


# ------------------------------------------------------------------ integer helpers
def rshift_round(x, s):
    """Arithmetic shift right by s with round-half-up (s <= 0 shifts left)."""
    if s <= 0:
        return x << (-s)
    return (x + (1 << (s - 1))) >> s


def sat(x, bits, stats):
    """Saturate to a signed `bits`-bit word; count how many values were clipped."""
    hi = (1 << (bits - 1)) - 1
    lo = -(1 << (bits - 1))
    stats["sat"] += int(np.count_nonzero((x > hi) | (x < lo)))
    return np.clip(x, lo, hi)


def resize_q(x, from_bits, to_bits, stats):
    """Change the word length of a Q1.(bits-1) value (pad with zeros or round + saturate)."""
    if to_bits >= from_bits:
        return x << (to_bits - from_bits)
    return sat(rshift_round(x, from_bits - to_bits), to_bits, stats)


def bitrev(n):
    bits = int(np.log2(n))
    idx = np.arange(n)
    rev = np.zeros(n, dtype=np.int64)
    for b in range(bits):
        rev |= ((idx >> b) & 1) << (bits - 1 - b)
    return rev


# ------------------------------------------------------------------ ROM contents
def twiddles(cfg):
    """Twiddle ROM: W^k = exp(-j*2*pi*k/N) for k = 0..N/2-1, Q1.(tw_bits-1)."""
    k = np.arange(rc.N_FFT // 2)
    w = np.exp(-2j * np.pi * k / rc.N_FFT)
    s = 1 << (cfg.tw_bits - 1)
    wr = np.clip(np.round(w.real * s), -s, s - 1).astype(np.int64)
    wi = np.clip(np.round(w.imag * s), -s, s - 1).astype(np.int64)
    return wr, wi


def make_reference(chirp_i, chirp_q, cfg):
    """Chirp-spectrum ROM: conj(FFT(chirp)) scaled down by 2^k so it fits, Q1.(ref_bits-1).

    Returns (H_real, H_imag, k). k is a design constant the RTL needs to know.
    """
    c = (chirp_i + 1j * chirp_q) / float(1 << (cfg.in_bits - 1))
    H = np.conj(np.fft.fft(c, rc.N_FFT))
    peak = max(np.abs(H.real).max(), np.abs(H.imag).max())
    k = max(0, int(np.ceil(np.log2(peak * (1 + 1e-9)))))
    Hn = H / 2.0 ** k
    s = 1 << (cfg.ref_bits - 1)
    hr = np.clip(np.round(Hn.real * s), -s, s - 1).astype(np.int64)
    hi = np.clip(np.round(Hn.imag * s), -s, s - 1).astype(np.int64)
    return hr, hi, k


# ------------------------------------------------------------------ FFT
def fft_fixed(xr, xi, shifts, cfg, stats, twr, twi):
    """Radix-2 DIT FFT on int64 arrays of shape (batch, N) with per-stage scaling."""
    n = xr.shape[-1]
    rev = bitrev(n)
    xr = xr[:, rev]
    xi = xi[:, rev]
    for s, sh in enumerate(shifts):
        half = 1 << s
        groups = n // (half * 2)
        wr = twr[np.arange(half) * groups]
        wi = twi[np.arange(half) * groups]
        r = xr.reshape(-1, groups, 2, half)
        i = xi.reshape(-1, groups, 2, half)
        ar, br = r[:, :, 0, :], r[:, :, 1, :]
        ai, bi = i[:, :, 0, :], i[:, :, 1, :]
        tr = sat(rshift_round(br * wr - bi * wi, cfg.tw_bits - 1), cfg.data_bits, stats)
        ti = sat(rshift_round(br * wi + bi * wr, cfg.tw_bits - 1), cfg.data_bits, stats)
        nr = np.stack([ar + tr, ar - tr], axis=2)
        ni = np.stack([ai + ti, ai - ti], axis=2)
        xr = sat(rshift_round(nr, sh), cfg.data_bits, stats).reshape(-1, n)
        xi = sat(rshift_round(ni, sh), cfg.data_bits, stats).reshape(-1, n)
    return xr, xi


# ------------------------------------------------------------------ pulse compression
def compress_fixed(in_i, in_q, chirp_i, chirp_q, cfg=None):
    
    cfg = cfg or Cfg()
    stats = {"sat": 0}
    lead = in_i.shape[:-1]
    L = in_i.shape[-1]
    xr = resize_q(in_i.reshape(-1, L).astype(np.int64), cfg.in_bits, cfg.data_bits, stats)
    xi = resize_q(in_q.reshape(-1, L).astype(np.int64), cfg.in_bits, cfg.data_bits, stats)
    T = xr.shape[0]

    # cut into overlapping 256-sample blocks (hop = 128)
    nb = -(-L // rc.HOP)
    pad_r = np.zeros((T, nb * rc.HOP + rc.PULSE_LEN), dtype=np.int64)
    pad_i = np.zeros_like(pad_r)
    pad_r[:, :L] = xr
    pad_i[:, :L] = xi
    idx = (np.arange(nb) * rc.HOP)[:, None] + np.arange(rc.N_FFT)[None, :]
    br = pad_r[:, idx].reshape(-1, rc.N_FFT)
    bi = pad_i[:, idx].reshape(-1, rc.N_FFT)

    twr, twi = twiddles(cfg)
    hr, hi, k = make_reference(chirp_i, chirp_q, cfg)

    # forward FFT -> multiply by conj(chirp spectrum) -> inverse FFT
    fr, fi = fft_fixed(br, bi, cfg.fwd_shifts, cfg, stats, twr, twi)
    yr = sat(rshift_round(fr * hr - fi * hi, cfg.ref_bits - 1), cfg.data_bits, stats)
    yi = sat(rshift_round(fr * hi + fi * hr, cfg.ref_bits - 1), cfg.data_bits, stats)
    zr, zi = fft_fixed(yr, sat(-yi, cfg.data_bits, stats), cfg.inv_shifts, cfg, stats, twr, twi)
    zi = sat(-zi, cfg.data_bits, stats)

    # keep the clean first half of every block and stitch them together
    out_r = zr[:, :rc.HOP].reshape(T, nb * rc.HOP)[:, :L].reshape(lead + (L,))
    out_i = zi[:, :rc.HOP].reshape(T, nb * rc.HOP)[:, :L].reshape(lead + (L,))
    gain = rc.N_FFT * 2.0 ** (-(sum(cfg.fwd_shifts) + k + sum(cfg.inv_shifts)))
    return out_r, out_i, {"sat": stats["sat"], "k": k, "gain": gain}


def to_float(out_r, out_i, cfg, info):
    """Fixed-point compressed output -> complex float on the same scale as the float reference."""
    return (out_r + 1j * out_i) / float(1 << (cfg.data_bits - 1)) / info["gain"]


# ------------------------------------------------------------------ CFAR
def cfar_fixed(mag2, cfg):
    """Integer CA-CFAR on |.|^2. Returns (flags, threshold, alpha_q)."""
    alpha_q = int(round(cfg.alpha * (1 << cfg.alpha_frac)))
    i, s = rc.train_sum(mag2, cfg.guard, cfg.train)
    thr = np.zeros(mag2.shape, dtype=np.int64)
    thr[..., i] = rshift_round(s * alpha_q, cfg.alpha_frac)
    flags = np.zeros(mag2.shape, dtype=bool)
    flags[..., i] = mag2[..., i] > thr[..., i]
    return flags, thr, alpha_q


def run_pipeline(in_i, in_q, chirp_i, chirp_q, cfg=None):
    """Whole golden model: int streams (..., L) in, everything the RTL should produce out."""
    cfg = cfg or Cfg()
    out_i, out_q, info = compress_fixed(in_i, in_q, chirp_i, chirp_q, cfg)
    mag2 = rshift_round(out_i ** 2 + out_q ** 2, cfg.mag2_sh)
    flags, thr, alpha_q = cfar_fixed(mag2, cfg)
    reports = rc.peak_reports(flags, mag2, cfg.peak_half)
    info["alpha_q"] = alpha_q
    return {"out_i": out_i, "out_q": out_q, "mag2": mag2, "thr": thr,
            "flags": flags, "reports": reports, "info": info, "cfg": cfg}


# ------------------------------------------------------------------ self-test
def selftest():
    print("== golden model self-test ==")

    # 1. FFT vs numpy at very high precision (should agree to ~1e-6)
    hi = Cfg(data_bits=28, tw_bits=28, fwd_shifts=(1,) * 8)
    rng = np.random.default_rng(1)
    x = (rng.uniform(-0.5, 0.5, (4, 256)) + 1j * rng.uniform(-0.5, 0.5, (4, 256)))
    xr = np.round(x.real * (1 << 27)).astype(np.int64)
    xi = np.round(x.imag * (1 << 27)).astype(np.int64)
    twr, twi = twiddles(hi)
    st = {"sat": 0}
    fr, fi = fft_fixed(xr, xi, hi.fwd_shifts, hi, st, twr, twi)
    got = (fr + 1j * fi) / (1 << 27)
    want = np.fft.fft(x, axis=-1) / 256
    print(f"FFT vs numpy (28-bit): max error {np.max(np.abs(got - want)):.2e}   saturations: {st['sat']}")

    # 2. whole compression vs the float reference at the default word lengths
    cfg = Cfg()
    z, pos = rc.make_streams(20, snr_db=-5, seed=7)
    ii, qq = rc.quantize(z)
    ci, cq = rc.chirp_int()
    res = run_pipeline(ii, qq, ci, cq, cfg)
    ref = rc.compress_float((ii + 1j * qq) / 32768.0, (ci + 1j * cq) / 32768.0)
    got = to_float(res["out_i"], res["out_q"], cfg, res["info"])
    sqnr = 10 * np.log10(np.sum(np.abs(ref) ** 2) / np.sum(np.abs(got - ref) ** 2))
    print(f"compression vs float ({cfg.data_bits}-bit): SQNR {sqnr:.1f} dB   saturations: {res['info']['sat']}")
    print(f"reference scale k = {res['info']['k']}, total gain = {res['info']['gain']}, alpha_q = {res['info']['alpha_q']}")
    hit = rc.detected(res["flags"], pos).mean()
    print(f"target detected in {hit * 100:.0f}% of 20 test streams")


if __name__ == "__main__":
    selftest()
