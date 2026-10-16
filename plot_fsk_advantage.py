#!/usr/bin/env python3
"""
FSK 가 OOK 보다 실제로 나은 점을 한 장으로 (2026-10-15).

   ~/.venvs/galvanic/bin/python plot_fsk_advantage.py \
       ../transceiver/data/eplus_keying_<시각>.npz adv_ook adv_fsk
   -> data/fsk_advantage.png

(a) 보낸 신호의 포락선 (TeensyTRX 가 자기 E+ 를 500 kS/s 로 녹음, 1 ms 마다 150-190 kHz 대역 세기, 켜졌을 때 = 1)
(b) 포락선 스펙트럼: 켜졌을 때 진폭 대비 잔물결 [dB]. 10-100 Hz = 심장 기기(페이스메이커·ICD) 감지 대역
(c) 같은 송신에 얹은 DBPSK 100 baud 심볼이 E+ 에서 맞게 풀린 비율 (OOK 는 '0' 동안 반송파가 꺼져 심볼이 사라짐)
(d) 터치스크린(태블릿) BER - 같은 송신, 블라인드 판정 (stent_ber.py --label adv_ook / adv_fsk)
"""
import csv, glob, json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(os.path.dirname(HERE), "transceiver"), os.path.join(os.path.dirname(HERE), "dds_signal_gen")]
import cap_ook_tx as TX
from ook_rx import prbs
matplotlib.rcParams["font.family"] = ["DejaVu Sans"]
COL = {"ook": "tab:red", "fsk": "tab:blue"}
CARDIAC = (10.0, 100.0)


def wilson(e, n, z=1.96):
    p = e / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def envelope(x, sps, f_lo=150e3, f_hi=190e3, ms=1):
    """ms 밀리초 조각마다 f_lo-f_hi 대역 세기 (반송파 진폭에 비례)"""
    B = int(sps * ms / 1000); nb = len(x) // B
    X = np.abs(np.fft.rfft(x[:nb * B].reshape(nb, B) * np.hanning(B), axis=1))
    fr = np.fft.rfftfreq(B, 1 / sps)
    band = (fr > f_lo) & (fr < f_hi)
    return np.sqrt((X[:, band] ** 2).sum(axis=1))


def env_spectrum(e_norm, fs=1000.0):
    """정규화 포락선(켜짐 = 1)의 진폭 스펙트럼 [dB re 켜짐 진폭]: 진폭 a 인 사인 잔물결이면 20 log10(a)"""
    y = e_norm - e_norm.mean()
    w = np.hanning(len(y))
    Y = np.abs(np.fft.rfft(y * w)) * 2 / w.sum()
    f = np.fft.rfftfreq(len(y), 1 / fs)
    Yf = np.fft.rfft(y)                                     # 대역 rms 는 창 없이
    Yf[(f < CARDIAC[0]) | (f > CARDIAC[1])] = 0
    band_rms = np.sqrt(np.mean(np.fft.irfft(Yf, len(y)) ** 2))
    return f, 20 * np.log10(Y + 1e-9), 20 * np.log10(band_rms + 1e-9)


def dbpsk_delivery(x, sps, meta):
    """E+ 에서 DBPSK 심볼을 풀어 맞게 전달된 비율 (이웃 두 심볼 모두 반송파가 있고 위상차가 PRBS9 와 맞음)"""
    plan = meta["plan"]; S = meta["sym_us"] * sps // 1000000
    ns = min(len(x) // S - 1, 800)
    bits = TX.frame_bits(plan["payload"], plan["prbs"])[0]
    per = plan["bit_us"] // meta["sym_us"]
    f1, f0 = plan["carrier_hz"], plan.get("zero_hz")
    zk = []
    for k in range(ns):
        b = bits[(k // per) % len(bits)]
        f = f1 if b else (f0 or f1)
        a_, b_ = k * S + S // 10, (k + 1) * S - S // 10
        tt = (np.arange(a_, b_) - k * S) / sps
        zk.append(np.sum(x[a_:b_] * np.exp(-2j * np.pi * f * tt)))
    zk = np.array(zk); mag = np.abs(zk)
    on = mag > 0.3 * np.median(mag[mag > np.percentile(mag, 60)])
    dec = (np.abs(np.angle(zk[1:] / zk[:-1])) > np.pi / 2).astype(int)
    want = np.array(prbs(ns, 9))[1:]
    good = on[1:] & on[:-1] & (dec == want)
    return int(good.sum()), ns - 1, float(on.mean())


def load_ber(label):
    out = []
    for d in sorted(glob.glob(os.path.join(HERE, "data", f"stentber_{label}_*"))):
        f = os.path.join(d, "result.json")
        if os.path.isfile(f):
            r = json.load(open(f))
            out.append(r)
    return out


def main():
    npz = sys.argv[1]
    labels = sys.argv[2:] or ["adv_ook", "adv_fsk"]
    z = np.load(npz)
    fig = plt.figure(figsize=(14, 9.5))
    gs = fig.add_gridspec(2, 2, hspace=0.38, wspace=0.22)
    a_env, a_spec = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
    a_psk, a_ber = fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])
    summary = {}
    for k in ("ook", "fsk"):
        m = json.loads(str(z[f"{k}_psk0_meta"]))
        x = z[f"{k}_psk0_codes"].astype(float) * 3.3 / 16384
        x = x[m["t0"]:]; x -= x.mean()
        e = envelope(x, m["sps"])[:7000]
        on_level = np.median(e[e > 0.5 * e.max()])
        en = e / on_level
        t = np.arange(len(en)) * 1e-3
        a_env.plot(t[:3000], en[:3000], lw=1.2, color=COL[k], label=k.upper())
        f, Y, band_db = env_spectrum(en)
        sel = (f >= 0.5) & (f <= 300)
        a_spec.plot(f[sel], Y[sel], lw=1.2, color=COL[k], label=f"{k.upper()}: 10-100 Hz ripple {band_db:+.0f} dB re carrier")
        summary[k] = dict(band_db=band_db, ripple_pct=float(en.std() * 100))
        mp = json.loads(str(z[f"{k}_psk100_meta"]))
        xp = z[f"{k}_psk100_codes"].astype(float) * 3.3 / 16384
        xp = xp[mp["t0"]:]; xp -= xp.mean()
        g, n, frac_on = dbpsk_delivery(xp, mp["sps"], mp)
        summary[k].update(psk_good=g, psk_n=n)
    a_env.set_xlabel("time from first bit [s]"); a_env.set_ylabel("carrier amplitude at stent (on = 1)")
    a_env.set_ylim(-0.1, 1.3); a_env.legend(fontsize=9, loc="upper right")
    a_env.set_title("(a) Transmitted envelope at the stent electrode (10 bps)", fontsize=10, loc="left")
    a_spec.axvspan(*CARDIAC, color="0.85", lw=0)
    a_spec.text(np.sqrt(CARDIAC[0] * CARDIAC[1]), 2, "cardiac sensing band", ha="center", va="top", fontsize=8, color="0.3")
    a_spec.set_xscale("log"); a_spec.set_ylim(-110, 5)
    a_spec.set_xlabel("envelope frequency [Hz]"); a_spec.set_ylabel("envelope ripple [dB re carrier amplitude]")
    a_spec.legend(fontsize=8, loc="lower left")
    a_spec.set_title("(b) Envelope spectrum: what an AM-sensitive implant would see", fontsize=10, loc="left")
    ks = ["ook", "fsk"]
    vals = [100 * summary[k]["psk_good"] / summary[k]["psk_n"] for k in ks]
    bars = a_psk.bar([k.upper() for k in ks], vals, color=[COL[k] for k in ks], width=0.55)
    for b_, k in zip(bars, ks):
        a_psk.text(b_.get_x() + b_.get_width() / 2, b_.get_height() + 2,
                   f"{summary[k]['psk_good']}/{summary[k]['psk_n']}", ha="center", fontsize=9)
    a_psk.set_ylim(0, 115); a_psk.set_ylabel("DBPSK symbols delivered [%]")
    a_psk.set_title("(c) In-body DBPSK (100 baud) carried by the same transmission", fontsize=10, loc="left")
    # (d) 태블릿 BER
    for k, lb in zip(ks, labels):
        rr = sorted(load_ber(lb), key=lambda r: r["bps"])
        if not rr:
            continue
        b = np.array([r["bps"] for r in rr])
        E = [r["blind"]["errors"] if r["blind"]["bits"] else None for r in rr]
        N = [r["blind"]["bits"] or r["genie"][r["feature"]]["bits"] for r in rr]
        y, lo, hi, zero = [], [], [], []
        for e_, n in zip(E, N):
            e_ = e_ if e_ is not None else n // 2
            l, h = wilson(e_, n)
            y.append(e_ / n if e_ else 0.5 / n); lo.append(max(l, 5e-4)); hi.append(h); zero.append(e_ == 0)
        y = np.array(y); zero = np.array(zero)
        a_ber.fill_between(b, lo, hi, color=COL[k], alpha=0.12, lw=0)
        a_ber.plot(b, y, "-", lw=2, color=COL[k], label=k.upper())
        a_ber.plot(b[~zero], y[~zero], "o", ms=7, color=COL[k])
        a_ber.plot(b[zero], y[zero], "o", ms=7, mfc="white", mec=COL[k], mew=1.5)
        summary[k]["ber"] = {f"{r['bps']:g}": (r["blind"]["errors"], r["blind"]["bits"]) for r in rr}
    a_ber.set_yscale("log"); a_ber.set_ylim(5e-4, 0.2); a_ber.axhline(1e-2, color="0.5", ls="--", lw=0.8)
    fmt = FuncFormatter(lambda v, _: f"{v:g}")
    a_ber.yaxis.set_major_formatter(fmt); a_ber.yaxis.set_minor_formatter(NullFormatter())
    a_ber.set_xlabel("tablet bit rate [bps]"); a_ber.set_ylabel("tablet BER (blind decoding)")
    a_ber.legend(fontsize=9, loc="upper left")
    tab = "errors / bits\n" + "\n".join(f"{k.upper()} {bp:>3s} bps: {e}/{n}" for k in ks for bp, (e, n) in summary[k].get("ber", {}).items())
    a_ber.set_xticks(sorted({float(bp) for k in ks for bp in summary[k].get("ber", {})}))
    a_ber.text(0.98, 0.97, tab, transform=a_ber.transAxes, ha="right", va="top", fontsize=8, family="monospace",
               bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="0.8"))
    a_ber.set_title("(d) Touchscreen BER with the DBPSK overlay (scan 68 Hz)", fontsize=10, loc="left")
    for a in (a_env, a_spec, a_psk, a_ber):
        a.grid(alpha=0.3, which="both")
    fig.suptitle("Same touchscreen BER, but FSK keeps the carrier on: flat envelope (no cardiac-band AM) and an unbroken in-body DBPSK link",
                 fontsize=11)
    out = os.path.join(HERE, "data", "fsk_advantage.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(out)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
