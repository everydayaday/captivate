#!/usr/bin/env python3
"""
OOK vs FSK 비교 그림 (stent_ber.py --keying ook / fsk 로 같은 조건에서 잰 결과).

   ~/.venvs/galvanic/bin/python plot_ook_fsk.py okfsk_ook okfsk_fsk        # 두 --label
   -> data/ook_vs_fsk_<라벨들>.png

윗줄 = 보낸 신호 (송신 시간표 그대로: OOK 는 0 에서 반송파를 끄고, FSK 는 0 에서 +3 kHz 로 옮겨 계속 켠다)
가운데 = 같은 비트 열을 받는 동안 터치스크린 S1 의 count (실측)
아랫줄 = BER (블라인드 판정, 옅은 띠 = 95 % Wilson 구간)과 켬/끔 흔들림 대비 [dB]
"""
import csv, glob, json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(os.path.dirname(HERE), "transceiver")]
import cap_ook_tx as TX
matplotlib.rcParams["font.family"] = ["DejaVu Sans"]
COL = {"ook": "tab:red", "fsk": "tab:blue"}
NAME = {"ook": "OOK (bit 0 = carrier off)", "fsk": "FSK (bit 0 = carrier +3 kHz, always on)"}


def wilson(e, n, z=1.96):
    p = e / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def load(label):
    out = []
    for d in sorted(glob.glob(os.path.join(HERE, "data", f"stentber_{label}_*"))):
        if not os.path.isfile(os.path.join(d, "result.json")):
            continue
        r = json.load(open(os.path.join(d, "result.json")))
        m = json.load(open(os.path.join(d, "meta.json")))
        rows = list(csv.DictReader(open(os.path.join(d, "capture.csv"))))
        el = m["electrode"]
        t = np.array([float(x["t_unix"]) for x in rows]); c = np.array([float(x[f"{el}_count"]) for x in rows])
        out.append(dict(r=r, m=m, t=t, c=c, keying="fsk" if r.get("zero_hz") else "ook", dir=d))
    return out


def main():
    labels = sys.argv[1:] or ["okfsk_ook", "okfsk_fsk"]
    runs = [x for lb in labels for x in load(lb)]
    if not runs:
        sys.exit("결과가 없다")
    keys = [k for k in ("ook", "fsk") if any(x["keying"] == k for x in runs)]
    fig = plt.figure(figsize=(14, 10.5))
    gs = fig.add_gridspec(3, 2, height_ratios=[0.8, 1.1, 1.0], hspace=0.45, wspace=0.18)
    bps_show = min(x["r"]["bps"] for x in runs)
    win_bits = 24
    for j, k in enumerate(keys):
        x = next(x for x in runs if x["keying"] == k and abs(x["r"]["bps"] - bps_show) < 1e-6)
        r, m = x["r"], x["m"]
        plan = m["plan"]; Tb = plan["bit_us"] / 1e6
        bits = TX.frame_bits(plan["payload"], plan["prbs"])[0]
        j0 = len(TX.PREAMBLE) + len(TX.SFD)                     # 첫 프레임 페이로드 시작부터
        tb0 = m["t_first_unix"] + j0 * Tb
        seg = np.array(bits[j0:j0 + win_bits])
        tt = np.arange(win_bits + 1) * Tb
        # 윗줄: 보낸 신호 (시간표)
        a = fig.add_subplot(gs[0, j])
        f1, f0 = r["carrier_hz"] / 1e3, (r["zero_hz"] or 0) / 1e3
        if k == "ook":
            a.fill_between(tt[:-1], 0, seg, step="post", color=COL[k], alpha=0.35, lw=0)
            a.step(tt[:-1], seg, where="post", color=COL[k], lw=1.5)
            a.set_ylim(-0.15, 1.35); a.set_yticks([0, 1]); a.set_yticklabels(["off", "on"])
            a.set_ylabel("carrier amplitude")
            a.text(0.01, 0.97, f"1 = {f1:.2f} kHz on,  0 = off", transform=a.transAxes, va="top", fontsize=9)
        else:
            a.fill_between(tt[:-1], -0.15, 1.15, color=COL[k], alpha=0.10, lw=0)
            a.step(tt[:-1], seg, where="post", color=COL[k], lw=1.5)
            a.set_ylim(-0.15, 1.35); a.set_yticks([0, 1])
            a.set_yticklabels([f"{f0:.2f} kHz", f"{f1:.2f} kHz"])
            a.set_ylabel("carrier frequency")
            a.text(0.01, 0.97, "amplitude constant (shaded): carrier never turns off", transform=a.transAxes, va="top", fontsize=9)
        a.set_xlim(0, tt[-1]); a.set_xlabel("time from payload start [s]")
        a.set_title(f"Sent: {NAME[k]}", fontsize=10, loc="left")
        # 가운데: 터치스크린이 받은 count
        a = fig.add_subplot(gs[1, j])
        msk = (x["t"] >= tb0 - 0.2) & (x["t"] <= tb0 + tt[-1] + 0.2)
        for i_, b in enumerate(seg):
            if b:
                a.axvspan(tt[i_], tt[i_ + 1], color="0.85", lw=0)
        a.plot(x["t"][msk] - tb0, x["c"][msk], ".-", ms=3, lw=0.8, color=COL[k])
        a.set_xlim(0, tt[-1]); a.set_xlabel("time from payload start [s]"); a.set_ylabel(f"{m['electrode']} count")
        a.set_title(f"Received by touchscreen ({m['electrode']}), {r['bps']:g} bps, scan {r['scan_hz']:.0f} Hz "
                    f"(grey = bit 1 sent)", fontsize=10, loc="left")
    # 아랫줄: BER 과 대비
    a1 = fig.add_subplot(gs[2, 0]); a2 = fig.add_subplot(gs[2, 1])
    for k in keys:
        rr = sorted([x for x in runs if x["keying"] == k], key=lambda x: x["r"]["bps"])
        b = np.array([x["r"]["bps"] for x in rr])
        E = [x["r"]["blind"]["errors"] if x["r"]["blind"]["bits"] else None for x in rr]
        N = [x["r"]["blind"]["bits"] or x["r"]["genie"][x["r"]["feature"]]["bits"] for x in rr]
        y, lo, hi, zero = [], [], [], []
        for e, n in zip(E, N):
            e = e if e is not None else n // 2
            l, h = wilson(e, n)
            y.append(e / n if e else 0.5 / n); lo.append(max(l, 5e-4)); hi.append(h); zero.append(e == 0)
        y = np.array(y); zero = np.array(zero)
        a1.fill_between(b, lo, hi, color=COL[k], alpha=0.12, lw=0)
        a1.plot(b, y, "-", lw=2, color=COL[k], label=k.upper())
        a1.plot(b[~zero], y[~zero], "o", ms=7, color=COL[k]); a1.plot(b[zero], y[zero], "o", ms=7, mfc="white", mec=COL[k], mew=1.5)
        c = [x["r"]["energy_ratio_db"] for x in rr]
        a2.plot(b, c, "o-", lw=2, ms=7, color=COL[k], label=k.upper())
    a1.set_yscale("log"); a1.set_ylim(5e-4, 0.2); a1.axhline(1e-2, color="0.5", ls="--", lw=0.8)
    fmt = FuncFormatter(lambda v, _: f"{v:g}")
    a1.yaxis.set_major_formatter(fmt); a1.yaxis.set_minor_formatter(NullFormatter())
    a1.set_xlabel("tablet bit rate [bps]"); a1.set_ylabel("tablet BER (blind decoding)")
    a1.set_title("BER (shaded = 95% Wilson interval, open = 0 errors at 0.5/N)", fontsize=10, loc="left")
    a2.set_xlabel("tablet bit rate [bps]"); a2.set_ylabel("count fluctuation, bit 1 vs bit 0 [dB]")
    a2.set_title("Received contrast (neighbour-difference energy, on/off)", fontsize=10, loc="left")
    a2.set_ylim(0, 30)
    tab = "errors / bits (blind)\n" + "\n".join(
        f"{x['keying'].upper():3s} {x['r']['bps']:4.0f} bps: "
        + (f"{x['r']['blind']['errors']}/{x['r']['blind']['bits']}" if x["r"]["blind"]["bits"] else "no sync")
        for x in sorted(runs, key=lambda x: (x["keying"], x["r"]["bps"])))
    a1.text(0.02, 0.97, tab, transform=a1.transAxes, ha="left", va="top", fontsize=8, family="monospace",
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="0.8"))
    for a in (a1, a2):
        a.grid(alpha=0.3, which="both"); a.legend(fontsize=9, loc="upper right"); a.set_xticks(sorted({x["r"]["bps"] for x in runs}))
    fig.suptitle("OOK vs FSK into a touchscreen: stent on S1, touch sensor charge-transfer frequency 168 kHz, "
                 f"scan {np.median([x['r']['scan_hz'] for x in runs]):.0f} Hz, no DBPSK overlay", fontsize=11)
    out = os.path.join(HERE, "data", "ook_vs_fsk_" + "_".join(labels) + ".png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(out)
    for x in sorted(runs, key=lambda x: (x["keying"], x["r"]["bps"])):
        r = x["r"]; b = r["blind"]
        print(f"{x['keying']:4s} {r['bps']:5g} bps  1: {r['carrier_hz'] / 1e3:.3f} kHz  0: "
              f"{(r['zero_hz'] or 0) / 1e3:.3f} kHz  대비 {r['energy_ratio_db']:+.1f} dB  "
              + (f"BER {b['errors']}/{b['bits']}" if b["bits"] else "동기 못 잡음"))


if __name__ == "__main__":
    main()
