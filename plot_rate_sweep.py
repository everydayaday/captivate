#!/usr/bin/env python3
"""
태블릿 속도 시험(stent_ber.py --bps 여러 개) 결과를 스캔률별로 모아 그린다.

   ~/.venvs/galvanic/bin/python plot_rate_sweep.py rateE51 rateE64 rateE85        # stent_ber 의 --label 들 (--labels: 점마다 숫자)
   -> data/rate_sweep_<라벨들>.png, 같은 이름 .csv

(a) 태블릿 FSK 속도 [bps] 에 따른 BER, 스캔률별 곡선
(b) 같은 점을 '비트당 측정 횟수' (= 스캔률 / 속도) 기준으로 - 곡선들이 겹치면 한계가 이것 하나로 정해진다는 뜻
BER 은 블라인드 판정(수신 쪽 정보만으로 동기·판정). 옅은 띠 = 95 % Wilson 신뢰구간 (막대는 추세를 가려서 띠로).
오류 0 이면 속 빈 점을 0.5/N 에 둔다. 점마다 오류/비트 숫자를 그림에 쓰려면 --labels.
"""
import csv, glob, json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
matplotlib.rcParams["font.family"] = ["DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


def wilson(e, n, z=1.96):
    if n == 0:
        return (np.nan, np.nan)
    p = e / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def load(label):
    rows = []
    for d in sorted(glob.glob(os.path.join(HERE, "data", f"stentber_{label}_*"))):
        f = os.path.join(d, "result.json")
        if not os.path.isfile(f):
            continue
        r = json.load(open(f))
        if r.get("missing", 0) > 0.05 * r["scan_hz"] * 20:     # USB 가 끊겨 표본이 많이 빠진 런은 뺀다
            print(f"  뺌 (표본 {r['missing']} 개 빠짐): {os.path.basename(d)}")
            continue
        b = r["blind"]
        rows.append(dict(label=label, bps=r["bps"], psk=r["psk_baud"], scan_hz=r["scan_hz"], k=r["samples_per_bit"],
                         f1_hz=r["carrier_hz"], f0_hz=r.get("zero_hz"),
                         errors=b["errors"] if b["bits"] else None, bits=b["bits"], feature=r["feature"],
                         genie_err=r["genie"][r["feature"]]["errors"], genie_bits=r["genie"][r["feature"]]["bits"]))
    return rows


SHOW_LABELS = False


def main():
    global SHOW_LABELS
    args = sys.argv[1:]
    SHOW_LABELS = "--labels" in args
    labels = [x for x in args if not x.startswith("--")] or ["rateE51", "rateE64", "rateE85"]
    rows = [r for lb in labels for r in load(lb)]
    if not rows:
        sys.exit("결과가 없다")
    out = os.path.join(HERE, "data", "rate_sweep_" + "_".join(labels))
    with open(out + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    fmt = FuncFormatter(lambda v, _: f"{v:g}")
    fig, ax = plt.subplots(1, 2, figsize=(14, 5.2))
    cols = plt.get_cmap("tab10")
    YMIN = 5e-4
    for i, lb in enumerate(labels):
        rr = sorted([r for r in rows if r["label"] == lb], key=lambda r: r["bps"])
        if not rr:
            continue
        fs = np.median([r["scan_hz"] for r in rr])
        f1 = np.median([r["f1_hz"] for r in rr]); f0s = [r["f0_hz"] for r in rr if r["f0_hz"]]
        lab = f"scan {fs:.0f} Hz  (1: {f1 / 1e3:.2f} kHz" + (f", 0: {np.median(f0s) / 1e3:.2f} kHz)" if f0s else ", 0: off)")
        for axi, key in ((0, "bps"), (1, "k")):
            pts = []
            for r in rr:
                n = r["bits"] or r["genie_bits"]
                e = r["errors"] if r["errors"] is not None else n // 2      # 동기 못 잡음 = 찍기 수준으로 표시
                l, h = wilson(e, n)
                pts.append((r[key], e / n if e else 0.5 / n, max(l, YMIN), h, e == 0, e, n))
            pts.sort(key=lambda p: p[0])
            x = np.array([p[0] for p in pts]); y = np.array([p[1] for p in pts])
            a = ax[axi]
            # 신뢰구간은 옅은 띠로 (막대는 추세를 가렸다 - 2026-10-15 사용자 의견)
            a.fill_between(x, [p[2] for p in pts], [p[3] for p in pts], color=cols(i), alpha=0.12, lw=0)
            a.plot(x, y, "-", lw=2, color=cols(i), label=lab)
            zero = np.array([p[4] for p in pts])
            a.plot(x[~zero], y[~zero], "o", ms=6, color=cols(i))
            a.plot(x[zero], y[zero], "o", ms=6, mfc="white", mec=cols(i), mew=1.5)
            if SHOW_LABELS:
                for p in pts:
                    a.annotate(f"{p[5]}/{p[6]}", (p[0], p[1]), textcoords="offset points", xytext=(4, 5), fontsize=7, color=cols(i))
    for a in ax:
        a.set_yscale("log"); a.set_ylim(YMIN, 0.2); a.axhline(1e-2, color="0.5", ls="--", lw=0.8)
        a.text(a.get_xlim()[0], 1.1e-2, " BER 1%", fontsize=8, color="0.4", va="bottom")
        a.grid(alpha=0.3, which="both"); a.yaxis.set_major_formatter(fmt); a.yaxis.set_minor_formatter(NullFormatter())
        a.set_ylabel("Tablet BER (blind decoding)"); a.legend(fontsize=9, loc="upper left")
    ax[0].set_xlabel("Tablet FSK bit rate [bps]")
    ax[0].set_title("(a) BER vs. tablet bit rate", fontsize=10, loc="left")
    ax[1].set_xlabel("Touch scans per bit (= scan rate / bit rate)"); ax[1].invert_xaxis()
    ax[1].set_title("(b) Same data vs. scans per bit (curves collapse onto one trend)", fontsize=10, loc="left")
    fig.text(0.01, 0.005, "Shaded band = 95% Wilson confidence interval. Open marker = zero errors (plotted at 0.5/N). "
             "Error/bit counts per point are in the matching .csv.",
             fontsize=8, color="0.35")
    psk = sorted({r["psk"] for r in rows})
    fig.suptitle(f"Stent (on S1) -> touchscreen: FSK (bit 1 at the touch sensor's frequency, bit 0 = +3 kHz) "
                 f"with DBPSK {', '.join(f'{p:g}' for p in psk)} baud overlay; touch charge-transfer frequency set to 168 kHz",
                 fontsize=11)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out + ".png", dpi=110)
    print(out + ".png")
    print(f"{'라벨':8s} {'스캔Hz':>6s} {'bps':>7s} {'비트당':>6s}  BER(블라인드)       BER(시각 알려 줌)")
    for r in sorted(rows, key=lambda r: (r["label"], r["bps"])):
        e = r["errors"]
        print(f"{r['label']:8s} {r['scan_hz']:6.1f} {r['bps']:7.2f} {r['k']:6.2f}  "
              + (f"{e:4d}/{r['bits']:<4d} = {e / r['bits']:.2e}" if e is not None else "동기 못 잡음        ")
              + f"   {r['genie_err']}/{r['genie_bits']}")


if __name__ == "__main__":
    main()
