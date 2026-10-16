#!/usr/bin/env python3
"""
두 전극 태블릿 BER 그림 (dual_ber.py 결과).

   ~/.venvs/galvanic/bin/python plot_dual.py dual_fsk2 dual_ook2
   -> data/dual_ber.png, data/dual_ber.csv

(a) FSK 를 받는 동안 S1 (bit 1 주파수) 과 P (bit 0 주파수) 의 count - 매 비트에 둘 중 하나가 출렁인다
(b) 태블릿 BER: OOK 한 전극 / OOK 두 전극(같은 주파수, 흔들림 합) / FSK 한 전극 / FSK 두 전극(두 주파수, 흔들림 차)
    블라인드 판정, 옅은 띠 = 95 % Wilson 구간, 속 빈 점 = 오류 0 (0.5/N)
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
STYLE = {("ook2", "single"): ("OOK, one electrode (S1)", "tab:red", "--"),
         ("ook2", "dual"): ("OOK, two electrodes, same frequency (S1 + P)", "tab:red", "-"),
         ("fsk2", "single"): ("FSK, one electrode (S1)", "tab:blue", "--"),
         ("fsk2", "dual"): ("FSK, two electrodes, one per tone (S1 - P)", "tab:blue", "-")}


def wilson(e, n, z=1.96):
    p = e / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def load(label):
    out = []
    for d in sorted(glob.glob(os.path.join(HERE, "data", f"dual_{label}_*"))):
        f = os.path.join(d, "result.json")
        if os.path.isfile(f):
            r = json.load(open(f)); r["dir"] = d
            out.append(r)
    return out


def main():
    labels = sys.argv[1:] or ["dual_fsk2", "dual_ook2"]
    runs = [r for lb in labels for r in load(lb)]
    if not runs:
        sys.exit("결과가 없다")
    fig, ax = plt.subplots(1, 2, figsize=(15, 5.6), gridspec_kw=dict(width_ratios=[1.1, 1]))
    # (a) FSK 예시 파형
    fsk = sorted([r for r in runs if r["mode"] == "fsk2"], key=lambda r: r["bps"])
    a = ax[0]
    if fsk:
        r = fsk[0]
        rows = list(csv.DictReader(open(os.path.join(r["dir"], "capture.csv"))))
        plan = r["plan"]; Tb = plan["bit_us"] / 1e6
        bits = TX.frame_bits(plan["payload"], plan["prbs"])[0]
        j0 = len(TX.PREAMBLE) + len(TX.SFD)
        # 보낸 시각은 결과에 없으니 S1 흔들림이 시작되는 곳 대신 저장된 보정 정보 대신 capture 의 첫 프레임 격자를 쓴다
        t_all = {e: (np.array([float(x["t_unix"]) for x in rows if x["electrode"] == e]),
                     np.array([float(x["count"]) for x in rows if x["electrode"] == e])) for e in ("S1", "P")}
        t_first = r.get("t_first_unix")
        if t_first is None:
            t_first = t_all["S1"][0][0] + 3.5005           # lead 3 s + sgo 0.5 s
        tb0 = t_first + j0 * Tb
        win = 16
        for e, col, off in (("S1", "tab:blue", 0), ("P", "tab:purple", 0)):
            tt, cc = t_all[e]
            m = (tt >= tb0 - 0.1) & (tt <= tb0 + win * Tb + 0.1)
            a.plot(tt[m] - tb0, cc[m] - np.median(cc[m]) + (60 if e == "S1" else -60), ".-", ms=3, lw=0.8, color=col,
                   label=f"{e} (listens to {'bit 1' if e == 'S1' else 'bit 0'} tone)")
        for i in range(win):
            if bits[j0 + i]:
                a.axvspan(i * Tb, (i + 1) * Tb, color="0.88", lw=0)
            a.text((i + 0.5) * Tb, 150, str(bits[j0 + i]), ha="center", fontsize=8, color="0.3")
        a.set_xlim(0, win * Tb); a.set_ylim(-160, 165)
        a.set_yticks([60, -60]); a.set_yticklabels(["S1", "P"])
        a.set_xlabel("time from payload start [s]")
        a.set_title(f"(a) FSK received on two electrodes ({r['bps']:g} bps): every bit makes one of them fluctuate "
                    f"(grey = bit 1)", fontsize=10, loc="left")
        a.legend(fontsize=8, loc="lower right")
    # (b) BER
    a = ax[1]
    table = []
    for (mode, kind), (lab, col, ls) in STYLE.items():
        rr = sorted([r for r in runs if r["mode"] == mode], key=lambda r: r["bps"])
        if not rr:
            continue
        b = np.array([r["bps"] for r in rr])
        y, lo, hi, zero = [], [], [], []
        for r in rr:
            x = r["best"][kind]
            n = x["bits"] or x["genie_bits"]
            e = x["errors"] if x["bits"] else n // 2
            l, h = wilson(e, n)
            y.append(e / n if e else 0.5 / n); lo.append(max(l, 5e-4)); hi.append(h); zero.append(e == 0)
            table.append(dict(mode=mode, receiver=kind, bps=round(r["bps"], 3), samples_per_bit=round(r["samples_per_bit"], 2),
                              errors=x["errors"] if x["bits"] else None, bits=x["bits"], feature=x["feature"]))
        y = np.array(y); zero = np.array(zero)
        a.fill_between(b, lo, hi, color=col, alpha=0.08 if ls == "--" else 0.14, lw=0)
        a.plot(b, y, ls, lw=2 if ls == "-" else 1.4, color=col, label=lab)
        a.plot(b[~zero], y[~zero], "o", ms=6, color=col)
        a.plot(b[zero], y[zero], "o", ms=6, mfc="white", mec=col, mew=1.5)
    a.set_yscale("log"); a.set_ylim(5e-4, 0.3); a.axhline(1e-2, color="0.5", ls=":", lw=0.8)
    fmt = FuncFormatter(lambda v, _: f"{v:g}")
    a.yaxis.set_major_formatter(fmt); a.yaxis.set_minor_formatter(NullFormatter())
    a.set_xlabel("tablet bit rate [bps]"); a.set_ylabel("tablet BER (blind decoding)")
    a.set_xticks(sorted({round(r["bps"], 2) for r in runs}))
    a.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    a.grid(alpha=0.3, which="both"); a.legend(fontsize=8, loc="upper left")
    sc = np.median([r["scan_hz"] for r in runs])
    a.set_title(f"(b) Tablet BER (scan {sc:.0f} Hz). Shaded = 95% Wilson, open = 0 errors", fontsize=10, loc="left")
    f1 = np.median([r["f1_hz"] for r in runs]) / 1e3
    f0s = [r["f0_hz"] for r in runs if r.get("f0_hz")]
    fig.suptitle(f"Letting the tablet hear FSK's '0': bit 1 = {f1:.2f} kHz (S1 tuned to 168.0 kHz), "
                 f"bit 0 = {np.median(f0s) / 1e3:.2f} kHz (proximity ring P tuned to 155.1 kHz)" if f0s else
                 "Two-electrode tablet reception", fontsize=11)
    fig.tight_layout()
    out = os.path.join(HERE, "data", "dual_ber")
    fig.savefig(out + ".png", dpi=110)
    with open(out + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(table[0].keys())); w.writeheader(); w.writerows(table)
    print(out + ".png")
    for t in sorted(table, key=lambda t: (t["bps"], t["mode"], t["receiver"])):
        print(f"{t['bps']:6.2f} bps  {t['mode']} {t['receiver']:6s}  " +
              (f"{t['errors']}/{t['bits']}" if t["bits"] else "동기 못 잡음") + f"  [{t['feature']}]")


if __name__ == "__main__":
    main()
