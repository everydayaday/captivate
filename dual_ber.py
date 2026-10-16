#!/usr/bin/env python3
"""
두 전극으로 받는 태블릿 BER: FSK 의 '0' 도 태블릿이 듣게 하면 OOK 보다 나은가 (2026-10-15).

   ~/.venvs/galvanic/bin/python dual_ber.py --mode fsk2 --bps 10,16.66667,20,25,33.33333 --label dualF
   ~/.venvs/galvanic/bin/python dual_ber.py --mode ook2 --bps 10,16.66667,20,25,33.33333 --label dualO

전극: S1 (슬라이더, 센서 2) 와 근접 링 P (센서 1, 슬라이더 바로 아래를 둘러쌈). 둘 다 스텐트 신호를 받는다
(2026-10-15: P 를 155.08 kHz 로 맞추고 반송파를 훑자 +32 dB).
  fsk2: S1 은 f_t 168.0 kHz (분주 2, 11/11), P 는 155.08 kHz (12/12). 송신 1 = S1 의 최적점, 0 = P 의 최적점.
        매 비트에 둘 중 하나가 출렁인다 -> 결정 = (S1 흔들림) - (P 흔들림)
  ook2: S1·P 둘 다 168.0 kHz. 송신 1 = 최적점 켬, 0 = 끔 -> 결정 = (S1 흔들림) + (P 흔들림)  (공정한 비교 기준)
같은 녹화에서 S1 하나만 쓴 결과도 같이 낸다 (= 지금까지의 한 전극 수신).
판정·동기는 cap_ook_rx 와 같다 (프리앰블+SFD 상관으로 프레임 격자, 판정 지향 문턱). 특징은 absdev·logenergy 중 나은 것.
결과: data/dual_<label>_<bps>bps_<시각>/ (capture.csv, result.json). 그림은 plot_dual.py.
"""
import argparse, csv, datetime, json, math, os, sys, threading, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, os.path.join(ROOT, "transceiver")]
import captivate_hid as CH
import cap_ook_rx as RX
import cap_ook_tx as TX
import stent_ber as SB

DATADIR = os.path.join(HERE, "data")
P_FD, P_CL, P_TL = 0x97, 0x98, 0x99


class Rec2(threading.Thread):
    """여러 전극의 사이클 패킷을 따로 모은다. keys = {이름: (센서, 사이클, 소자)}"""

    def __init__(self, br, keys):
        super().__init__(daemon=True)
        self.br, self.keys = br, keys
        self.lock, self.stop_ev = threading.Lock(), threading.Event()
        self.data = {k: [] for k in keys}
        self.error = None

    def run(self):
        while not self.stop_ev.is_set():
            try:
                p = self.br.read_packet(50)
            except OSError as e:
                self.error = str(e); print(f"  ! 터치스크린 USB 읽기 오류: {e}"); return
            if p is None or p[0] != 1:
                continue
            for k, (sid, cyc, ei) in self.keys.items():
                if p[1] == sid and p[2] == cyc:
                    d = CH.parse(p)
                    if d["kind"] == "cycle":
                        with self.lock:
                            self.data[k].append((time.time(), d["count"][ei]))

    def snapshot(self):
        with self.lock:
            return {k: (np.array([v[0] for v in L]), np.array([v[1] for v in L], float)) for k, L in self.data.items()}

    def stop(self):
        self.stop_ev.set(); self.join(1.0)


def zscore(v):
    q1, q2, q3 = np.percentile(v, [25, 50, 75])
    return (v - q2) / max(q3 - q1, 1e-6)


def analyse(rec, t_first, plan, a):
    """S1 하나, 두 전극(합 또는 차) 각각 블라인드 BER"""
    payload = TX.frame_bits(plan["payload"], plan["prbs"])[1]
    t1, c1 = rec[a.e1]; t2, c2 = rec[a.e2]
    n1, T1, t01, _ = RX.regularise(t1, a.scan_ms / 1e3)
    n2, T2, t02, _ = RX.regularise(t2, a.scan_ms / 1e3)
    x1, _ = RX.on_grid(n1, c1); x2, _ = RX.on_grid(n2, c2)
    g = t01 + np.arange(len(x1)) * T1                       # S1 격자 시각에 P 를 맞춘다 (같은 스캔에서 온 패킷)
    i2 = np.clip(np.round((g - t02) / T2).astype(int), 0, len(x2) - 1)
    k = plan["bit_us"] / 1e6 / T1
    RX.set_guard(k)
    F1, F2 = RX.features(x1), RX.features(x2)
    feats = {}
    for nm in ("absdev", "logenergy"):
        feats[f"single_{nm}"] = F1[nm]
        z1, z2 = zscore(F1[nm]), zscore(F2[nm][i2])
        feats[f"dual_{nm}"] = z1 - z2 if a.mode == "fsk2" else z1 + z2
    s0g = (t_first - t01) / T1
    P = plan["frame_bits"] * k
    out = {}
    for name, f in feats.items():
        fr = RX.find_frames(f, k, plan["frame_bits"], payload)
        valid = [d for d in fr["frames"] if d["valid"]]
        bits = sum(d["bits"] for d in valid); errs = sum(d["errors"] for d in valid)
        genie = [RX.decode_frame(f, k, s0g + m * P, payload) for m in range(plan["n_frames"])
                 if s0g + m * P + (len(RX.HDR) + len(payload)) * k < len(f)]
        out[name] = dict(n_valid=len(valid), bits=bits, errors=errs, ber=errs / bits if bits else None,
                         genie_errors=sum(d["errors"] for d in genie), genie_bits=sum(d["bits"] for d in genie),
                         q=float(np.mean([d["q"] for d in valid])) if valid else None)
    best = {}
    for kind in ("single", "dual"):
        cand = {n: v for n, v in out.items() if n.startswith(kind)}
        bn = max(cand, key=lambda n: (cand[n]["n_valid"], -(cand[n]["ber"] if cand[n]["ber"] is not None else 1)))
        best[kind] = dict(cand[bn], feature=bn)
    return dict(features=out, best=best, scan_hz=1 / T1, samples_per_bit=k)


def main():
    ap = argparse.ArgumentParser(description="두 전극 태블릿 BER (FSK 두 주파수 vs OOK 같은 주파수)")
    ap.add_argument("--mode", choices=("fsk2", "ook2"), required=True)
    ap.add_argument("--bps", type=lambda s: [float(v) for v in s.split(",")], default=[10, 16.66667, 20, 25, 33.33333])
    ap.add_argument("--e1", default="S1"); ap.add_argument("--e2", default="P")
    ap.add_argument("--L1", type=int, default=11, help="전극 1 충전=전달 길이 (분주 2): f_t = 2.016 MHz/(L+1) (11 -> 168.0 kHz)")
    ap.add_argument("--L2", type=int, default=12, help="fsk2 의 전극 2 길이 (12 -> 155.08 kHz). ook2 면 L1 과 같게 둔다")
    ap.add_argument("--scan-ms", type=int, default=15)
    ap.add_argument("--target-bits", type=int, default=760); ap.add_argument("--max-run-s", type=float, default=90.0)
    ap.add_argument("--lead", type=float, default=3.0); ap.add_argument("--tail", type=float, default=2.0)
    ap.add_argument("--label", default="dual")
    ap.add_argument("--cal-span", type=float, default=0.02); ap.add_argument("--cal-step", type=float, default=100.0)
    ap.add_argument("--cal-dwell", type=float, default=0.2)
    a = ap.parse_args()
    a.payload, a.prbs, a.frames, a.wave = 127, 7, 2, "square"
    a.no_recal = True
    L2 = a.L2 if a.mode == "fsk2" else a.L1
    s1, cy1, ei1 = SB.ELECTRODES[a.e1][:3]; s2, cy2, ei2 = SB.ELECTRODES[a.e2][:3]
    import trx_recip as T
    import serial
    br = CH.Bridge("I2C")
    dev = serial.Serial(T.find_teensy(None), 921600, timeout=0.3, write_timeout=1.0)
    T.check_trx(dev)
    a.sensor_id = s1
    orig, ct_o, tune2_o, c1_o = {}, {}, {}, None
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rows = []
    try:
        orig, _ = RX.setup_board(br, a)                       # 센서 1 쪽 필터·재보정 끔 + 스캔 간격
        for cmd, val, _ in RX.TUNE:                            # 센서 2 쪽도
            tune2_o[cmd] = RX.param(br, cmd, s2); RX.param(br, cmd, s2, value=val)
        c1_o = RX.param(br, 0xC1); RX.param(br, 0xC1, value=0)
        for sid, L in ((s1, a.L1), (s2, L2)):
            ct_o[sid] = [RX.param(br, c, sid) for c in (P_FD, P_CL, P_TL)]
            for c, v in zip((P_FD, P_CL, P_TL), (2, L, L)):
                RX.param(br, c, sid, value=v)
        RX.param(br, CH.P_FORCE_RECAL, 0, value=1)            # 명시적으로 다시 보정 (안 하면 P count 가 멈춰 있었다)
        t0 = time.time()
        while time.time() - t0 < 4.0:
            br.read_packet(20)
        ft1, ft2 = SB.F_OSC / 8 / (a.L1 + 1), SB.F_OSC / 8 / (L2 + 1)
        cal = {}
        for e, ft in ((a.e1, ft1), (a.e2, ft2)) if a.mode == "fsk2" else ((a.e1, ft1),):
            a.electrode = e
            for _ in range(3):
                c = SB.calibrate(br, dev, T, a, ft)
                if c["found"] and np.isfinite(c["peak_hz"]):
                    break
                print(f"  {e} 보정 반응 없음 - 다시"); time.sleep(2)
            cal[e] = c
            print(f"  {e} 보정: 최적 {c['peak_hz'] / 1e3:.3f} kHz (+{c['peak_gain_db']:.1f} dB)")
        f1 = cal[a.e1]["peak_hz"]
        f0 = cal[a.e2]["peak_hz"] if a.mode == "fsk2" else None
        print(f"송신: 1 = {f1 / 1e3:.3f} kHz, 0 = " + (f"{f0 / 1e3:.3f} kHz (FSK)" if f0 else "끔 (OOK)"))
        for bps in a.bps:
            nfr = SB.frames_for(a, bps)
            frame_s = (len(TX.PREAMBLE) + len(TX.SFD) + a.payload + TX.GAP_BITS) / bps
            link = argparse.Namespace(carrier=f1, zero_hz=f0, bps=bps, payload=a.payload, prbs=a.prbs, wave=a.wave,
                                      seconds=(nfr - 0.5) * frame_s)
            fw, plan = TX.build(link)
            T.ask(dev, "off", want=r"# off")
            RX.param(br, CH.P_FORCE_RECAL, 0, value=1)
            t0 = time.time()
            while time.time() - t0 < 1.2:
                br.read_packet(20)
            T.upload(dev, fw, a.wave); T.ask(dev, "psk 0", want=r"# psk")
            rec = Rec2(br, {a.e1: (s1, cy1, ei1), a.e2: (s2, cy2, ei2)}); rec.start()
            time.sleep(a.lead)
            tc = time.time(); T.ask(dev, "sgo 500", want=r"armed in|sgo:"); t_first = tc + 0.5005
            time.sleep(0.5 + plan["total_s"] + a.tail)
            rec.stop(); T.ask(dev, "off", want=r"# off")
            if rec.error:
                print("  USB 오류로 이 점을 건너뜀"); break
            data = rec.snapshot()
            res = analyse(data, t_first, plan, a)
            res.update(mode=a.mode, bps=plan["bps"], f1_hz=f1, f0_hz=f0, plan=plan, calibration=cal, L1=a.L1, L2=L2,
                       t_first_unix=t_first)
            d = os.path.join(DATADIR, f"dual_{a.label}_{bps:g}bps_{stamp}")
            os.makedirs(d, exist_ok=True)
            json.dump(res, open(os.path.join(d, "result.json"), "w"), indent=1, ensure_ascii=False)
            with open(os.path.join(d, "capture.csv"), "w", newline="") as fh:
                w = csv.writer(fh); w.writerow(["electrode", "t_unix", "count"])
                for e, (tt, cc) in data.items():
                    for x, y in zip(tt, cc):
                        w.writerow([e, f"{x:.4f}", int(y)])
            b1, b2 = res["best"]["single"], res["best"]["dual"]
            fmt = lambda b: f"{b['errors']}/{b['bits']}" if b["bits"] else "동기 못 잡음"
            print(f"  {bps:6.2f} bps (비트당 {res['samples_per_bit']:.2f}): {a.e1} 하나 {fmt(b1)} [{b1['feature']}]   "
                  f"두 전극 {fmt(b2)} [{b2['feature']}]")
            rows.append(res)
    finally:
        try:
            T.ask(dev, "off", want=r"# off"); T.ask(dev, "psk 0", want=r"# psk")
        except Exception:
            pass
        dev.close()

        def restore(b):
            for sid, v in ct_o.items():
                for c, x in zip((P_FD, P_CL, P_TL), v):
                    RX.param(b, c, sid, value=x)
            for cmd, v in tune2_o.items():
                if v is not None:
                    RX.param(b, cmd, s2, value=v)
            if c1_o is not None:
                RX.param(b, 0xC1, value=c1_o)
            RX.param(b, CH.P_FORCE_RECAL, 0, value=1)
            RX.restore_board(b, orig)
        try:
            restore(br)
        except OSError:
            print("  터치스크린 USB 를 다시 열어 설정을 되돌린다")
            try:
                br.close()
            except Exception:
                pass
            time.sleep(2); br = CH.Bridge("I2C"); restore(br)
        br.close()


if __name__ == "__main__":
    main()
