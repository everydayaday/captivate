#!/usr/bin/env python3
"""
스텐트 -> 터치스크린(CapTIvate 전극 하나, 기본 버튼 1) BER 시험을 명령 하나로. 트랜시버(TeensyTRX)와 CapTIvate 가 같은 노트북에 꽂혀 있을 때.

   ~/.venvs/galvanic/bin/python stent_ber.py --carrier 163000
   ~/.venvs/galvanic/bin/python stent_ber.py --electrode S1 --carrier 1012.5,163e3   # 슬라이더 S1 에 붙였을 때
   ~/.venvs/galvanic/bin/python stent_ber.py --carrier 100e3,150e3,200e3        # 차례로, 끝에 비교표·비교 그림
   ~/.venvs/galvanic/bin/python stent_ber.py --carrier 163e3 --wave sine        # 사인과 비교
   옵션: --bps 5  --frames 2 (반송파마다 프레임 수, 프레임 = 34 s at 5 bps)  --no-live  --label

반송파마다 하는 일
  1) 송신을 끈 채로 버튼 1 기준선을 잡고, --lead 초(기본 4) 동안 송신 없는 잡음을 녹화한다.
  2) TeensyTRX 에 시간표를 올리고 시작한다. 이 프로그램이 직접 보내므로 몇 번째 비트가 언제 나가는지 안다 (USB 지연 ~1 ms).
  3) 보내는 동안 창에 실시간으로: 버튼 1 count 와 보낸 비트(주황 = 1), 비트마다의 평균 count (보낸 1/0 색).
  4) 끝나면 분석해서 data/stentber_<label>_<반송파>_<시각>/result.png 를 연다:
     (a) 받은 원신호와 보낸 비트  (b) 켤 때·끌 때 평균 계단 응답 = 터치스크린이 반송파를 '어떻게' 보는지
     (c) 비트 판정값 분포 = 0/1 이 얼마나 확실히 갈리는지  (d) 보낸/받은 비트 = 잘 해석되는지
     BER 두 가지: 블라인드 = 수신 쪽 정보만으로 동기·판정 (cap_ook_rx 와 같은 실제 수신기)
                  시각 알려 줌 = 보낸 시각을 알려 주고 판정 (동기 실수를 뺀 상한). 둘이 같으면 동기는 문제 없다.

100–200 kHz 에서 무엇이 보일까 (가설, 이 시험으로 확인):
  CapTIvate 는 버튼마다 ~125 us 동안 전하를 250 번 옮겨 담아 한 번 재고, 그걸 20 ms 마다 한다. 100–200 kHz 는 그 창 안에
  12–25 주기라 반송파 자체는 평균돼 거의 안 보인다. 대신 반송파가 켜져 있는 동안 스텐트의 평균 전압이 달라지면
  (사각파: 끔 = 0 V, 켬 = 0/1.65 V 사각 -> 평균 ~0.8 V 위, E+ 는 직류로 떠 있어 그대로 넘어감) 그만큼 count 가 이동한다
  -> 수신 특징 'level'. 사인은 켬/끔 평균이 같아 이 이동이 없다 -> --wave sine 으로 비교하면 메커니즘이 갈린다.

보드 설정은 cap_ook_rx 와 같다 (녹화 동안 스캔 20 ms, 버튼 카운트 필터·자동 재보정 끔. 끝나면 되돌림).
보드 USB 는 한 프로그램만 열 수 있으니 bswp_map 창은 닫고 실행한다.
"""
import argparse, datetime, json, math, os, subprocess, sys, threading, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, os.path.join(ROOT, "transceiver")]
import captivate_hid as CH
import cap_ook_rx as RX
import cap_ook_tx as TX

DATADIR = os.path.join(HERE, "data")
# 전극 이름 -> (센서 ID, 사이클 ID, 사이클 안 소자 번호, 사이클 소자 수). bswp_map.py 의 CYCLES 와 같다
ELECTRODES = {}
for (sid_, cyc_), names_ in {(0, 0): ["B1", "B2", "B3", "B4"], (0, 1): ["B5", "B6", "B7", "B8"], (1, 0): ["P"],
                            (2, 0): ["S0", "S1", "S2", "S3"], (3, 0): ["W0", "W1", "W2"]}.items():
    for i_, n_ in enumerate(names_):
        ELECTRODES[n_] = (sid_, cyc_, i_, len(names_), names_)
SENSOR_NAME = {0: "버튼", 1: "근접 링", 2: "슬라이더", 3: "휠"}
SGO_DELAY_MS = 500
USB_MS = 0.5                                    # sgo 명령이 Teensy 에 닿기까지 (USB CDC, 대략)


def fmt_hz(f):
    return f"{f / 1e3:g}k" if f >= 1e3 else f"{f:g}"


class Rec(threading.Thread):
    """받는 전극이 든 사이클 패킷(그 사이클 소자 전부의 count·LTA)을 받는 스레드"""

    def __init__(self, br, el):
        super().__init__(daemon=True)
        self.br, self.lock, self.stop_ev = br, threading.Lock(), threading.Event()
        self.sid, self.cyc, _, self.n, _ = ELECTRODES[el]
        self.t, self.c, self.l = [], [], []
        self.error = None

    def run(self):
        while not self.stop_ev.is_set():
            try:
                p = self.br.read_packet(50)
            except OSError as e:                # USB 브리지가 끊김 (2026-10-15 85 Hz 장시간에서 한 번) - 받은 데까지만
                self.error = str(e)
                print(f"  ! 터치스크린 USB 읽기 오류: {e} - 이 런의 녹화를 여기서 멈춤")
                return
            if p is None or p[0] != 1 or p[1] != self.sid or p[2] != self.cyc:
                continue
            d = CH.parse(p)
            if d["kind"] == "cycle" and d["n"] == self.n:
                with self.lock:
                    self.t.append(time.time()); self.c.append(d["count"]); self.l.append(d["lta"])

    def snapshot(self):
        with self.lock:
            return np.array(self.t), np.array(self.c, float), np.array(self.l, float)

    def stop(self):
        self.stop_ev.set(); self.join(1.0)


# ================================================================ 실시간 창
class Live:
    """보내는 동안: 위 = 최근 20 s 버튼 1 count 와 보낸 비트, 아래 = 비트마다의 평균 count (보낸 1 주황 / 0 파랑)"""

    def __init__(self, title, el):
        import matplotlib
        import matplotlib.pyplot as plt
        matplotlib.rcParams["font.family"] = ["AppleGothic", "DejaVu Sans"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        plt.ion()
        self.plt = plt
        self.fig, (self.a1, self.a2) = plt.subplots(2, 1, figsize=(11, 7))
        self.fig.canvas.manager.set_window_title(f"스텐트 -> {el} 실시간")
        self.ln, = self.a1.plot([], [], lw=0.8, color="k", label=f"{el} count")
        self.a1b = self.a1.twinx()
        self.sent, = self.a1b.step([], [], where="post", color="tab:orange", lw=1.2, label="보낸 비트")
        self.a1b.set_ylim(-0.2, 4); self.a1b.set_yticks([0, 1]); self.a1b.set_ylabel("보낸 비트", color="tab:orange")
        self.a1.set_xlabel("시간 [s]"); self.a1.set_ylabel("count")
        self.sc1, = self.a2.plot([], [], "o", ms=4, color="tab:orange", label="보낸 1")
        self.sc0, = self.a2.plot([], [], "o", ms=4, color="tab:blue", label="보낸 0")
        self.thr, = self.a2.plot([], [], "--", color="0.4", lw=1, label="문턱")
        self.a2.set_xlabel("비트 번호"); self.a2.set_ylabel("비트 평균 count"); self.a2.legend(fontsize=8, loc="upper right")
        self.fig.suptitle(title, fontsize=11)
        self.fig.tight_layout()
        self.fig.show()

    def alive(self):
        return self.plt.fignum_exists(self.fig.number)

    def update(self, t, b1, bt, bv, bits, vals, thr, status):
        if len(t):
            m = t >= t[-1] - 20
            self.ln.set_data(t[m], b1[m])
            self.a1.set_xlim(t[-1] - 20, t[-1] + 0.5)
            lo, hi = b1[m].min(), b1[m].max()
            self.a1.set_ylim(lo - 0.5 - 0.6 * (hi - lo + 1), hi + 0.5)
        self.sent.set_data(bt, bv)
        if len(vals):
            j = np.arange(len(vals)); s = np.array(bits[:len(vals)], bool)
            self.sc1.set_data(j[s], vals[s]); self.sc0.set_data(j[~s], vals[~s])
            self.thr.set_data([0, len(vals)], [thr, thr] if thr is not None else [np.nan, np.nan])
            self.a2.set_xlim(-1, max(20, len(vals) + 1))
            lo, hi = np.nanmin(vals), np.nanmax(vals)
            self.a2.set_ylim(lo - 0.3, hi + 0.3)
        self.a2.set_title(status, fontsize=10, loc="left")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def live_bits(t, b1, t_first, Tb, bits, T_scan):
    """보낸 시각을 아는 실시간 판정: 비트마다 [시작 + 표본 1 개, 끝) 안의 count 평균"""
    nb = min(len(bits), int((t[-1] - t_first) / Tb))
    if nb <= 0:
        return np.array([])
    j = np.arange(nb)
    ia = np.searchsorted(t, t_first + j * Tb + T_scan)
    ib = np.searchsorted(t, t_first + (j + 1) * Tb)
    cs = np.concatenate([[0.0], np.cumsum(b1)])
    return np.where(ib > ia, (cs[ib] - cs[ia]) / np.maximum(ib - ia, 1), np.nan)


# ================================================================ 한 반송파
# ================================================================ 터치 센서 전하 전달 주파수 맞추기
# 2026-10-15 실측: 터치 센서는 자기 전하 전달 주파수 f_t 근처(±0.5 kHz) 신호만 받는다 (충전 시각과 전달 시각 사이에 바뀐
# 스텐트 전압만 남고 나머지는 상쇄). 데모 설정(분주 2, 충전·전달 길이 0)은 f_t = 2.016 MHz 이고, 1 kHz·100–200 kHz 는
# count 가 0.1 count 도 안 움직였다. f_t = F_OSC / 2^분주 / (충전 길이 + 전달 길이 + 2):
# 분주 2, 0/0 -> 2.016 MHz, 11/11 -> 168.0 kHz, 분주 3, 5/5 -> 168.0, 6/6 -> 144.0 (모두 실측 일치).
# 반송파를 주면 가장 가까운 f_t 를 만드는 설정을 고르고, 짧은 보정 스윕으로 실제 반응이 가장 큰 주파수에 송신을 맞춘다.
F_OSC = 16.128e6                                # CapTIvate 발진기 (16 MHz 명목, 2.016 MHz x 8 로 잰 값)
P_FREQDIV, P_CHARGE, P_TRANSFER = 0x97, 0x98, 0x99


def choose_ct(fc):
    """반송파 fc 에 가장 가까운 f_t 를 만드는 (분주, 충전 길이, 전달 길이, 예상 f_t).
    실측으로 맞는 걸 확인한 조합만 쓴다: 분주 2, 충전 길이 = 전달 길이 = L -> f_t = 2.016 MHz / (L + 1).
    (분주 2 0/0 = 2.016 MHz, 11/11 = 168.0 kHz, 분주 3 5/5 = 168.0, 6/6 = 144.0 이 모두 공식대로였다. 분주 1 은 반응이
    약했고(23/23 에서 1/16), 충전 ≠ 전달(23/24)은 공식에서 벗어났다.) 100–200 kHz 에서 고를 수 있는 값:
    100.8 106.1 112.0 118.6 126.0 134.4 144.0 155.1 168.0 183.3 201.6 kHz"""
    L = int(np.clip(round(F_OSC / 4 / 2 / fc) - 1, 0, 31))
    cands = [l for l in (L - 1, L, L + 1) if 0 <= l <= 31]
    L = min(cands, key=lambda l: abs(F_OSC / 8 / (l + 1) - fc))
    return 2, L, L, F_OSC / 8 / (L + 1)


def calibrate(br, dev, T, a, ft_pred):
    """반송파를 ft_pred ±cal_span 에서 계단 스윕하며 받는 전극의 흔들림(이웃 차 에너지)이 가장 큰 주파수를 찾는다"""
    ei = ELECTRODES[a.electrode][2]
    freqs = np.arange(ft_pred * (1 - a.cal_span), ft_pred * (1 + a.cal_span) + a.cal_step / 2, a.cal_step)
    fw = [(int(a.cal_dwell * 1e6), 1, [TX.fword(f) for f in freqs])]
    T.upload(dev, fw, a.wave)
    T.ask(dev, "psk 0", want=r"# psk")
    rec = Rec(br, a.electrode); rec.start()
    time.sleep(1.5)
    t_cmd = time.time()
    T.ask(dev, "sgo 300", want=r"armed in|sgo:")
    t_first = t_cmd + 0.3005
    time.sleep(0.3 + len(freqs) * a.cal_dwell + 0.5)
    rec.stop(); T.ask(dev, "off", want=r"# off")
    t, C, _ = rec.snapshot()
    x = C[:, ei]; e = np.concatenate([[0.0], np.diff(x) ** 2])
    en = []
    for i in range(len(freqs)):
        m = (t >= t_first + i * a.cal_dwell + 0.04) & (t < t_first + (i + 1) * a.cal_dwell)
        en.append(float(e[m].mean()) if m.any() else np.nan)
    en = np.array(en)
    if not np.isfinite(en).any():               # 데이터가 하나도 없었다 (USB 끊김 등)
        return dict(freqs=freqs.tolist(), energy=en.tolist(), base=float("nan"), peak_hz=float("nan"),
                    peak_step_hz=float("nan"), peak_gain_db=float("nan"), found=False)
    base = max(float(np.nanmedian(en)), 0.25)    # 대부분의 계단은 반응 밖이라 중앙값 = 바탕 (count 가 정수라 바탕이 0 일 수 있어 하한)
    k = int(np.nanargmax(en))
    # 이웃 계단과 무게 중심으로 조금 더 정밀하게
    w = slice(max(0, k - 2), min(len(en), k + 3))
    fpk = float(np.nansum(freqs[w] * en[w]) / np.nansum(en[w]))
    gain_db = 10 * np.log10(max(en[k], 1e-9) / max(base, 1e-9))
    return dict(freqs=freqs.tolist(), energy=en.tolist(), base=base, peak_hz=fpk, peak_step_hz=float(freqs[k]),
                peak_gain_db=float(gain_db), found=bool(gain_db >= 7))


def frames_for(a, bps):
    """점마다 프레임 수: --target-bits 가 있으면 그만큼(페이로드 기준) 모으되 한 점이 --max-run-s 를 넘지 않게"""
    if not getattr(a, "target_bits", 0):
        return a.frames
    frame_s = (len(TX.PREAMBLE) + len(TX.SFD) + a.payload + TX.GAP_BITS) / bps
    return int(max(1, min(math.ceil(a.target_bits / a.payload), a.max_run_s // frame_s)))


def link_for(a, fc, bps=None):
    frame_s = (len(TX.PREAMBLE) + len(TX.SFD) + a.payload + TX.GAP_BITS) / (bps or a.bps)
    return argparse.Namespace(carrier=fc, zero_hz=(fc + a.fsk_offset if a.keying == "fsk" else None),
                              bps=bps or a.bps, payload=a.payload, prbs=a.prbs, wave=a.wave,
                              seconds=(frames_for(a, bps or a.bps) - 0.5) * frame_s, probe_carriers=TX.PROBE_CARRIERS,
                              probe_block=1.0, probe_reps=3)


def run_carrier(br, dev, T, fc, a, bps=None, psk_baud=0):
    link = link_for(a, fc, bps)
    fw, plan = TX.build(link)
    bits_all = TX.frame_bits(a.payload, a.prbs)[0] * plan["n_frames"]
    Tb = plan["bit_us"] / 1e6
    print(f"\n=== {a.wave} {plan['carrier_hz'] / 1e3:.3f} kHz"
          + (f" / 0 = {plan['zero_hz'] / 1e3:.3f} kHz (FSK, 포락선 일정)" if plan.get("zero_hz") else " OOK")
          + f", {plan['bps']:g} bps, 프레임 {plan['n_frames']} 개 "
          f"({plan['total_s']:.0f} s, 페이로드 {plan['payload'] * plan['n_frames']} 비트) ===")
    T.ask(dev, "off", want=r"# off")
    br.write_param(CH.P_FORCE_RECAL, 0, 0)       # 송신 끈 채로 기준선
    t0 = time.time()
    while time.time() - t0 < 1.2:
        br.read_packet(20)
    up = T.upload(dev, fw, a.wave)
    if psk_baud:                                # DBPSK 겹치기 (TeensyTRX psk 명령, 2026-10-15 펌웨어)
        sym_us = int(round(1e6 / psk_baud))
        if plan["bit_us"] % sym_us:
            raise ValueError(f"DBPSK {psk_baud} baud 심볼({sym_us} us)이 태블릿 비트({plan['bit_us']} us)를 나누지 않는다")
        r = T.ask(dev, f"psk {sym_us} 9", want=r"# psk")
        if f"sym_us={sym_us}" not in r:
            raise RuntimeError(f"psk 설정 실패 (펌웨어가 옛것?): {r.strip()[:80]!r}")
        print(f"  DBPSK 겹치기: {1e6 / sym_us:g} baud (심볼 {sym_us} us, 태블릿 비트당 {plan['bit_us'] // sym_us} 심볼)")
    else:
        T.ask(dev, "psk 0", want=r"# psk")
    rec = Rec(br, a.electrode); rec.start()
    ei = ELECTRODES[a.electrode][2]
    live = Live(f"{a.wave} {fmt_hz(plan['carrier_hz'])}Hz, {plan['bps']:g} bps - {a.electrode}", a.electrode) if a.live else None
    t_rec0 = time.time()
    t_first = None
    status = "송신 전 잡음 녹화"
    t_end = t_rec0 + a.lead + SGO_DELAY_MS / 1e3 + plan["total_s"] + a.tail
    try:
        while time.time() < t_end:
            if t_first is None and time.time() - t_rec0 >= a.lead:
                t_cmd = time.time()
                r = T.ask(dev, f"sgo {SGO_DELAY_MS}", want=r"armed in|sgo:", timeout=2.0)
                if "armed" not in r:
                    raise RuntimeError(f"sgo 실패: {r.strip()[:100]!r}")
                t_first = t_cmd + (SGO_DELAY_MS + USB_MS) / 1e3
                print(f"  송신 시작 {datetime.datetime.now():%H:%M:%S}")
            time.sleep(0.25)
            if live and not live.alive():
                print("  실시간 창을 닫음 - 녹화는 계속")
                live = None
            tt, cc, _ = rec.snapshot()
            if len(tt) < 3:
                continue
            tr = tt - t_rec0
            vals, thr = np.array([]), None
            if t_first is not None and tt[-1] > t_first:
                vals = live_bits(tt, cc[:, ei], t_first, Tb, bits_all, a.scan_ms / 1e3)
                ok = ~np.isnan(vals)
                bb = np.array(bits_all[:len(vals)], bool)
                if (bb & ok).any() and (~bb & ok).any():
                    m1, m0 = vals[bb & ok].mean(), vals[~bb & ok].mean()
                    thr = (m1 + m0) / 2
                    dec = (vals > thr) ^ (m1 < m0)
                    nerr = int(np.sum(dec[ok] != bb[ok]))
                    status = (f"비트 {len(vals)}/{len(bits_all)}   1 일 때 평균 {m1 - m0:+.2f} count   "
                              f"오류 {nerr} (보낸 비트로 맞춘 문턱 - 참고용, 최종은 끝나고 블라인드로)")
            if live:
                if t_first is not None:
                    nb = len(bits_all)
                    j = np.arange(nb + 1)
                    bt = t_first - t_rec0 + j * Tb
                    bv = np.concatenate([bits_all, [0]])
                    bt = np.concatenate([[tr[0]], [bt[0]], bt]); bv = np.concatenate([[0], [0], bv])
                else:
                    bt, bv = np.array([tr[0], tr[-1]]), np.array([0, 0])
                live.update(tr, cc[:, ei], bt, bv, bits_all, vals, thr, status)
            elif int(time.time() * 4) % 20 == 0:
                print(f"  {tr[-1]:6.1f} s  {status}", flush=True)
    finally:
        rec.stop()
        T.ask(dev, "off", want=r"# off")
        if live:                                # 반송파마다 창이 쌓이지 않게 (결과는 result.png 로 열린다)
            live.plt.close(live.fig)
    st = T.sweep_stat(dev)
    t, C, L = rec.snapshot()
    return dict(t=t, C=C, L=L, t_first=t_first, t_rec0=t_rec0, plan=plan, link=link, upload=up, sweep_stat=st,
                psk_baud=psk_baud)


# ================================================================ 분석
def analyse_run(run, a):
    t, C, L, plan, link = run["t"], run["C"], run["L"], run["plan"], run["link"]
    n, T, t0, jit = RX.regularise(t, a.scan_ms / 1e3)
    ei = ELECTRODES[a.electrode][2]
    X = np.column_stack([RX.on_grid(n, C[:, i])[0] for i in range(C.shape[1])])
    x = X[:, ei]
    payload = TX.frame_bits(a.payload, a.prbs)[1]
    Tb = plan["bit_us"] / 1e6
    k = Tb / T
    RX.set_guard(k)
    P = plan["frame_bits"] * k
    s0g = (run["t_first"] - t0) / T              # 보낸 시각으로 아는 첫 비트 [표본]
    feats = RX.features(x)
    # 시각 알려 준 판정
    genie = {}
    for name, f in feats.items():
        fr = []
        for m in range(plan["n_frames"]):
            s0 = s0g + m * P
            if s0 + (len(RX.HDR) + a.payload) * k < len(x):
                d = RX.decode_frame(f, k, s0, payload); d["m"] = m; d["s0"] = s0; fr.append(d)
        bits = sum(d["bits"] for d in fr); errs = sum(d["errors"] for d in fr)
        genie[name] = dict(frames=fr, bits=bits, errors=errs, ber=errs / bits if bits else None,
                           q=float(np.mean([d["q"] for d in fr])) if fr else None,
                           contrast=float(np.mean([d["mean1"] - d["mean0"] for d in fr])) if fr else None)
    # 블라인드 (수신 정보만)
    blind = RX.analyse(x, T, link)
    blind.pop("_grid"); blind.pop("_feats")
    feat = blind["feature"] if blind.get("bits") else min(genie, key=lambda nm: (genie[nm]["errors"], -(genie[nm]["q"] or 0)))
    bf = [d for d in blind["features"][feat]["frames"] if d["valid"]]
    sync_bits = [((d["s0"] - s0g) - round((d["s0"] - s0g) / P) * P) / k for d in bf]
    # 잡음 (송신 전, 첫 비트 1 s 앞까지)
    pre = np.arange(len(x)) < s0g - 1 / T
    sigma = float(np.std(x[pre])) if pre.sum() > 10 else float("nan")
    lta = float(np.median(L[:, ei]))
    # 계단 응답: 비트 경계(0->1, 1->0)마다 앞 비트 평균을 뺀 count, 경계 기준 시각으로 모아 평균
    bits_all = np.array(TX.frame_bits(a.payload, a.prbs)[0] * plan["n_frames"])
    rel = {+1: [], -1: []}
    for j in range(1, len(bits_all)):
        if bits_all[j] == bits_all[j - 1]:
            continue
        b0 = s0g + j * k
        lo_, hi_ = int(math.ceil(b0 - k)), int(math.floor(b0 + 2 * k))
        if lo_ < 0 or hi_ >= len(x):
            continue
        idx = np.arange(lo_, hi_ + 1)
        ref = x[int(math.ceil(b0 - k + 1)):int(math.ceil(b0))].mean()
        rel[1 if bits_all[j] else -1].append(np.column_stack([(idx - b0) * T, x[idx] - ref]))
    step = {}
    for s_, segs in rel.items():
        if not segs:
            continue
        allp = np.vstack(segs)
        edges = np.arange(-Tb, 2 * Tb + T, T)
        cen, mu, se, rms = [], [], [], []
        for e0, e1 in zip(edges[:-1], edges[1:]):
            mm = (allp[:, 0] >= e0) & (allp[:, 0] < e1)
            if mm.sum() > 2:
                cen.append((e0 + e1) / 2); mu.append(allp[mm, 1].mean()); se.append(allp[mm, 1].std() / math.sqrt(mm.sum()))
                rms.append(float(np.sqrt(np.mean(allp[mm, 1] ** 2))))
        step["up" if s_ > 0 else "down"] = dict(t=cen, mean=mu, se=se, rms=rms, n=len(segs))
    g = genie[feat]
    res = dict(electrode=a.electrode, carrier_hz=plan["carrier_hz"], zero_hz=plan.get("zero_hz"), wave=a.wave,
               psk_baud=run.get("psk_baud", 0), bps=plan["bps"], scan_hz=1 / T, samples_per_bit=k,
               missing=int(len(x) - len(t)), arrival_jitter_ms=jit * 1e3, feature=feat,
               blind=dict(bits=blind.get("bits", 0), errors=blind.get("errors"), ber=blind.get("ber"),
                          frames_valid=len(bf), sync_offset_bits=[float(v) for v in sync_bits],
                          features={nm: {kk: vv for kk, vv in v.items() if kk != "frames"} for nm, v in blind["features"].items()}),
               genie={nm: {kk: vv for kk, vv in v.items() if kk != "frames"} for nm, v in genie.items()},
               level_contrast_counts=genie["level"]["contrast"], level_contrast_pct=(100 * genie["level"]["contrast"] / lta
                                                                                     if genie["level"]["contrast"] is not None else None),
               energy_ratio_db=(float(np.mean([10 * np.log10(max(d["on"], 1e-12) / max(d["off"], 1e-12)) for d in genie["energy"]["frames"]]))
                                if genie["energy"]["frames"] else None),
               noise_sigma_counts=sigma, b1_lta=lta, step=step, sweep_stat=run["sweep_stat"])
    return res, dict(x=x, X=X, T=T, s0g=s0g, k=k, P=P, genie=genie, blind=blind, bf=bf, bits_all=bits_all, payload=payload)


def plot_run(res, aux, path):
    import matplotlib
    from matplotlib.figure import Figure
    matplotlib.rcParams["font.family"] = ["AppleGothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    x, T, s0g, k, bits_all = aux["x"], aux["T"], aux["s0g"], aux["k"], aux["bits_all"]
    tg = np.arange(len(x)) * T
    fig = Figure(figsize=(14, 11))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.1, 1, 0.8])
    # (a) 원신호
    a = fig.add_subplot(gs[0, :])
    a.plot(tg, x, lw=0.6, color="k", label=f"{res['electrode']} count")
    bt = (s0g + np.arange(len(bits_all) + 1) * k) * T
    ab = a.twinx()
    ab.fill_between(bt[:-1], 0, bits_all, step="post", color="tab:orange", alpha=0.18, lw=0)
    ab.set_ylim(0, 1); ab.set_yticks([]); a.set_xlim(0, tg[-1])
    a.axvspan(0, s0g * T, color="0.85", alpha=0.6, lw=0)
    a.text(s0g * T / 2, np.max(x), "송신 없음", ha="center", va="top", fontsize=9, color="0.3")
    a.set_title(f"(a) 터치스크린({res['electrode']})이 받은 count - 주황 = 보낸 1", fontsize=10, loc="left")
    a.set_xlabel("시간 [s]"); a.set_ylabel("count")
    # (b) 계단 응답
    a = fig.add_subplot(gs[1, 0])
    shake = res["feature"] != "level"          # 흔들림으로 받는 경우: 평균은 0 근처라 흔들림 크기(rms)를 그린다
    for key, col, lab in (("up", "tab:orange", "0 -> 1 (켤 때)"), ("down", "tab:blue", "1 -> 0 (끌 때)")):
        s = res["step"].get(key)
        if s and shake:
            a.plot(s["t"], s["rms"], "o-", ms=3, lw=1, color=col, label=f"{lab}, 경계 {s['n']} 개")
        elif s:
            a.errorbar(s["t"], s["mean"], yerr=s["se"], fmt="o-", ms=3, lw=1, color=col, label=f"{lab}, 경계 {s['n']} 개 평균")
    a.axvline(0, color="k", lw=0.8); a.axhline(0, color="0.6", lw=0.8)
    Tb = k * T
    a.axvspan(T, Tb, color="0.9", lw=0, label="판정에 쓰는 구간 (비트 앞 표본 1 개 뺌)")
    a.set_xlabel("비트 경계로부터 [s]")
    a.set_ylabel("앞 비트 대비 count 흔들림 rms" if shake else "앞 비트 대비 count")
    a.set_title("(b) 반송파를 켜고 끌 때 count 가 어떻게 바뀌나 - " + ("흔들림 크기 (rms)" if shake else "평균 계단 응답"),
                fontsize=10, loc="left")
    a.legend(fontsize=8)
    # (c) 판정값 분포
    a = fig.add_subplot(gs[1, 1])
    feat = res["feature"]
    fr = aux["genie"][feat]["frames"]
    if fr:
        nz = np.concatenate([np.array(d["norm"]) for d in fr])
        tr = np.concatenate([np.array(aux["payload"], bool) for _ in fr])
        bins = np.linspace(min(nz.min(), -1.5), max(nz.max(), 1.5), 50)
        a.hist(nz[~tr], bins, alpha=0.6, color="tab:blue", label="보낸 0")
        a.hist(nz[tr], bins, alpha=0.6, color="tab:orange", label="보낸 1")
        a.axvline(0, color="k", lw=1)
        a.legend(fontsize=8)
    a.set_title(f"(c) 페이로드 비트 판정값 ({feat}) - 0 = 문턱, ±0.5 = 헤더의 1/0 평균", fontsize=10, loc="left")
    # (d) 보낸/받은 비트
    a = fig.add_subplot(gs[2, :])
    src = aux["bf"][0] if aux["bf"] else (fr[0] if fr else None)
    if src is not None:
        got = np.array(src["got"]); pl = np.array(aux["payload"])
        nshow = len(pl)
        a.imshow(np.vstack([pl, got]), aspect="auto", cmap="Oranges", vmin=-0.3, vmax=1.3, interpolation="nearest",
                 extent=(-0.5, nshow - 0.5, 1.5, -0.5))
        bad = np.where(got != pl)[0]
        a.plot(bad, np.ones(len(bad)), "x", color="tab:red", ms=8, mew=2)
        a.set_yticks([0, 1]); a.set_yticklabels(["보낸", "받은"])
        a.set_xlabel("페이로드 비트 번호")
        which = "블라인드" if aux["bf"] else "시각 알려 줌"
        a.set_title(f"(d) 첫 프레임 페이로드: 보낸 비트 vs 받은 비트 ({which} 판정, 진한색 = 1, x = 틀림 {len(bad)} 개)",
                    fontsize=10, loc="left")
    b = res["blind"]; g = res["genie"][feat]
    bl = f"{b['errors']}/{b['bits']} = {b['ber']:.2e}" if b["bits"] else "동기 못 잡음"
    gn = f"{g['errors']}/{g['bits']} = {g['ber']:.2e}" if g["bits"] else "-"
    lc, lp = res["level_contrast_counts"], res["level_contrast_pct"]
    ct = res.get("touch_ct")
    fig.suptitle(f"{res['electrode']}  {res['wave']} {res['carrier_hz'] / 1e3:.2f} kHz"
                 + (f" / 0 = {res['zero_hz'] / 1e3:.2f} kHz FSK" if res.get("zero_hz") else " OOK")
                 + (f" (터치 센서 분주 {ct['freq_div']}, 충전/전달 {ct['charge_len']}/{ct['transfer_len']})" if ct else "") + ","
                 f" {res['bps']:g} bps, 스캔 {res['scan_hz']:.1f} Hz   "
                 + (f"1 일 때 count {lc:+.2f} ({lp:+.2f} %)" if lc is not None else "")
                 + f", 송신 없을 때 잡음 σ {res['noise_sigma_counts']:.2f}\n"
                 f"BER 블라인드 {bl}   |   시각 알려 줌 {gn}   (특징 {feat})", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=100)


def report(res):
    b, feat = res["blind"], res["feature"]
    g = res["genie"][feat]
    print(f"  스캔 {res['scan_hz']:.2f} Hz, 비트당 {res['samples_per_bit']:.2f} 표본, 빠진 표본 {res['missing']}")
    print(f"  터치스크린이 본 것: 1 일 때 count {res['level_contrast_counts']:+.2f} ({res['level_contrast_pct']:+.2f} % of {res['b1_lta']:.0f}), "
          f"이웃 표본 흔들림 {res['energy_ratio_db']:+.1f} dB, 송신 없을 때 잡음 σ {res['noise_sigma_counts']:.2f} count")
    for nm in res["genie"]:
        gg = res["genie"][nm]; bb = b["features"][nm]
        print(f"  [{nm:9s}] 시각 알려 줌 {gg['errors']}/{gg['bits']}" + (f" Q {gg['q']:.2f}" if gg["q"] is not None else "") +
              f"   블라인드 유효 프레임 {bb['n_valid']}, {bb['errors']}/{bb['bits']}")
    if b["bits"]:
        print(f"  => BER (블라인드, {feat}) {b['errors']}/{b['bits']} = {b['ber']:.2e}"
              + (f" (95% 상한 {3 / b['bits']:.1e})" if b["errors"] == 0 else "")
              + (f",  찾은 프레임 위치 - 보낸 시각 = {np.median(b['sync_offset_bits']):+.2f} 비트" if b["sync_offset_bits"] else ""))
    else:
        print(f"  => 블라인드로는 프레임을 못 찾음. 시각 알려 줌 BER {g['errors']}/{g['bits']}")


def plot_summary(rows, path):
    from matplotlib.figure import Figure
    fig = Figure(figsize=(9, 6))
    a1 = fig.add_subplot(2, 1, 1); a2 = fig.add_subplot(2, 1, 2, sharex=a1)
    f = list(range(len(rows)))
    labels = [f"{r['carrier_hz'] / 1e3:.1f}k\n{r['bps']:g}bps\npsk{r['psk_baud']:g}" for r in rows]
    a1.plot(f, [r["energy_ratio_db"] if r["energy_ratio_db"] is not None else np.nan for r in rows], "o-")
    a1.axhline(0, color="0.6", lw=0.8); a1.set_ylabel("흔들림 켬/끔 [dB]")
    a1.set_title("런별: 터치스크린에 보인 흔들림과 BER", fontsize=10, loc="left")
    ber = [max(r["blind"]["ber"], 0.5 / max(r["blind"]["bits"], 1)) if r["blind"]["bits"] else 0.5 for r in rows]
    a2.semilogy(f, ber, "o-", color="tab:red")
    for fi, r, bv in zip(f, rows, ber):
        a2.annotate(f"{r['blind']['errors']}/{r['blind']['bits']}" if r["blind"]["bits"] else "동기X", (fi, bv),
                    textcoords="offset points", xytext=(0, 6), ha="center", fontsize=8)
    a2.set_xticks(f); a2.set_xticklabels(labels, fontsize=7)
    a2.set_ylabel("BER (블라인드, 0 은 0.5/N 로 표시)")
    for a in (a1, a2):
        a.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def save_capture(d, t, n, C, L, names, meta):
    import csv
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "capture.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["n", "t_unix"] + [f"{nm}_count" for nm in names] + [f"{nm}_lta" for nm in names])
        for i in range(len(t)):
            w.writerow([int(n[i]), f"{t[i]:.4f}"] + [int(v) for v in C[i]] + [int(v) for v in L[i]])
    json.dump(meta, open(os.path.join(d, "meta.json"), "w"), indent=1, ensure_ascii=False)


def reanalyse(d, a):
    """저장된 녹화를 다시 분석해 result.json·result.png 를 새로 쓴다"""
    import csv
    meta = json.load(open(os.path.join(d, "meta.json")))
    rows = list(csv.DictReader(open(os.path.join(d, "capture.csv"))))
    names = meta["cycle_names"]
    t = np.array([float(r["t_unix"]) for r in rows])
    C = np.array([[float(r[f"{nm}_count"]) for nm in names] for r in rows])
    L = np.array([[float(r[f"{nm}_lta"]) for nm in names] for r in rows])
    lk = meta["link"]
    for k_ in ("bps", "payload", "prbs", "wave"):
        setattr(a, k_, lk[k_])
    a.keying, a.fsk_offset = ("fsk", lk["zero_hz"] - lk["carrier"]) if lk.get("zero_hz") else ("ook", 0.0)
    a.electrode, a.scan_ms = meta["electrode"], meta["scan_ms_set"]
    link = argparse.Namespace(**lk)
    run = dict(t=t, C=C, L=L, t_first=meta["t_first_unix"], plan=meta["plan"], link=link, sweep_stat={})
    res, aux = analyse_run(run, a)
    print(f"다시 분석: {os.path.relpath(d, ROOT)}")
    report(res)
    json.dump(res, open(os.path.join(d, "result.json"), "w"), indent=1, ensure_ascii=False)
    plot_run(res, aux, os.path.join(d, "result.png"))
    if not a.no_open:
        subprocess.run(["open", os.path.join(d, "result.png")])


# ================================================================ main
def main():
    ap = argparse.ArgumentParser(description="스텐트 -> CapTIvate 전극 BER 시험 (송신·수신 한 번에)")
    ap.add_argument("--electrode", default="B1", choices=list(ELECTRODES),
                    help="스텐트를 올린 전극 (기본 B1. 버튼 B1-B8, 슬라이더 S0-S3, 휠 W0-W2, 근접 링 P)")
    ap.add_argument("--carrier", type=lambda s: [float(v) for v in s.split(",")], default=[163000.0],
                    help="반송파 [Hz], 쉼표로 여러 개 (기본 163000)")
    ap.add_argument("--wave", choices=("square", "sine"), default="square")
    ap.add_argument("--keying", choices=("ook", "fsk"), default="ook",
                    help="ook = 비트 0 에 출력 멈춤 / fsk = 비트 0 에 반송파 + --fsk-offset 로 계속 송신 (터치 센서 대역 밖이라 "
                         "센서에겐 OOK 와 같고, 송신 포락선이 일정해 DBPSK 를 얹을 수 있다)")
    ap.add_argument("--fsk-offset", type=float, default=3000.0, help="fsk 비트 0 주파수 = 반송파 + 이 값 [Hz] (기본 3000)")
    ap.add_argument("--bps", type=lambda s: [float(v) for v in s.split(",")], default=[5.0],
                    help="태블릿 비트레이트, 쉼표로 여러 개 (기본 5 = 50 Hz 스캔에서 비트당 10 표본)")
    ap.add_argument("--psk-baud", type=lambda s: [float(v) for v in s.split(",")], default=[0.0],
                    help="DBPSK 겹치기 속도 [baud], 쉼표로 여러 개 (0 = 없음). 심볼이 태블릿 비트를 나눠야 한다")
    ap.add_argument("--frames", type=int, default=2, help="반송파마다 프레임 수 (기본 2, 5 bps 에서 프레임 34 s)")
    ap.add_argument("--target-bits", type=int, default=0, help="점마다 모을 페이로드 비트 수 (주면 --frames 대신, 0 = 안 씀)")
    ap.add_argument("--max-run-s", type=float, default=90.0, help="--target-bits 를 쓸 때 한 점 송신 시간 상한 [s]")
    ap.add_argument("--payload", type=int, default=127)
    ap.add_argument("--prbs", type=int, default=7)
    ap.add_argument("--scan-ms", type=int, default=20)
    ap.add_argument("--conv-count", type=int, default=None,
                    help="시험 동안 버튼 센서 측정 목표 카운트 (기본 500, 데모 250). 250 은 count 한 칸이 0.4 %% 이고 잡음이 "
                         "그보다 작아 한 칸 아래 변화가 안 보인다. 1000 은 스캔이 3.8 Hz 로 떨어졌다 (2026-10-15). 0 = 안 바꿈. "
                         "기본: 버튼이면 500, 다른 센서(슬라이더 600, 휠 500, 근접 800)는 안 바꿈")
    ap.add_argument("--lead", type=float, default=4.0, help="송신 전 잡음 녹화 [s]")
    ap.add_argument("--tail", type=float, default=2.0)
    ap.add_argument("--label", default="stent_b1")
    ap.add_argument("--no-live", dest="live", action="store_false", help="실시간 창 없이")
    ap.add_argument("--no-open", action="store_true")
    ap.add_argument("--reanalyse", metavar="DIR", help="저장된 stentber_* 폴더를 다시 분석 (송신·녹화 없이)")
    ap.add_argument("--no-tune", dest="tune", action="store_false",
                    help="터치 센서 전하 전달 주파수를 반송파에 맞추지 않음 (데모 2.016 MHz 그대로 - 100-200 kHz 는 안 보인다)")
    ap.add_argument("--no-cal", action="store_true", help="보정 스윕 없이 예상 f_t 로 바로 보냄")
    ap.add_argument("--cal-span", type=float, default=0.02, help="보정 스윕 폭 (기본 ±2 %%)")
    ap.add_argument("--cal-step", type=float, default=100.0, help="보정 스윕 간격 [Hz] (기본 100)")
    ap.add_argument("--cal-dwell", type=float, default=0.2, help="보정 스윕 계단 길이 [s] (기본 0.2)")
    a = ap.parse_args()
    if a.reanalyse:
        reanalyse(a.reanalyse, a)
        return
    a.no_recal = True                           # 기준선은 반송파마다 송신 끈 채로 잡는다
    a.sensor_id = ELECTRODES[a.electrode][0]
    if a.conv_count is None:
        a.conv_count = 500 if a.sensor_id == 0 else 0
    print(f"받는 전극: {a.electrode} ({SENSOR_NAME[a.sensor_id]} 센서 {a.sensor_id}, 사이클 {ELECTRODES[a.electrode][1]}, "
          f"소자 {ELECTRODES[a.electrode][2]})")

    for fc in a.carrier:
        if not 100e3 <= fc <= 200e3:
            print(f"참고: {fc:g} Hz 는 100-200 kHz 밖")
    import trx_recip as T
    br = CH.Bridge("I2C")
    dev = T.open_port(T.find_teensy(None))
    T.check_trx(dev)
    orig = {}
    rows = []
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    cc_orig = None
    ct_orig = None
    c1_orig = None
    try:
        orig, got = RX.setup_board(br, a)
        if a.conv_count:
            cc_orig = RX.param(br, 0x81, a.sensor_id)
            cc = RX.param(br, 0x81, a.sensor_id, value=a.conv_count)
            print(f"  버튼 측정 목표 카운트 {cc_orig} -> {cc} (보드가 다시 보정, 6 s 기다림)")
            t0 = time.time()
            while time.time() - t0 < 6.0:
                br.read_packet(20)
        if a.tune:
            ct_orig = {c: RX.param(br, c, a.sensor_id) for c in (P_FREQDIV, P_CHARGE, P_TRANSFER)}
        if True:                                # 센서 요약 패킷은 늘 끈다 (필요한 건 받는 전극의 사이클 패킷뿐). 초당 ~430 패킷 넘게
                                                # 몇 분 보내면 PGMR HID 브리지가 끊겼다 (2026-10-15: 20 ms + 요약 = 460/s, 12 ms = 427/s)
            c1_orig = RX.param(br, 0xC1)
            print(f"  센서 요약 패킷 끔 (C1 {c1_orig} -> {RX.param(br, 0xC1, value=0)}) - USB 부하 줄이기")
        for fc in a.carrier:
            ct = cal = None
            if a.tune:
                div, cl, tl, ftp = choose_ct(fc)
                got_ct = [RX.param(br, c, a.sensor_id, value=v) for c, v in ((P_FREQDIV, div), (P_CHARGE, cl), (P_TRANSFER, tl))]
                print(f"\n터치 센서 전하 전달 주파수를 {fc / 1e3:g} kHz 쪽으로: 분주 {div}, 충전/전달 길이 {cl}/{tl} "
                      f"-> 예상 {ftp / 1e3:.2f} kHz (보드 응답 {got_ct}), 다시 보정 4 s")
                t0 = time.time()
                while time.time() - t0 < 4.0:
                    br.read_packet(20)
                ct = dict(freq_div=div, charge_len=cl, transfer_len=tl, ft_pred_hz=ftp)
                if not a.no_cal:
                    cal = calibrate(br, dev, T, a, ftp)
                    for _ in range(2):              # 센서를 다시 맞춘 직후엔 값이 잠깐 멈춰 반응이 0 으로 나올 때가 있었다
                        if cal["found"] and np.isfinite(cal["peak_hz"]):
                            break
                        print("  보정 스윕에 뚜렷한 반응이 없다 - 3 s 뒤 다시")
                        t0 = time.time()
                        while time.time() - t0 < 3.0:
                            br.read_packet(20)
                        cal = calibrate(br, dev, T, a, ftp)
                    print(f"  보정 스윕 {ftp * (1 - a.cal_span) / 1e3:.1f}-{ftp * (1 + a.cal_span) / 1e3:.1f} kHz: "
                          f"가장 큰 반응 {cal['peak_hz'] / 1e3:.3f} kHz (+{cal['peak_gain_db']:.1f} dB)"
                          + ("" if cal["found"] else "  - 뚜렷한 반응 없음, 예상값으로 보냄"))
                    fc_use = cal["peak_hz"] if cal["found"] and np.isfinite(cal["peak_hz"]) else ftp
                else:
                    fc_use = ftp
                if abs(fc_use - fc) > 1:
                    print(f"  보내는 반송파: {fc / 1e3:g} -> {fc_use / 1e3:.3f} kHz (터치 센서가 받는 주파수에 맞춤)")
                fc = fc_use
            for bps in a.bps:
                for pb in a.psk_baud:
                    a.bps_now = bps
                    try:
                        run = run_carrier(br, dev, T, fc, a, bps, pb)
                    except ValueError as e:
                        print(f"  건너뜀: {e}"); continue
                    run["ct"], run["cal"] = ct, cal
                    if run["t_first"] is None or len(run["t"]) < 50:
                        print("  송신 전에 끝남 - 건너뜀"); continue
                    d = os.path.join(DATADIR, f"stentber_{a.label}_{a.electrode}_{a.wave}_{fmt_hz(run['plan']['carrier_hz'])}Hz_"
                                              f"{bps:g}bps_psk{pb:g}_{stamp}")
                    meta = dict(label=a.label, probe=False, scan_ms_set=a.scan_ms, conv_count=a.conv_count, link=vars(run["link"]), plan=run["plan"],
                                board_orig=orig, board_set=got, t_first_unix=run["t_first"], t_rec0_unix=run["t_rec0"],
                                upload=run["upload"], electrode=a.electrode, cycle_names=ELECTRODES[a.electrode][4])
                    n, _, _, _ = RX.regularise(run["t"], a.scan_ms / 1e3)
                    save_capture(d, run["t"], n, run["C"], run["L"], ELECTRODES[a.electrode][4], meta)
                    res, aux = analyse_run(run, a)
                    res["touch_ct"], res["calibration"] = run.get("ct"), run.get("cal")
                    meta["touch_ct"], meta["calibration"] = run.get("ct"), run.get("cal")
                    json.dump(meta, open(os.path.join(d, "meta.json"), "w"), indent=1, ensure_ascii=False)
                    report(res)
                    json.dump(res, open(os.path.join(d, "result.json"), "w"), indent=1, ensure_ascii=False)
                    plot_run(res, aux, os.path.join(d, "result.png"))
                    print(f"  저장: {os.path.relpath(d, ROOT)}")
                    if not a.no_open:
                        subprocess.run(["open", os.path.join(d, "result.png")])
                    rows.append(res)
    except KeyboardInterrupt:
        print("\n중지 (Ctrl+C)")
    finally:
        try:
            T.ask(dev, "off", want=r"# off")
        except Exception:
            pass
        dev.close()

        def restore(br):
            if c1_orig is not None:
                RX.param(br, 0xC1, value=c1_orig)
            if ct_orig and all(v is not None for v in ct_orig.values()):
                for c, v in ct_orig.items():
                    RX.param(br, c, a.sensor_id, value=v)
                t0 = time.time()
                while time.time() - t0 < 2.0:
                    br.read_packet(20)
                print(f"터치 센서 분주/길이 되돌림: {[RX.param(br, c, a.sensor_id) for c in ct_orig]}")
            if cc_orig:
                RX.param(br, 0x81, a.sensor_id, value=cc_orig)
                t0 = time.time()
                while time.time() - t0 < 2.0:
                    br.read_packet(20)
            RX.restore_board(br, orig)
        try:
            restore(br)
        except OSError:                         # 브리지가 끊겼다 다시 붙었으면 새로 열어서 되돌린다
            print("  터치스크린 USB 를 다시 열어 설정을 되돌린다")
            try:
                br.close()
            except Exception:
                pass
            time.sleep(2.0)
            br = CH.Bridge("I2C")
            restore(br)
        br.close()
    if len(rows) > 1:
        print(f"\n반송파     태블릿 bps  DBPSK baud  스캔 Hz  비트당 표본  특징        흔들림 켬/끔  BER(블라인드)        BER(시각 알려 줌)")
        for r in rows:
            g = r["genie"][r["feature"]]; b = r["blind"]
            print(f"{r['carrier_hz'] / 1e3:8.2f}k  {r['bps']:8.3g}  {r['psk_baud']:9g}  {r['scan_hz']:7.1f}  {r['samples_per_bit']:9.2f}   "
                  f"{r['feature']:10s} {r['energy_ratio_db'] if r['energy_ratio_db'] is not None else float('nan'):+8.1f} dB   "
                  + (f"{b['errors']:3d}/{b['bits']:<4d} {b['ber']:.1e}" if b["bits"] else "동기 못 잡음      ")
                  + f"   {g['errors']}/{g['bits']}")
        p = os.path.join(DATADIR, f"stentber_{a.label}_{a.wave}_summary_{stamp}.png")
        plot_summary(rows, p)
        json.dump(rows, open(p.replace(".png", ".json"), "w"), indent=1, ensure_ascii=False)
        print(f"비교 그림: {os.path.relpath(p, ROOT)}")
        if not a.no_open:
            subprocess.run(["open", p])


if __name__ == "__main__":
    main()
