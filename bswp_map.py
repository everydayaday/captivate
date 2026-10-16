#!/usr/bin/env python3
"""
CAPTIVATE-BSWP 패널 실시간 정전용량 지도. 공장 기본 데모 펌웨어 그대로, Design Center 없이 맥에서.

    python bswp_map.py            (또는 Finder 에서 run_bswp_map.command 더블클릭)

왼쪽: 패널을 위에서 본 그림 (커넥터가 위). 전극 16개를 각자 변화량 (LTA - count)/LTA [%] 로 칠한다.
  LTA = 보드가 들고 있는 장기 평균(기준선). 자기용량 전극은 손가락이 닿으면 count 가 기준선 아래로 내려간다.
  흰 테두리 = 보드가 그 전극을 '터치'로 판정, 노란 테두리 = '근접'으로 판정.
  슬라이더·휠은 보드가 계산한 위치를 ▼ 와 ● 로 찍는다.
  '색 기준' 을 '터치 기준 대비' 로 바꾸면 변화량 / 그 전극의 터치 임계값으로 칠한다 (1 = 보드가 터치로 판정하는 선).
오른쪽: 최근 10 초 변화량 곡선 (위: 버튼 1-8, 아래: 슬라이더 S0-S3·휠 W0-W2·근접 링 P). 점선 = 터치 판정선.
아래: 패킷 속도, 주 버튼, 슬라이더·휠 위치, 센서 상태, 판정선(보드에서 읽은 임계값).
[기준선 다시 잡기]: 3 초 뒤 보드에 Force Re-Calibrate(8Eh)를 보내 16 전극의 LTA 를 지금 값으로 다시 잡는다.
  손을 안 댔는데 전극이 계속 노랑/흰 테두리면 기준선이 틀어진 것이다. 보드는 근접·터치 중에는 LTA 를 멈추므로
  (halt = 1) 저절로는 2 분 넘게 걸린다. 화면 아래에 8 초 넘게 켜진 전극을 경고로 띄운다.
[기록] 을 누르면 data/bswp_<시각>.csv 에 스캔마다 한 줄 (전극별 count·LTA·터치·근접, 슬라이더·휠 위치, 센서 상태).

데모 펌웨어는 손이 없으면 절전이다 (근접 링만 10 Hz 로 혼자 재고 데이터는 안 보낸다. 손을 떼고 32 스캔 ≈ 1 초 뒤 잠들고,
절전 중에는 약 50 초마다 한 번 깬다). 그래서 [보드 계속 깨워 두기](기본 켬)가 켤 때 보드 RAM 의 컨트롤러 값을 바꾼다:
C5 비활성 타임아웃 = 65535 스캔(약 36 분), C4 절전 중 깨는 간격 = 1. 그러면 한 번 깬 뒤로는 계속 30 Hz 로 보낸다.
이미 자고 있으면 다음에 깰 때부터 적용되니 처음 한 번은 손을 패널 위로 가져가야 한다 (안 그래도 최대 약 50 초).
창을 닫아도 깨워 둔 채로 둔다 (다음에 켜면 바로 실시간). 데모 원래 동작(32·5)으로 돌리려면 체크를 끄거나
MCU 보드의 RESET 버튼을 누르거나 USB 를 뺐다 꽂는다.
켤 때 3 초 뒤 한 번 기준선을 다시 잡는다 (아래 [기준선 다시 잡기]와 같다).

통신·패킷 형식은 captivate_hid.py 머리말 참고.
"""
import csv, datetime, os, sys, threading, time
from collections import deque
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import patches, cm, colors

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import captivate_hid as CH

matplotlib.rcParams["font.family"] = ["AppleGothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

DATADIR = os.path.join(HERE, "data")
HISTORY_S = 10.0
SLEEP_AFTER_S = 1.0                     # 이만큼 데이터가 없으면 '절전 중'
STUCK_S = 8.0                           # 근접·터치 플래그가 이만큼 계속 켜져 있으면 경고
RECAL_DELAY_S = 3.0                     # [기준선 다시 잡기] 누르고 손 뗄 시간

# ---------------------------------------------------------------- 패널 배치
# (센서ID, 사이클ID) -> 그 사이클 소자 순서대로의 전극 이름. TI 문서의 BSWP 핀 배정과 같다:
#   버튼 1-4 = keypad E00-E03 = CAP0.0-CAP3.0 (위 줄),  버튼 5-8 = E04-E07 = CAP0.1-CAP3.1 (아래 줄)
#   슬라이더 S0-S3 = CAP0.2-CAP3.2 (S0 은 양 끝 두 조각),  휠 W0-W2 = CAP0.3-CAP2.3,  근접 링 P = CAP3.3
CYCLES = {(0, 0): ["B1", "B2", "B3", "B4"], (0, 1): ["B5", "B6", "B7", "B8"],
          (1, 0): ["P"], (2, 0): ["S0", "S1", "S2", "S3"], (3, 0): ["W0", "W1", "W2"]}
SENSORS = {0: "버튼", 1: "근접", 2: "슬라이더", 3: "휠"}
NAMES = ["B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "S0", "S1", "S2", "S3", "W0", "W1", "W2", "P"]
IDX = {n: i for i, n in enumerate(NAMES)}
LAST_SENSOR = max(SENSORS)              # 보드는 센서 0,1,2,3 순서로 보낸다: 3 의 센서 패킷 = 스캔 한 번 끝

# 그림 좌표 (대략 mm, 원점 = 패널 왼쪽 아래, 커넥터가 위)
PANEL_W, PANEL_H = 120.0, 84.0
WHEEL_C, WHEEL_R, WHEEL_W = (27.0, 50.0), 18.0, 9.0
WHEEL_SPAN = {"W0": (30, 150), "W1": (-90, 30), "W2": (150, 270)}   # 도. W0 위, W1 오른쪽 아래, W2 왼쪽 아래
BTN_X, BTN_Y, BTN_R = [62.0, 77.0, 92.0, 107.0], [58.0, 40.0], 6.3
SL_X0, SL_X1, SL_Y0, SL_Y1 = 8.0, 112.0, 9.0, 21.0
FRAME = 4.0                             # 근접 링 테두리 두께


def slider_pieces():
    pitch = (SL_X1 - SL_X0) / 4
    edges = [SL_X0, SL_X0 + pitch / 2] + [SL_X0 + pitch / 2 + pitch * k for k in (1, 2, 3)] + [SL_X1]
    owner = ["S0", "S1", "S2", "S3", "S0"]
    return [(owner[k], edges[k], edges[k + 1]) for k in range(5)]


RES = 0.4                               # 패널 그림 한 픽셀 = 0.4 mm


def panel_raster():
    """패널을 픽셀 그림으로: 픽셀마다 전극 번호(NAMES 순서, -1 = 바탕)와, 전극 테두리(바깥 2 픽셀)인지.
    매 프레임은 색표 두 개(전극 16 + 바탕)를 이 번호로 펼치기만 한다 (imshow 하나)."""
    xs = np.arange(0, PANEL_W, RES) + RES / 2
    ys = np.arange(0, PANEL_H, RES) + RES / 2
    X, Y = np.meshgrid(xs, ys)          # 행 = y (아래부터, imshow origin="lower")
    lab = np.full(X.shape, -1, np.int16)
    lab[(X < FRAME) | (X > PANEL_W - FRAME) | (Y < FRAME) | (Y > PANEL_H - FRAME)] = IDX["P"]
    for k, n in enumerate(NAMES[:8]):
        lab[(X - BTN_X[k % 4]) ** 2 + (Y - BTN_Y[k // 4]) ** 2 < BTN_R ** 2] = IDX[n]
    R = np.hypot(X - WHEEL_C[0], Y - WHEEL_C[1])
    A = np.degrees(np.arctan2(Y - WHEEL_C[1], X - WHEEL_C[0]))
    ring = (R < WHEEL_R) & (R > WHEEL_R - WHEEL_W)
    for n, (a0, a1) in WHEEL_SPAN.items():
        lab[ring & ((A - a0) % 360 < a1 - a0)] = IDX[n]
    for n, x0, x1 in slider_pieces():
        lab[(X >= x0) & (X < x1) & (Y >= SL_Y0) & (Y < SL_Y1)] = IDX[n]
    edge = np.zeros(lab.shape, bool)
    for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0), (0, 2), (0, -2), (2, 0), (-2, 0)):
        edge |= lab != np.roll(np.roll(lab, dy, 0), dx, 1)
    return lab, edge & (lab >= 0)


NICE = [1, 1.5, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60, 80, 100, 150, 200]


def nice_up(x):
    """눈금이 덜 바뀌게 x 이상인 '깔끔한' 수로 올린다 (축·색 범위가 바뀔 때만 전체를 다시 그리므로)"""
    return next((v for v in NICE if v >= x), NICE[-1])


# ---------------------------------------------------------------- 수신 스레드
class Receiver(threading.Thread):
    """HID 패킷을 받아 스캔 한 번씩(frame) 모은다. GUI 는 latest / history 만 읽는다."""

    def __init__(self, bridge):
        super().__init__(daemon=True)
        self.br = bridge
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.cur = {}                   # 이번 스캔에 들어온 사이클 패킷 {(sid, cyc): dict}
        self.sens = {}                  # 센서 패킷 {sid: dict}
        self.latest = None              # 마지막으로 완성된 스캔
        self.history = deque()          # (t, 변화량% 배열)
        self.n_packets = 0
        self.t_last = 0.0
        self.frame_times = deque(maxlen=60)
        self.resolution = {}            # {2: 슬라이더 해상도, 3: 휠 해상도} 보드에 물어서 받은 값
        self.touch_thr = {}             # 전극 이름 -> 터치 판정 변화량 [%] (= 보드의 상대 임계값/128)
        self.prox_thr = {}              # 센서 ID -> 근접 판정 변화량 [count]
        self.flag_since = np.full(16, np.nan)   # 전극마다 근접·터치 플래그가 켜진 시각 (꺼져 있으면 NaN)
        self.outbox = deque()           # 보드로 보낼 패킷. 이 스레드만 HID 에 쓴다
        self.t_sent = 0.0
        self.t_asked = 0.0
        self.orig = {}                  # 깨워 두기 전 보드 값 {C4/C5: 값}
        self.ctrl = {}                  # 보드가 마지막으로 알려 준 컨트롤러 값 {C4/C5: 값}
        self.awake_wanted = False
        self.writer = None              # 기록 중이면 csv.writer
        self.warn = ""

    def keep_awake(self, on):
        """보드를 절전에 안 들어가게: 손이 없어진 뒤 65535 스캔(약 36 분) 더 깨어 있고, 그래도 잠들면 바로 다시 깬다.
        끌 때는 처음에 읽어 둔 원래 값(없으면 데모 값 32·5)으로 돌린다. 지금 잠들어 있으면 다음에 깰 때부터 적용된다
        (손을 가까이 대거나, 최대 약 50 초)."""
        if on:
            self.outbox.extend([[CH.P_INACTIVITY, 0, 0, 0, 0, 0, 0], [CH.P_WAKE_INTERVAL, 0, 0, 0, 0, 0, 0]])
            self.outbox.extend([[CH.P_INACTIVITY, 1, 0, 0, 0, 0xFF, 0xFF], [CH.P_WAKE_INTERVAL, 1, 0, 0, 0, 1, 0]])
        else:
            self.outbox.extend(self.restore_packets())
        self.awake_wanted = on

    def restore_packets(self):
        ina = self.orig.get(CH.P_INACTIVITY, CH.DEMO_INACTIVITY)
        wak = self.orig.get(CH.P_WAKE_INTERVAL, CH.DEMO_WAKE_INTERVAL)
        return [[CH.P_INACTIVITY, 1, 0, 0, 0, ina & 0xFF, ina >> 8], [CH.P_WAKE_INTERVAL, 1, 0, 0, 0, wak, 0]]

    def recalibrate(self):
        """16 전극 모두 기준선(LTA)을 지금 값으로 다시 잡게 한다 (파라미터 8Eh = Force Re-Calibrate).
        보드는 요청 하나에 패널 전체를 다시 보정하고(CAPT_calibrateUI) 그동안 ~0.5 초 스캔을 멈춘다. 그래서 한 번만 보낸다
        (센서마다 보냈더니 0.5 초씩 네 번 멈췄다)."""
        self.outbox.append([CH.P_FORCE_RECAL, 1, 0, 0, 0, 0, 0])

    def _missing_params(self):
        q = [[CH.P_RESOLUTION, 0, sid, 0, 0, 0, 0] for sid in (2, 3) if sid not in self.resolution]
        q += [[CH.P_PROX_THR, 0, sid, 0, 0, 0, 0] for sid in SENSORS if sid not in self.prox_thr]
        for (sid, cyc), names in CYCLES.items():
            q += [[CH.P_TOUCH_THR, 0, sid, cyc, e, 0, 0] for e, n in enumerate(names) if n not in self.touch_thr]
        return q

    def run(self):
        while not self.stop.is_set():
            now = time.time()
            # 보드로 보내기: 한 번에 몰아 보내면 보드 수신 버퍼가 넘칠 수 있어 30 ms 간격으로 하나씩
            if self.outbox and now - self.t_sent > 0.03:
                self.br.send_packet(self.outbox.popleft())
                self.t_sent = now
            p = self.br.read_packet(20 if self.outbox else 50)
            now = time.time()
            if p is None:
                continue
            d = CH.parse(p)
            with self.lock:
                self.n_packets += 1
                self.t_last = now
                if d["kind"] == "cycle":
                    key = (d["sensor"], d["cycle"])
                    if key in self.cur:                     # 같은 사이클이 또 왔다 = 새 스캔 시작
                        self._finish(now)
                    names = CYCLES.get(key)
                    if names is None or len(names) != d["n"]:
                        self.warn = f"예상과 다른 사이클 s{key[0]} c{key[1]} (소자 {d['n']}개) - BSWP 데모 펌웨어가 맞나?"
                    self.cur[key] = d
                elif d["kind"] == "sensor":
                    self.sens[d["sensor"]] = d
                    if d["sensor"] == LAST_SENSOR:
                        self._finish(now)
                elif d["kind"] == "param":
                    self._got_param(d)
            # 해상도·임계값은 보드가 깨어 있을 때만 답한다. 모르는 게 있으면 깨어 있는 동안 2 초에 한 번 묻는다
            if not self.outbox and now - self.t_asked > 2.0:
                self.t_asked = now
                self.outbox.extend(self._missing_params())

    def _got_param(self, d):
        sid, data = d["sensor"], d["data"]
        if d["cmd"] == CH.P_RESOLUTION:
            self.resolution[sid] = d["value"]
        elif d["cmd"] == CH.P_PROX_THR:
            self.prox_thr[sid] = d["value"]
        elif d["cmd"] in (CH.P_INACTIVITY, CH.P_WAKE_INTERVAL):
            v = d["value"]
            self.ctrl[d["cmd"]] = v
            # 읽기 답(rw=0)만 원래 값으로 기억. 이전 실행이 깨워 둔 채 죽었으면(65535·1) 데모 값을 원래 값으로 본다
            if d["rw"] == 0 and d["cmd"] not in self.orig:
                awake_val = 0xFFFF if d["cmd"] == CH.P_INACTIVITY else 1
                self.orig[d["cmd"]] = v if v != awake_val else (
                    CH.DEMO_INACTIVITY if d["cmd"] == CH.P_INACTIVITY else CH.DEMO_WAKE_INTERVAL)
        elif d["cmd"] == CH.P_TOUCH_THR:
            names = CYCLES.get((sid, data[0]), [])
            if data[1] < len(names):
                self.touch_thr[names[data[1]]] = 100.0 * data[2] / 128

    def _finish(self, now):
        if not self.cur:
            return
        cnt = np.full(16, np.nan); lta = np.full(16, np.nan)
        touch = np.zeros(16, bool); prox = np.zeros(16, bool)
        for key, d in self.cur.items():
            for k, name in enumerate(CYCLES.get(key, [])[:d["n"]]):
                i = IDX[name]
                cnt[i], lta[i], touch[i], prox[i] = d["count"][k], d["lta"][k], d["touch"][k], d["prox"][k]
        with np.errstate(invalid="ignore", divide="ignore"):
            delta = 100.0 * (lta - cnt) / lta
        f = dict(t=now, count=cnt, lta=lta, delta=delta, touch=touch, prox=prox,
                 sens={k: dict(v) for k, v in self.sens.items()})
        on = touch | prox
        self.flag_since[~on] = np.nan
        self.flag_since[on & np.isnan(self.flag_since)] = now
        self.latest = f
        self.history.append((now, delta))
        while self.history and now - self.history[0][0] > HISTORY_S:
            self.history.popleft()
        self.frame_times.append(now)
        if self.writer:
            self._write_row(f)
        self.cur = {}

    # ------------------------------------------------------------ 기록
    def start_record(self, path):
        with self.lock:
            self.fh = open(path, "w", newline="")
            self.writer = csv.writer(self.fh)
            head = ["t_unix"]
            for n in NAMES:
                head += [f"{n}_count", f"{n}_lta", f"{n}_touch", f"{n}_prox"]
            head += ["keypad_dominant", "slider_pos", "wheel_pos", "slider_res", "wheel_res"]
            head += [f"{SENS_EN[s]}_status" for s in sorted(SENSORS)]
            self.writer.writerow(head)
            self.n_rows = 0

    def stop_record(self):
        with self.lock:
            if self.writer:
                self.fh.close()
            self.writer = None

    def _write_row(self, f):
        row = [f"{f['t']:.4f}"]
        for i in range(16):
            row += [_int(f["count"][i]), _int(f["lta"][i]), int(f["touch"][i]), int(f["prox"][i])]
        s = f["sens"]
        row += [s.get(0, {}).get("b0", ""), _pos(s.get(2)), _pos(s.get(3)),
                self.resolution.get(2, ""), self.resolution.get(3, "")]
        row += [s.get(k, {}).get("status", "") for k in sorted(SENSORS)]
        self.writer.writerow(row)
        self.n_rows += 1


SENS_EN = {0: "keypad", 1: "prox", 2: "slider", 3: "wheel"}


def _int(x):
    return "" if np.isnan(x) else int(x)


def _pos(d):
    if not d or d["pos"] == 0xFFFF:
        return ""
    return d["pos"]


# ---------------------------------------------------------------- 화면
# 빨리 그리기: 축·눈금·범례·색 막대처럼 안 바뀌는 것은 한 번 그려 배경으로 떠 두고(copy_from_bbox), 매 프레임은
# 바뀌는 것(패널 그림 하나, 글씨, 곡선)만 그 위에 그려 붙인다(blit). 축 범위나 색 범위가 바뀔 때만 전체를 다시 그린다.
# 프레임을 다 그린 뒤 최소 MIN_GAP_MS 는 비워 두어 버튼·체크박스 클릭이 처리되게 한다.
FRAME_MS = 33                           # 화면 목표 간격 (보드 스캔과 같은 30 Hz)
MIN_GAP_MS = 15
BG_RGBA = (43, 43, 43, 255)
EDGE_IDLE, EDGE_PROX, EDGE_TOUCH = (85, 85, 85, 255), (250, 204, 21, 255), (255, 255, 255, 255)


class App:
    def __init__(self, root, bridge):
        self.root = root
        self.br = bridge
        self.rx = Receiver(bridge)
        self.peak = deque(maxlen=150)   # 자동 색 범위용: 최근 스캔들(약 5 초)의 최대 변화량
        root.title("CAPTIVATE-BSWP 정전용량 지도")

        top = ttk.Frame(root, padding=(8, 6)); top.pack(fill="x")
        ttk.Label(top, text="색 범위 [%]").pack(side="left")
        self.scale_var = tk.StringVar(value="자동")
        ttk.Combobox(top, textvariable=self.scale_var, width=6, state="readonly",
                     values=["자동", "1", "2", "5", "10", "20", "40"]).pack(side="left", padx=(4, 16))
        ttk.Label(top, text="색 기준").pack(side="left")
        self.color_mode = tk.StringVar(value="변화량 %")
        ttk.Combobox(top, textvariable=self.color_mode, width=11, state="readonly",
                     values=["변화량 %", "터치 기준 대비"]).pack(side="left", padx=(4, 16))
        self.show_vals = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="전극 위에 숫자", variable=self.show_vals).pack(side="left")
        self.awake_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="보드 계속 깨워 두기", variable=self.awake_var,
                        command=lambda: self.rx.keep_awake(self.awake_var.get())).pack(side="left", padx=(16, 0))
        ttk.Button(top, text="기준선 다시 잡기", command=self.ask_recal).pack(side="left", padx=(16, 4))
        # 글자 수가 바뀌어도 창 크기가 안 바뀌게 폭을 고정한다
        self.recal_lbl = ttk.Label(top, text="", width=30); self.recal_lbl.pack(side="left")
        self.recal_at = None
        self.rec_btn = ttk.Button(top, text="기록", width=9, command=self.toggle_record); self.rec_btn.pack(side="right")
        self.rec_lbl = ttk.Label(top, text="", width=34, anchor="e"); self.rec_lbl.pack(side="right", padx=8)

        self.fig = Figure(figsize=(12, 6.2), dpi=100)
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.35, 1], hspace=0.35, wspace=0.12,
                                   left=0.02, right=0.98, top=0.93, bottom=0.08)
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self.ax_b = self.fig.add_subplot(gs[0, 1])
        self.ax_o = self.fig.add_subplot(gs[1, 1])
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        # 상태 글씨는 줄 수·폭을 고정 (예전에는 글씨 길이에 따라 창이 늘었다 줄었다 하며 그때마다 전체를 다시 그렸다)
        self.status = tk.Label(root, anchor="nw", justify="left", font=("Menlo", 11), padx=8, pady=6,
                               height=6, width=150)
        self.status.pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.cmap = matplotlib.colormaps["YlOrRd"]
        self.norm = colors.Normalize(0, 5)
        self.anim = []                  # 매 프레임 다시 그리는 것들
        self.bg = None
        self.need_full = True
        self.ylim = {}
        self.shrink_since = {}
        self.last_f = None
        self.t_status = 0.0
        self.draw_ms = deque(maxlen=30)
        self.frame_t = deque(maxlen=30)
        self._draw_panel()
        self._draw_traces()
        self.canvas.mpl_connect("draw_event", self._on_draw)

        # 창 크기를 지금 크기로 고정 (사용자가 손으로 바꾸는 건 된다)
        root.update_idletasks()
        root.geometry(f"{root.winfo_reqwidth()}x{root.winfo_reqheight()}")

        self.rx.keep_awake(True)
        self.rx.start()
        self.ask_recal()                # 켤 때 한 번 기준선을 잡는다 (손이 있었어도 떼면 보드가 음의 터치로 알아서 다시 잡는다)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    def _anim(self, artist):
        artist.set_animated(True)
        self.anim.append(artist)
        return artist

    # ------------------------------------------------------------ 그림 뼈대
    def _draw_panel(self):
        ax = self.ax_map
        self.lab, self.edge = panel_raster()
        self.lab1 = (self.lab + 1).astype(np.intp)          # 0 = 바탕, 1..16 = 전극
        self.edge_lab1 = self.lab1[self.edge]
        self.rgba = np.empty(self.lab.shape + (4,), np.uint8)
        self.im = self._anim(ax.imshow(self.rgba, extent=(0, PANEL_W, 0, PANEL_H), origin="lower",
                                       interpolation="nearest", zorder=1))
        ax.set_xlim(-3, PANEL_W + 3); ax.set_ylim(-3, PANEL_H + 9)
        ax.set_aspect("equal"); ax.axis("off")
        ax.set_title("패널 (위에서 본 모습, 커넥터가 위)", fontsize=10, loc="left")
        ax.add_patch(patches.Rectangle((PANEL_W / 2 - 22, PANEL_H), 44, 5, color="#cfcfcf", zorder=0))
        ax.text(PANEL_W / 2, PANEL_H + 2.5, "커넥터 → MCU 보드", ha="center", va="center", fontsize=8)

        self.vals = {}                          # 전극 이름 -> 숫자 Text 들 (S0 은 양 끝 두 개)
        self.texts = {n: [] for n in NAMES}     # 전극 이름 -> 이름 Text 들 (배경 밝기에 따라 색을 바꾼다)
        T = lambda *a, **k: self._anim(ax.text(*a, zorder=4, **k))
        self.texts["P"].append(T(PANEL_W - 2, PANEL_H - 0.4, "근접 링 P", ha="right", va="top", fontsize=8))
        self.vals["P"] = [T(PANEL_W - 30, PANEL_H - 0.4, "", ha="right", va="top", fontsize=8)]
        for k, name in enumerate(NAMES[:8]):
            x, y = BTN_X[k % 4], BTN_Y[k // 4]
            self.texts[name].append(T(x, y + 1.5, name[1], ha="center", va="center", fontsize=13))
            self.vals[name] = [T(x, y - 2.6, "", ha="center", va="center", fontsize=7)]
        for name, (a0, a1) in WHEEL_SPAN.items():
            mid = np.deg2rad((a0 + a1) / 2); r = WHEEL_R - WHEEL_W / 2
            x, y = WHEEL_C[0] + r * np.cos(mid), WHEEL_C[1] + r * np.sin(mid)
            self.texts[name].append(T(x, y + 1.3, name, ha="center", va="center", fontsize=9))
            self.vals[name] = [T(x, y - 1.8, "", ha="center", va="center", fontsize=7)]
        for name, x0, x1 in slider_pieces():
            xc, yc = (x0 + x1) / 2, (SL_Y0 + SL_Y1) / 2
            self.texts[name].append(T(xc, yc + 1.8, name, ha="center", va="center", fontsize=9))
            self.vals.setdefault(name, []).append(T(xc, yc - 2.2, "", ha="center", va="center", fontsize=7))
        self.wheel_dot = self._anim(ax.plot([], [], "o", ms=11, mfc="#22c55e", mec="w", mew=1.5, zorder=6)[0])
        self.wheel_txt = T(*WHEEL_C, "", ha="center", va="center", fontsize=9, color="w")
        self.slider_mark = self._anim(ax.plot([], [], "v", ms=13, mfc="#22c55e", mec="w", mew=1.5, zorder=6)[0])

        self.cbar = self.fig.colorbar(cm.ScalarMappable(norm=self.norm, cmap=self.cmap), ax=ax,
                                      orientation="horizontal", fraction=0.04, pad=0.02, aspect=40)
        self.cbar.ax.tick_params(labelsize=8)
        self._set_cbar_label(False)
        self._paint_panel(np.zeros(16), np.zeros(16, bool), np.zeros(16, bool))

    def _set_cbar_label(self, rel):
        self.cbar.set_label("터치 기준 대비 (1 = 보드의 터치 판정선)" if rel else
                            "변화량 (LTA − count)/LTA [%]  (손가락이 닿으면 커진다)", fontsize=8)

    def _draw_traces(self):
        tab = matplotlib.colormaps["tab10"]
        self.lines = {}
        for ax, names, title in ((self.ax_b, NAMES[:8], "버튼 1-8"),
                                 (self.ax_o, NAMES[8:], "슬라이더 S0-S3 · 휠 W0-W2 · 근접 링 P")):
            for k, n in enumerate(names):
                style = "--" if n.startswith("W") else ("-" if n != "P" else ":")
                self.lines[n] = self._anim(ax.plot([], [], style, lw=1.3, color=tab(k % 10) if n != "P" else "k",
                                                   label=n)[0])
            ax.set_xlim(-HISTORY_S, 0); ax.set_ylim(-1, 5); self.ylim[ax] = (-1, 5)
            ax.axhline(0, color="0.7", lw=0.8)
            ax.set_title(title, fontsize=9, loc="left"); ax.set_ylabel("변화량 [%]", fontsize=8)
            ax.tick_params(labelsize=8); ax.grid(alpha=0.3)
            ax.legend(fontsize=7, ncol=8, loc="upper left", frameon=False, handlelength=1.5, columnspacing=0.8)
        self.ax_o.set_xlabel("시간 [s] (0 = 지금)", fontsize=8)
        # 터치 판정선 (보드에서 읽은 임계값. 읽기 전에는 안 보인다. 바뀌면 배경을 다시 그린다)
        self.thr_lines = {}
        self.thr_shown = {}
        for key, ax, col in (("B", self.ax_b, "0.35"), ("S", self.ax_o, tab(1)), ("W", self.ax_o, tab(5)), ("P", self.ax_o, "k")):
            ln, = ax.plot([], [], "--", lw=1.0, color=col, alpha=0.8)
            tx = ax.text(-0.15, 0, "", ha="right", va="bottom", fontsize=7, color=col)
            self.thr_lines[key] = (ln, tx)

    # ------------------------------------------------------------ 그리기
    def _on_draw(self, event):
        """전체 그리기(처음·창 크기 바뀜·축 범위 바뀜) 끝에 불린다: 움직이는 것 없는 배경을 떠 두고 그 위에 그린다"""
        self.bg = self.canvas.copy_from_bbox(self.fig.bbox)
        for a in self.anim:
            self.fig.draw_artist(a)

    def _render(self):
        if self.need_full or self.bg is None:
            self.need_full = False
            self.canvas.draw()          # -> _on_draw
            return
        self.canvas.restore_region(self.bg)
        for a in self.anim:
            self.fig.draw_artist(a)
        self.canvas.blit(self.fig.bbox)

    def _paint_panel(self, v, touch, prox):
        """v = 전극 16 개의 색 값(norm 전). 패널 그림 한 장을 만든다"""
        fill = np.empty((17, 4), np.uint8); fill[0] = BG_RGBA
        fill[1:] = (self.cmap(self.norm(np.maximum(v, 0.0))) * 255).astype(np.uint8)
        edge = np.empty((17, 4), np.uint8); edge[0] = BG_RGBA
        edge[1:] = np.where(touch[:, None], EDGE_TOUCH, np.where(prox[:, None], EDGE_PROX, EDGE_IDLE))
        np.take(fill, self.lab1, axis=0, out=self.rgba)
        self.rgba[self.edge] = edge[self.edge_lab1]
        self.im.set_data(self.rgba)

    def _set_ylim(self, ax, lo, hi, now):
        """곡선 축 범위: 넓힐 때는 바로, 좁힐 때는 3 초 지켜본 뒤 (그때만 전체를 다시 그린다)"""
        want = (-nice_up(-lo) if lo < -1 else -1, nice_up(hi))
        cur = self.ylim[ax]
        if want == cur:
            self.shrink_since[ax] = None
            return
        if want[0] < cur[0] or want[1] > cur[1]:          # 넓히기: 다른 쪽은 그대로 두고 바로
            want = (min(want[0], cur[0]), max(want[1], cur[1]))
        elif self.shrink_since.get(ax) is None:
            self.shrink_since[ax] = now
            return
        elif now - self.shrink_since[ax] < 3.0:
            return
        self.shrink_since[ax] = None
        self.ylim[ax] = want
        ax.set_ylim(*want)
        self.need_full = True

    # ------------------------------------------------------------ 갱신
    def tick(self):
        t0 = time.perf_counter()
        try:
            self._update()
        finally:
            dt = (time.perf_counter() - t0) * 1e3
            self.root.after(int(max(MIN_GAP_MS, FRAME_MS - dt)), self.tick)

    def _update(self):
        rx = self.rx
        with rx.lock:
            f = rx.latest
            hist = list(rx.history) if f is not self.last_f else None
            res = dict(rx.resolution)
            ft = list(rx.frame_times)
            n_pk, t_last, warn = rx.n_packets, rx.t_last, rx.warn
            tthr, pthr = dict(rx.touch_thr), dict(rx.prox_thr)
            ctrl = dict(rx.ctrl)
            since = rx.flag_since.copy()
            n_rows = rx.n_rows if rx.writer else None
        now = time.time()
        asleep = (now - t_last) > SLEEP_AFTER_S
        self._recal_countdown(now)
        if n_rows is not None:
            self.rec_lbl.config(text=f"● {os.path.basename(self.rec_path)}  {n_rows} 줄", foreground="#dc2626")
        if f is None:
            if now - self.t_status > 0.5:
                self.t_status = now
                self.status.config(text=f"{self.br.version}  {self.br.mode}\n"
                                        "아직 데이터 없음 - 보드가 절전 중이다. 패널 위로 손을 한 번 가져가면 깨어나고\n"
                                        "('보드 계속 깨워 두기'가 켜져 있으면) 그 뒤로는 손을 떼도 계속 30 Hz 로 보낸다.\n"
                                        "손을 안 대도 최대 약 50 초 안에 저절로 깬다.", fg="#b45309")
            if self.need_full:
                self._render()
            return

        # 판정선이 새로 들어오면 배경에 그린다
        groups = {"B": NAMES[:8], "S": NAMES[8:12], "W": NAMES[12:15], "P": ["P"]}
        label = {"B": "버튼 터치", "S": "슬라이더 터치", "W": "휠 터치", "P": "근접 링 터치"}
        thr_top = {self.ax_b: 2.0, self.ax_o: 2.0}
        for key, names in groups.items():
            v = [tthr[n] for n in names if n in tthr]
            if not v:
                continue
            y = float(np.mean(v))
            ln, tx = self.thr_lines[key]
            thr_top[ln.axes] = max(thr_top[ln.axes], y * 1.12)
            if self.thr_shown.get(key) != y:
                self.thr_shown[key] = y
                ln.set_data([-HISTORY_S, 0], [y, y]); tx.set_position((-0.15, y)); tx.set_text(f"{label[key]} {y:.1f}%")
                self.need_full = True

        new_frame = f is not self.last_f
        if new_frame or self.need_full:
            self.last_f = f
            # 색 범위. '터치 기준 대비' 는 변화량 / 그 전극의 터치 임계값 (1 = 보드가 터치로 판정하는 선)
            rel = self.color_mode.get() == "터치 기준 대비"
            d = np.nan_to_num(f["delta"], nan=0.0)
            if rel:
                d = np.nan_to_num(d / np.array([tthr.get(n, np.nan) for n in NAMES]), nan=0.0)
                vmax = 1.2
            else:
                if new_frame:
                    self.peak.append(float(np.max(d)))
                vmax = nice_up(1.15 * max(self.peak, default=1.0)) if self.scale_var.get() == "자동" \
                    else float(self.scale_var.get())
            if self.norm.vmax != vmax or rel != getattr(self, "_rel", None):
                self._rel = rel
                self.norm.vmax = vmax
                self.cbar.update_normal(self.cbar.mappable)
                self._set_cbar_label(rel)
                self.need_full = True
            self._paint_panel(d, f["touch"], f["prox"])

            show = self.show_vals.get()
            for i, n in enumerate(NAMES):
                ink = "w" if self.norm(max(d[i], 0.0)) > 0.6 else "k"
                val = f"{f['delta'][i]:+.1f}%" if show and not np.isnan(f["delta"][i]) else ""
                for t in self.vals[n]:
                    t.set_text(val); t.set_color(ink)
                for t in self.texts[n]:
                    t.set_color(ink)

            # 슬라이더·휠 위치 (보드가 계산한 값)
            s = f["sens"]
            sl, wh = s.get(2), s.get(3)
            if sl and sl["pos"] != 0xFFFF and res.get(2):
                self.slider_mark.set_data([SL_X0 + (SL_X1 - SL_X0) * sl["pos"] / res[2]], [SL_Y1 + 3.2])
            else:
                self.slider_mark.set_data([], [])
            if wh and wh["pos"] != 0xFFFF and res.get(3):
                # 위치 0 = W0 가운데(위), W0 -> W1 -> W2 방향(시계 방향)으로 증가한다고 본다
                ang = np.deg2rad(90 - 360.0 * wh["pos"] / res[3])
                r = WHEEL_R - WHEEL_W / 2
                self.wheel_dot.set_data([WHEEL_C[0] + r * np.cos(ang)], [WHEEL_C[1] + r * np.sin(ang)])
                self.wheel_txt.set_text(str(wh["pos"]))
            else:
                self.wheel_dot.set_data([], []); self.wheel_txt.set_text("")

            # 시간 곡선
            if hist:
                t = np.array([h[0] for h in hist]) - now
                D = np.array([h[1] for h in hist])
                for i, n in enumerate(NAMES):
                    self.lines[n].set_data(t, D[:, i])
                for ax, part in ((self.ax_b, D[:, :8]), (self.ax_o, D[:, 8:])):
                    ok = np.isfinite(part).any()
                    self._set_ylim(ax, np.nanmin(part) if ok else -1.0,
                                   max(thr_top[ax], (np.nanmax(part) if ok else 1.0) * 1.15), now)
            t0 = time.perf_counter()
            self._render()
            self.draw_ms.append((time.perf_counter() - t0) * 1e3)
            self.frame_t.append(now)

        if now - self.t_status < 0.25:      # 상태 글씨는 초당 4 번만
            return
        self.t_status = now
        s = f["sens"]
        sl, wh = s.get(2), s.get(3)
        rate = (len(ft) - 1) / (ft[-1] - ft[0]) if len(ft) > 1 and ft[-1] > ft[0] else 0.0
        fps = (len(self.frame_t) - 1) / (self.frame_t[-1] - self.frame_t[0]) \
            if len(self.frame_t) > 1 and self.frame_t[-1] > self.frame_t[0] else 0.0
        def flags(sid):
            x = s.get(sid)
            return ",".join(k for k, v in x["flags"].items() if v) if x else "?"
        kp = s.get(0, {})
        dom = f"{kp.get('b0', -1) + 1}" if kp and "touch" in flags(0) else "-"
        slt = (f"{sl['pos']}/{res.get(2, '?')}" if sl and sl["pos"] != 0xFFFF else "-")
        wht = (f"{wh['pos']}/{res.get(3, '?')}" if wh and wh["pos"] != 0xFFFF else "-")
        ina = ctrl.get(CH.P_INACTIVITY)
        awake = ("계속 깨움" if ina == 0xFFFF else (f"손 떼고 {ina} 스캔 뒤 절전" if ina is not None else "?"))
        line1 = (f"{'절전 중 - 화면은 마지막 값 (손을 패널 위로)' if asleep else '수신 중'}   보드: {awake}   "
                 f"스캔 {rate:4.1f} Hz   화면 {fps:4.1f} fps (그리기 {np.mean(self.draw_ms) if self.draw_ms else 0:.0f} ms)   "
                 f"패킷 {n_pk}   체크섬 오류 {self.br.bad_checksum}")
        line2 = (f"주 버튼 {dom}   슬라이더 {slt}   휠 {wht}   "
                 f"상태: 버튼[{flags(0) or '-'}] 근접[{flags(1) or '-'}] 슬라이더[{flags(2) or '-'}] 휠[{flags(3) or '-'}]")
        line3 = (f"기준(LTA) 버튼 {np.nanmean(f['lta'][:8]):.0f}  슬라이더 {np.nanmean(f['lta'][8:12]):.0f}  "
                 f"휠 {np.nanmean(f['lta'][12:15]):.0f}  근접 {f['lta'][15]:.0f}  카운트")
        def thr_txt(names, sid):
            v = [tthr[n] for n in names if n in tthr]
            t = f"{np.mean(v):.1f}%" if v else "?"
            lta = np.nanmean([f["lta"][IDX[n]] for n in names])
            pz = f"{100 * pthr[sid] / lta:.1f}%" if sid in pthr and lta > 0 else "?"
            return f"{t}/{pz}"
        line4 = (f"판정선 터치/근접: 버튼 {thr_txt(NAMES[:8], 0)}  슬라이더 {thr_txt(NAMES[8:12], 2)}  "
                 f"휠 {thr_txt(NAMES[12:15], 3)}  근접 링 {thr_txt(['P'], 1)}")
        stuck = [n for i, n in enumerate(NAMES) if not asleep and now - since[i] > STUCK_S]
        line5 = (f"⚠ {', '.join(stuck)} 가 {STUCK_S:.0f} 초 넘게 근접/터치 상태. 손을 안 대고 있는데 이렇다면 "
                 "기준선이 틀어진 것 -> 손을 떼고 [기준선 다시 잡기]") if stuck else ""
        self.status.config(text="\n".join([line1, line2, line3, line4, line5, warn]),
                           fg="#b45309" if asleep or warn or stuck else "black")

    # ------------------------------------------------------------ 기준선 다시 잡기
    def ask_recal(self):
        self.recal_at = time.time() + RECAL_DELAY_S

    def _recal_countdown(self, now):
        if self.recal_at is None:
            return
        left = self.recal_at - now
        if left > 0:
            self.recal_lbl.config(text=f"{left:.0f} 초 뒤 - 패널에서 손을 떼세요", foreground="#dc2626")
            return
        self.recal_at = None
        self.rx.recalibrate()
        self.peak.clear()
        self.recal_lbl.config(text=f"기준선 다시 잡음 {datetime.datetime.now():%H:%M:%S}", foreground="black")

    # ------------------------------------------------------------ 기록·종료
    def toggle_record(self):
        if self.rx.writer:
            self.rx.stop_record()
            self.rec_btn.config(text="기록")
            self.rec_lbl.config(text=f"저장됨: data/{os.path.basename(self.rec_path)}", foreground="black")
        else:
            os.makedirs(DATADIR, exist_ok=True)
            self.rec_path = os.path.join(DATADIR, datetime.datetime.now().strftime("bswp_%Y%m%d_%H%M%S.csv"))
            self.rx.start_record(self.rec_path)
            self.rec_btn.config(text="기록 멈춤")

    def close(self):
        self.rx.stop_record()
        self.rx.stop.set()
        self.rx.join(timeout=1.0)
        # 깨워 둔 건 그대로 둔다: 되돌리면 다음에 켤 때 또 손으로 깨워야 한다. 데모 원래대로 = 체크 해제 또는 MCU RESET
        self.br.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        br = CH.Bridge("I2C")
    except Exception as e:
        root.withdraw()
        messagebox.showerror("CapTIvate", f"{e}")
        return
    App(root, br)
    root.mainloop()


if __name__ == "__main__":
    main()
