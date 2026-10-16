#!/usr/bin/env python3
"""
CapTIvate BSWP 패널 버튼 1 로 트랜시버 스텐트의 OOK 비트를 받아 BER 을 잰다. 송신 = transceiver/cap_ook_tx.py.

   python cap_ook_rx.py --label stent_b1           # 보드 설정 -> 기준선 -> 녹화 -> 분석·그림
   python cap_ook_rx.py --probe --label probe      # 송신 --probe 와 짝: 반송파별 반응 표
   python cap_ook_rx.py --reanalyse data/capook_stent_b1_<시각>     # 저장된 녹화를 다시 분석 (옵션 바꿔도 됨)

순서
  1) 스텐트를 버튼 1 위에 올려 두고, 송신은 끈 채로 이걸 먼저 켠다. 보드를 설정하고 기준선을 잡는다 (~5 초).
  2) "지금 송신 시작" 이 뜨면 트랜시버 쪽에서 cap_ook_tx.py 를 실행한다.
     --bps --payload --prbs (--probe 면 --probe-*) 는 양쪽을 같게. 송신 --seconds 는 여기 --tx-seconds 와 같게.
  3) 녹화가 끝나면(또는 Ctrl+C) 분석해서 data/capook_<label>_<시각>/ 에 capture.csv, meta.json, result.json,
     result.png 를 남긴다.

보드 설정 (녹화 동안만, 끝나면 원래 값으로 되돌린다. 계속 깨워 두기 C5/C4 만은 bswp_map.py 처럼 그대로 둔다):
  C2 스캔 간격 = --scan-ms (기본 20 ms = 50 Hz. 데모는 33)
  버튼 센서(ID 0): 카운트 필터 끔(89h) - 반송파가 만드는 표본 사이 흔들림을 깎지 않게
                   자동 재보정 끔(8Dh), 근접·터치 타임아웃 최대(88h), 음의 터치 임계 1000(9Bh)
                   - 송신이 count 를 위아래로 흔들어도 보드가 기준선을 다시 잡지(그동안 ~0.5 초 스캔 멈춤) 않게
  그다음 기준선을 한 번 잡는다(8Eh, 스텐트를 올려 둔 채, 송신 끈 채). --no-recal 이면 건너뜀.

수신 신호 = 버튼 1 (keypad 사이클 0 소자 0 = CAP0.0) 의 count. 반송파가 어떻게 보일지 몰라 두 특징을 다 본다:
  level  = count 평균 이동 (사각파 OOK 의 평균 전압 차, 스텐트 부하 변화). 느린 기준선은 블록 중앙값으로 뺀다
  energy = 이웃 표본 차의 제곱 (반송파가 스캔마다 다른 위상으로 찍혀 흔들리는 것)
비트 = 표본 묶음 평균. 비트 앞 표본 1 개는 버린다 (경계에 걸친 측정). 프리앰블+SFD(27 비트) 상관으로 프레임을 찾고
(모든 프레임 자리의 상관 합이 가장 큰 프레임 격자 하나), 프레임마다 헤더의 1/0 평균 가운데를 문턱으로 페이로드를 판정해
PRBS 정답지와 비교한다. 두 특징 중 유효 프레임이 많고 오류가 적은 쪽을 고른다 (--feature 로 고정 가능).
유효 프레임 = 헤더 상관 |r| >= 0.3 이고 페이로드 BER < 0.35 (잡음에서 우연히 잡힌 '동기' 를 버린다).

표본 시각: HID 로 온 호스트 시각은 수 ms 흔들리지만 보드 스캔은 보드 크리스털(32.768 kHz) 타이머로 정확히 같은 간격이다.
그래서 호스트 시각으로 스캔 주기를 최소제곱으로 맞추고 표본마다 번호를 매긴다 (빠진 스캔은 NaN -> 보간, 개수 보고).
"""
import argparse, csv, datetime, json, math, os, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                    # Visiting
sys.path[:0] = [HERE, os.path.join(ROOT, "transceiver")]
import cap_ook_tx as TX                         # 프레임·시간표 (송신과 같은 함수)
from cap_ook_tx import PREAMBLE, SFD

DATADIR = os.path.join(HERE, "data")
HDR = np.array(PREAMBLE + SFD)
GUARD = 1.0                                     # 비트 앞에서 버리는 표본 수 (비트당 표본 4 개 미만이면 0, set_guard)


def set_guard(k):
    global GUARD
    GUARD = 1.0 if k >= 4 else 0.0
R_LOCAL, BER_INVALID = 0.3, 0.35

# 녹화 동안 바꾸는 버튼 센서(ID 0) 파라미터: (명령, 녹화 중 값, 설명)
TUNE = [(0x89, 0, "카운트 필터 끔"), (0x8D, 0, "자동 재보정 끔"),
        (0x88, 0xFFFF, "타임아웃 최대"), (0x9B, 1000, "음의 터치 임계 1000")]


# ================================================================ 보드
def param(br, cmd, sid=0, value=None, b3=0, b4=0, timeout=1.5):
    """파라미터 읽기(value=None)/쓰기 -> 보드가 돌려준 값. 못 받으면 None. 기다리는 동안 온 다른 패킷은 버린다"""
    import captivate_hid as CH
    if value is None:
        br.send_packet([cmd, 0, sid, b3, b4, 0, 0])
    else:
        br.write_param(cmd, value, sid, b3, b4)
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = br.read_packet(20)
        if p is None:
            continue
        d = CH.parse(p)
        if d["kind"] == "param" and d["cmd"] == cmd and (cmd >= 0xC0 or d["sensor"] == sid):
            return d["value"]
    return None


def wait_awake(br, limit_s=120):
    """보드가 스캔 데이터를 보낼 때까지 (자고 있으면 손을 패널 위로 한 번 가져가야 깬다)"""
    import captivate_hid as CH
    br.write_param(CH.P_INACTIVITY, 0xFFFF); time.sleep(0.05)
    br.write_param(CH.P_WAKE_INTERVAL, 1)
    t0 = last = time.time()                     # 깨어 있으면 안내 없이 바로 넘어간다
    while time.time() - t0 < limit_s:
        p = br.read_packet(50)
        if p is not None and p[0] == 1:
            return True
        if time.time() - last > 5:
            last = time.time()
            print(f"  보드가 자는 중 - 패널 위로 손을 한 번 가져가 깨워 주세요 ({time.time() - t0:.0f} s)")
    return False


def setup_board(br, a):
    import captivate_hid as CH
    print("보드 설정")
    if not wait_awake(br):
        sys.exit("보드가 안 깨어난다 (USB·HID 브리지 확인)")
    orig = {"C2": param(br, CH.P_ACTIVE_PERIOD)}
    sid = getattr(a, "sensor_id", 0)            # 받는 전극의 센서 (기본 0 = 버튼)
    for cmd, _, _ in TUNE:
        orig[f"{cmd:02X}"] = param(br, cmd, sid)
    if None in orig.values():
        print(f"  경고: 일부 원래 값을 못 읽음 {orig}")
    got = {"C2": param(br, CH.P_ACTIVE_PERIOD, value=a.scan_ms)}
    for cmd, val, what in TUNE:
        got[f"{cmd:02X}"] = param(br, cmd, sid, value=val)
    orig["sensor_id"] = sid
    print(f"  스캔 간격 {orig['C2']} -> {got['C2']} ms, 센서 {sid}: " +
          ", ".join(f"{w} ({orig[f'{c:02X}']}->{got[f'{c:02X}']})" for c, _, w in TUNE))
    if not a.no_recal:
        print("  기준선 잡기 (스텐트는 올려 둔 채, 송신은 끈 채)")
        br.write_param(CH.P_FORCE_RECAL, 0, 0)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            br.read_packet(20)
    return orig, got


def restore_board(br, orig):
    import captivate_hid as CH
    if orig.get("C2"):
        param(br, CH.P_ACTIVE_PERIOD, value=orig["C2"])
    for cmd, _, _ in TUNE:
        v = orig.get(f"{cmd:02X}")
        if v is not None:
            param(br, cmd, orig.get("sensor_id", 0), value=v)
    print("보드 설정 원래대로 (계속 깨워 두기만 남김)")


def record(br, seconds):
    """버튼 사이클 0 패킷(B1-B4)을 seconds 동안. Ctrl+C 로 일찍 끝낼 수 있다."""
    import captivate_hid as CH
    T, C, L, F = [], [], [], []
    t0 = time.time(); t_print = t0 + 1.0; i_print = 0; t_prev = t0
    try:
        while time.time() - t0 < seconds:
            p = br.read_packet(50)
            if p is None or p[0] != 1 or p[1] != 0 or p[2] != 0:
                continue
            d = CH.parse(p)
            if d["kind"] != "cycle" or d["n"] != 4:
                continue
            T.append(time.time()); C.append(d["count"]); L.append(d["lta"])
            F.append(sum(int(b) << i for i, b in enumerate(d["touch"])) | sum(int(b) << (4 + i) for i, b in enumerate(d["prox"])))
            if T[-1] >= t_print:
                c = np.array(C[i_print:], float)
                print(f"  {T[-1] - t0:6.1f} s  표본 {len(T):6d} ({len(c) / max(T[-1] - t_prev, 1e-3):4.1f}/s)   "
                      f"B1 {c[:, 0].mean():6.1f} ±{c[:, 0].std():4.2f}   B2 {c[:, 1].mean():6.1f} ±{c[:, 1].std():4.2f}"
                      f"   이웃차 {np.abs(np.diff(c[:, 0])).mean() if len(c) > 1 else 0:4.2f}", flush=True)
                t_print += 1.0; i_print = len(T); t_prev = T[-1]
    except KeyboardInterrupt:
        print("  녹화 중지 (Ctrl+C)")
    return np.array(T), np.array(C, float), np.array(L, float), np.array(F, int)


# ================================================================ 표본 시각
def regularise(t, T_nom):
    """호스트 시각 t -> 표본 번호 n (빠진 스캔은 건너뛴다), 실제 스캔 주기 T, 첫 표본의 보드 시각 t0, 도착 지연 흔들림.
    호스트 도착은 실제 스캔보다 늦기만 한다(USB·브리지 지연). 그래서 도착 시각의 아래 경계(덜 늦은 표본들)에 직선을
    맞추고, 그 직선 위로 번호를 내림한다 -> 지연이 주기의 85 % 아래면 번호가 틀리지 않는다.
    (간격 반올림만 쓰면 10 ms 넘게 늦은 표본 하나가 '스캔 둘' 로 세어져 뒤가 다 밀렸다.)"""
    T = T_nom
    for _ in range(3):
        n = _number(t, T)
        Tf, t0 = np.polyfit(n, t, 1)
        r = t - (t0 + n * Tf)
        low = r <= np.percentile(r, 30)             # 덜 늦은 표본들로 주기를 맞춘다
        T, t0 = np.polyfit(n[low], t[low], 1)
    r = t - (t0 + n * T)
    return n, float(T), float(t0 + r.min()), float(np.std(r))


def _number(t, T, W=50):
    """번호 매기기: 최근 W 표본의 (도착 - 번호·T) 최솟값 = 가장 덜 늦은 도착을 기준으로 내림"""
    from collections import deque
    n = np.zeros(len(t), int)
    win = deque([(0, t[0])])                    # (번호, t - 번호·T) 의 슬라이딩 최솟값 (단조 덱)
    for i in range(1, len(t)):
        while win[0][0] < n[i - 1] - W:
            win.popleft()
        k = max(int(math.floor((t[i] - win[0][1]) / T + 0.15)), n[i - 1] + 1)
        n[i] = k
        off = t[i] - k * T
        while win and win[-1][1] >= off:
            win.pop()
        win.append((k, off))
    return n


def on_grid(n, v):
    """번호 n 의 값 v 를 고른 격자로. 빠진 곳은 선형 보간, 빠진 표시(mask) 같이"""
    N = int(n[-1]) + 1
    x = np.full(N, np.nan); x[n] = v
    miss = np.isnan(x)
    if miss.any():
        idx = np.arange(N)
        x[miss] = np.interp(idx[miss], idx[~miss], x[~miss])
    return x, miss


# ================================================================ 특징·동기·복조
def baseline(x, block):
    """block 표본 덩어리의 중앙값을 이어 붙인 느린 기준선"""
    block = max(4, int(block))
    nb = max(1, len(x) // block)
    centers = (np.arange(nb) + 0.5) * block
    meds = np.array([np.median(x[i * block:(i + 1) * block if i < nb - 1 else len(x)]) for i in range(nb)])
    return np.interp(np.arange(len(x)), centers, meds) if nb > 1 else np.full(len(x), meds[0])


def features(x):
    """표본마다의 특징. level = count 그대로 (느린 이동은 프레임 안에서 문턱이 따라간다), energy = (x[n] - x[n-1])^2,
    logenergy = log10(1 + energy / 잡음) - 흔들림 크기가 비트마다 들쭉날쭉할 때(전하 전달 주파수 근처 반송파의 맥놀이)
    energy 는 큰 값 몇 개가 '1' 평균을 끌어올려 문턱이 너무 높아졌다 (2026-10-15 S1 2.016 MHz 런, 틀린 8 개가 다 1->0)."""
    x = np.asarray(x, float)
    e = np.concatenate([[0.0], np.diff(x) ** 2])
    m = max(float(np.median(e[1:])) if len(e) > 1 else 0.0, 0.25)
    # absdev = log10(1 + (x - 기준선)^2 / 잡음): 표본 하나만으로 '흔들림 중'인지 본다 (비트당 표본이 2-3 개라 이웃 차를
    # 비트 안에서 못 만들 때). 기준선 = 50 표본 덩어리 중앙값 (켜진 동안의 출렁임은 기준선 위아래로 고르게 퍼져 중앙값을 덜 옮긴다)
    dv = (x - baseline(x, 50)) ** 2
    m2 = max(float(np.median(dv)), 0.25)
    return {"level": x, "energy": e, "logenergy": np.log10(1 + e / m), "absdev": np.log10(1 + dv / m2)}


def local_mean(v, m, W):
    """비트마다 ±W 비트 안에서 m 인 비트들의 v 평균 (없으면 NaN)"""
    cv = np.concatenate([[0.0], np.cumsum(np.where(m, v, 0.0))])
    cm = np.concatenate([[0], np.cumsum(m)])
    j = np.arange(len(v))
    a, b = np.clip(j - W, 0, len(v)), np.clip(j + W + 1, 0, len(v))
    cnt = cm[b] - cm[a]
    return np.where(cnt > 0, (cv[b] - cv[a]) / np.maximum(cnt, 1), np.nan)


def bit_means(cs, s0, k, j):
    """누적합 cs 로 비트 j(배열) 의 표본 평균. s0 = 프레임 시작 [표본, 실수], 배열이면 (s0 개수, 비트 개수)"""
    s0 = np.atleast_1d(np.asarray(s0, float))[:, None]
    N = len(cs) - 1
    a = np.clip(np.ceil(s0 + j * k + GUARD), 0, N).astype(int)
    b = np.clip(np.ceil(s0 + (j + 1) * k), 0, N).astype(int)
    return (cs[b] - cs[a]) / np.maximum(b - a, 1)


def header_corr(f, k, s0):
    """프리앰블+SFD 상관 (피어슨). 부호 = 1 일 때 특징이 커지면 +"""
    cs = np.concatenate([[0.0], np.cumsum(f)])
    H = bit_means(cs, s0, k, np.arange(len(HDR)))
    Hc = H - H.mean(axis=1, keepdims=True)
    tc = HDR - HDR.mean()
    den = np.linalg.norm(Hc, axis=1) * np.linalg.norm(tc)
    return np.where(den > 0, Hc @ tc / np.where(den > 0, den, 1), 0.0)


def decode_frame(f, k, s0, payload_true, W=16):
    """한 프레임 판정. 처음 문턱 = 헤더 1/0 평균의 가운데 (1 이 더 작으면 뒤집기). 그다음 판정 지향으로 두 번:
    아는 비트(헤더, 끝의 갭 0 16 개)와 판정한 페이로드로 비트마다 ±W 비트 안의 1/0 평균을 내서 문턱이 느린 이동을 따라간다."""
    cs = np.concatenate([[0.0], np.cumsum(f)])
    N = len(f)
    nh, npl, ng = len(HDR), len(payload_true), TX.GAP_BITS
    j = np.arange(nh + npl + ng)
    v = bit_means(cs, s0, k, j)[0]
    inside = np.ceil(s0 + (j + 1) * k) <= N         # 녹화 안에 다 들어온 비트만 (끝 갭은 잘릴 수 있다)
    on, off = v[:nh][HDR == 1].mean(), v[:nh][HDR == 0].mean()
    inv = on < off
    pl = slice(nh, nh + npl)
    # 처음 문턱: 아는 0 (헤더의 0 + 끝 갭 16 개) 으로 끔 레벨의 직선 흐름을 맞추고, 켬 = 끔 + 헤더의 1/0 차.
    # (헤더만 쓰면 34 s 프레임 동안 기준선이 흘러 약한 신호에서 첫 판정이 40 % 넘게 틀렸고 판정 지향이 못 돌아왔다)
    z = np.concatenate([np.where(HDR == 0)[0], np.arange(nh + npl, nh + npl + ng)])
    z = z[inside[z]]
    if (z >= nh).any():
        b1, b0 = np.polyfit(z, v[z], 1)
    else:
        b1, b0 = 0.0, off
    off_lin = b0 + b1 * j
    dlt = on - (b0 + b1 * np.mean(np.where(HDR == 1)[0]))
    got = (v[pl] > (off_lin + dlt / 2)[pl]) ^ inv
    bits = np.concatenate([HDR, got, np.zeros(ng, int)]).astype(bool)
    for _ in range(3):
        bits[pl] = got
        on_l = local_mean(v, bits & inside, W); off_l = local_mean(v, ~bits & inside, W)
        on_l = np.where(np.isnan(on_l), on, on_l); off_l = np.where(np.isnan(off_l), off, off_l)
        thr = (on_l + off_l) / 2
        got = (v[pl] > thr[pl]) ^ inv
    truth = np.array(payload_true, bool)
    err = int(np.sum(got != truth))
    # 채널 품질(Q)은 판정과 무관하게 정답 비트로: 비트마다 주변의 참 0 평균을 빼고 1/0 로 나눠 평균·표준편차
    tb = np.concatenate([HDR, truth, np.zeros(ng, int)]).astype(bool)
    off_t = local_mean(v, ~tb & inside, W)
    rel = (v - np.where(np.isnan(off_t), off, off_t))[pl]
    m1, m0 = rel[truth].mean(), rel[~truth].mean()
    s1, s0_ = rel[truth].std(), rel[~truth].std()
    q = abs(m1 - m0) / max(s1 + s0_, 1e-12)
    span = np.where(on_l != off_l, on_l - off_l, 1.0)[pl]
    return dict(errors=err, bits=npl, ber=err / npl, thr=float(np.mean(thr[pl])), inverted=bool(inv),
                on=float(on), off=float(off), mean1=float(m1), mean0=float(m0), std1=float(s1), std0=float(s0_),
                q=float(q), norm=((v[pl] - thr[pl]) / span).tolist(), got=got.astype(int).tolist())


def find_frames(f, k, frame_bits, payload_true, step=0.25):
    """프레임 격자 찾기. 송신은 Teensy 크리스털로 프레임을 정확히 P = frame_bits·k 표본마다 되풀이하므로, 프레임마다 따로
    찾지 않고 모든 프레임 자리의 헤더 상관 |r| 합이 가장 큰 격자 위상 하나를 고른다 (프레임마다 따로 찾으면 1-2 표본씩
    흔들려 경계 표본이 옆 비트에 섞였다). 그다음 격자 위의 프레임을 다 판정한다."""
    N = len(f)
    P = frame_bits * k
    last = N - (len(HDR) + len(payload_true)) * k - 1
    if last <= 0:
        return dict(frames=[], grid=None)
    grid = np.arange(0, last, step)
    r = header_corr(f, k, grid)
    ra = np.abs(r)
    i0 = int(np.argmax(ra))
    phases = np.arange(0, P, step)
    score = np.zeros(len(phases))
    for m in range(int(last / P) + 1):
        idx = np.round((phases + m * P) / step).astype(int)
        ok = idx < len(grid)
        score[ok] += ra[idx[ok]]
    ph = phases[int(np.argmax(score))]
    out = []
    for m in range(int(last / P) + 1):
        s0 = ph + m * P
        if s0 > last:
            break
        ri = float(r[min(int(round(s0 / step)), len(r) - 1)])
        d = decode_frame(f, k, s0, payload_true)
        d.update(m=int(m), s0=float(s0), r=ri, valid=bool(abs(ri) >= R_LOCAL and d["ber"] < BER_INVALID))
        out.append(d)
    return dict(frames=out, grid=(grid, r), frame_period=float(P), anchor_r=float(r[i0]))


def analyse(x, T, link, feature="auto"):
    """x = 버튼 1 count (고른 격자), T = 스캔 주기 [s], link = 송신과 같은 옵션 -> 결과 dict"""
    fw, plan = TX.build(link)
    _, payload_true = TX.frame_bits(link.payload, link.prbs)
    k = (1.0 / plan["bps"]) / T                  # 비트당 표본 (실수)
    set_guard(k)
    feats = features(x)
    res = {}
    for name, f in feats.items():
        fr = find_frames(f, k, plan["frame_bits"], payload_true)
        valid = [d for d in fr["frames"] if d["valid"]]
        bits = sum(d["bits"] for d in valid); errs = sum(d["errors"] for d in valid)
        res[name] = dict(frames=fr["frames"], frame_period=fr.get("frame_period"), anchor_r=fr.get("anchor_r"),
                         n_valid=len(valid), bits=bits, errors=errs, ber=errs / bits if bits else None,
                         q_mean=float(np.mean([d["q"] for d in valid])) if valid else None,
                         r_mean=float(np.mean([abs(d["r"]) for d in valid])) if valid else None, _grid=fr["grid"])
    if feature == "auto":
        feature = max(res, key=lambda n: (res[n]["n_valid"], -(res[n]["ber"] if res[n]["ber"] is not None else 1),
                                          res[n]["r_mean"] or 0))
    sel = res[feature]
    out = dict(plan=plan, T_s=T, fs_hz=1 / T, samples_per_bit=k, feature=feature,
               features={n: {kk: vv for kk, vv in v.items() if kk != "_grid"} for n, v in res.items()})
    if sel["bits"]:
        q = sel["q_mean"]
        out.update(bits=sel["bits"], errors=sel["errors"], ber=sel["ber"],
                   ber_upper95=(3.0 / sel["bits"] if sel["errors"] == 0 else None),
                   q=q, ber_from_q=0.5 * math.erfc(q / math.sqrt(2)))
        valid = [d for d in sel["frames"] if d["valid"]]
        if feature == "level":
            out["contrast_counts"] = float(np.mean([d["mean1"] - d["mean0"] for d in valid]))
        else:                                       # mean1/0 은 끔 레벨 기준이라 비율은 헤더 평균으로
            out["energy_ratio_db"] = float(np.mean([10 * np.log10(max(d["on"], 1e-12) / max(d["off"], 1e-12))
                                                    for d in valid]))
    out["_grid"] = sel["_grid"]
    out["_feats"] = feats
    return out


# ================================================================ 반송파 고르기
def analyse_probe(x, T, link, tau_hint=None, hint_tol_s=1.5):
    """송신 --probe 시간표를 녹화에 맞춰(시작 시각 탐색) 반송파마다 켬/끔 차이를 잰다.
    tau_hint = 송신 로그로 아는 시작 표본 (있으면 그 ±hint_tol_s 안에서만 찾는다)"""
    _, plan = TX.probe_plan(link)
    fs = 1 / T
    nblk = plan["block_s"] * fs
    ncar, reps = len(plan["carriers_hz"]), plan["reps"]
    L = int(round(plan["total_s"] * fs))
    if len(x) < L + 2:
        return dict(plan=plan, error=f"녹화가 시간표({plan['total_s']:.0f} s)보다 짧다")
    x = np.asarray(x, float)
    e = features(x)["energy"]
    # 덩어리(반송파 ci, 반복 rr, 켬 st=0 / 끔 st=1) 마다 표본 구간. 앞뒤 15 % 는 경계라 뺀다
    nmk = plan["marker_s"] * fs                 # 맨 앞 표지 (ci = -1): nmk 켬, nmk 끔

    def make_blocks(frac):
        """(ci, st, 시작, 끝) 표본. 앞뒤 frac 만큼(최소 1 표본)은 경계라 뺀다"""
        g, gm = max(1.0, frac * nblk), max(1.0, frac * nmk)
        bl = [(-1, st, int(math.ceil(st * nmk + gm)), int(math.floor((st + 1) * nmk - gm))) for st in (0, 1)]
        bl += [(ci, st, int(math.ceil(b0 + g)), int(math.floor(b0 + nblk - g)))
               for ci in range(ncar) for rr in range(reps) for st in (0, 1)
               for b0 in [2 * nmk + (ci * 2 * reps + 2 * rr + st) * nblk]]
        return bl

    blocks = make_blocks(0.15)                  # 대충 맞출 때: 15 % 버림 (몇 표본 틀려도 섞이지 않게)
    cx = np.concatenate([[0.0], np.cumsum(x)]); ce = np.concatenate([[0.0], np.cumsum(e)])
    taus = np.arange(0, len(x) - L)
    if tau_hint is not None:
        near = taus[np.abs(taus - tau_hint) <= hint_tol_s * fs]
        taus = near if len(near) else taus
    # 잡음 크기 (덩어리 평균 하나의 표준편차): 이웃 표본 차로 어림. count 가 정수라 차의 대부분이 0 일 수 있어
    # 중앙값 절대편차는 0 이 된다 -> 표준편차를 쓴다 (한 번 0 이 되어 level 점수가 무한대로 맞춤을 망쳤다)
    w = blocks[0][3] - blocks[0][2]
    sx = max(np.std(np.diff(x)) / math.sqrt(2), 0.1) / math.sqrt(w)
    se = (np.std(e) + 1e-9) / math.sqrt(w)

    def contrasts(tau, blocks=blocks):
        """시작 tau(배열) 에서 반송파마다 (켬 - 끔) level·energy 평균 차. 마지막 줄(ci = -1) = 표지"""
        tau = np.atleast_1d(tau)
        dl = np.zeros((ncar + 1, len(tau))); de = np.zeros((ncar + 1, len(tau)))
        for ci, st, a_, b_ in blocks:
            sgn = (1 if st == 0 else -1) / (1 if ci < 0 else reps)
            dl[ci] += sgn * (cx[tau + b_] - cx[tau + a_]) / (b_ - a_)
            de[ci] += sgn * (ce[tau + b_] - ce[tau + a_]) / (b_ - a_)
        return dl, de

    # 시작 시각: 반송파마다의 켬/끔 차(잡음 대비)의 제곱합이 가장 큰 곳 = 실제로 반응하는 반송파들이 맞춘다
    dl, de = contrasts(taus)
    sm = math.sqrt(reps * nblk / nmk)           # 표지는 덩어리 하나라 잡음이 다르다 (반복 없음, 길이 다름)
    score = ((dl[:-1] / sx) ** 2 + (de[:-1] / se) ** 2).sum(axis=0) + (dl[-1] / (sx * sm)) ** 2 + (de[-1] / (se * sm)) ** 2
    tau = int(taus[np.argmax(score)])
    # 정밀하게: 경계를 1 표본만 버리고 대충 맞춘 곳 ±(15 % + 2) 표본 안에서 다시 (15 % 버림은 그 안에서 점수가 평평하다)
    fine = make_blocks(0.0)
    near = np.arange(max(0, tau - int(0.15 * nblk) - 2), min(len(x) - L - 1, tau + int(0.15 * nblk) + 2) + 1)
    dlf, def_ = contrasts(near, fine)
    sf = ((dlf[:-1] / sx) ** 2 + (def_[:-1] / se) ** 2).sum(axis=0) + (dlf[-1] / (sx * sm)) ** 2 + (def_[-1] / (se * sm)) ** 2
    tau = int(near[np.argmax(sf)])
    rows = []
    dl1, de1 = contrasts(tau)
    for ci, fc in enumerate(plan["carriers_hz"]):
        on = np.concatenate([np.arange(a_, b_) for c_, st, a_, b_ in blocks if c_ == ci and st == 0]) + tau
        off = np.concatenate([np.arange(a_, b_) for c_, st, a_, b_ in blocks if c_ == ci and st == 1]) + tau
        er = 10 * np.log10(max(e[on].mean(), 1e-12) / max(e[off].mean(), 1e-12))
        rows.append(dict(carrier_hz=fc, level_counts=float(dl1[ci, 0]), level_z=float(dl1[ci, 0] / (sx / math.sqrt(reps))),
                         energy_db=float(er), rms_on=float(np.sqrt(e[on].mean() / 2)), rms_off=float(np.sqrt(e[off].mean() / 2))))
    c = score
    best_l = max(rows, key=lambda r: abs(r["level_z"]))
    best_e = max(rows, key=lambda r: r["energy_db"])
    marker = dict(level_counts=float(dl1[-1, 0]), energy_db=float(10 * np.log10(
        max(np.mean(e[tau + blocks[0][2]:tau + blocks[0][3]]), 1e-12) / max(np.mean(e[tau + blocks[1][2]:tau + blocks[1][3]]), 1e-12))))
    return dict(plan=plan, T_s=T, tau_samples=tau, tau_s=tau * T, tau_hint=tau_hint, marker=marker,
                align_peak=float((c.max() - np.median(c)) / (1.4826 * np.median(np.abs(c - np.median(c))) + 1e-12)),
                rows=rows, best_level=best_l["carrier_hz"], best_energy=best_e["carrier_hz"])


# ================================================================ 그림
def plot_data(res, t, X, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    matplotlib.rcParams["font.family"] = ["AppleGothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    T, k, plan = res["T_s"], res["samples_per_bit"], res["plan"]
    sel = res["features"][res["feature"]]
    valid = [d for d in sel["frames"] if d["valid"]]
    fig, ax = plt.subplots(2, 2, figsize=(14, 8))
    a = ax[0, 0]
    a.plot(t, X[:, 0], lw=0.6, color="0.3", label="B1 count")
    for d in sel["frames"]:
        a.axvline(d["s0"] * T, color="tab:green" if d["valid"] else "tab:red", lw=1, alpha=0.7)
    a.set_title(f"버튼 1 count 전체 (세로선 = 찾은 프레임 시작: 초록 유효 {len(valid)} / 빨강 무효)", fontsize=10, loc="left")
    a.set_xlabel("시간 [s]"); a.set_ylabel("count")
    a = ax[0, 1]
    if valid:
        d = valid[0]
        nb = min(len(HDR) + 40, len(HDR) + plan["payload"])
        t0s, t1s = d["s0"] * T, (d["s0"] + nb * k) * T
        m = (t >= t0s - 1) & (t <= t1s + 1)
        a.plot(t[m], X[m, 0], ".-", lw=0.7, ms=3, color="k", label="B1")
        bits = list(HDR) + TX.frame_bits(plan["payload"], plan["prbs"])[1]
        ylo, yhi = X[m, 0].min(), X[m, 0].max()
        for j in range(nb):
            if bits[j]:
                a.axvspan((d["s0"] + j * k) * T, (d["s0"] + (j + 1) * k) * T, color="tab:orange", alpha=0.18, lw=0)
            if j >= len(HDR):
                ok = d["got"][j - len(HDR)] == bits[j]
                a.text((d["s0"] + (j + 0.5) * k) * T, yhi + 0.03 * (yhi - ylo + 1), "" if ok else "x",
                       ha="center", color="tab:red", fontsize=9)
        a.axvline((d["s0"] + len(HDR) * k) * T, color="tab:blue", lw=1, ls="--")
        a.set_title(f"첫 유효 프레임 앞부분 (주황 = 보낸 1, 파란 점선 = 페이로드 시작, x = 틀린 비트)", fontsize=10, loc="left")
        a.set_xlabel("시간 [s]")
    a = ax[1, 0]
    if valid:
        nz = np.concatenate([np.array(d["norm"]) for d in valid])
        tr = np.concatenate([np.array(TX.frame_bits(plan["payload"], plan["prbs"])[1], bool) for _ in valid])
        bins = np.linspace(min(nz.min(), -1.5), max(nz.max(), 1.5), 50)
        a.hist(nz[~tr], bins, alpha=0.6, label="보낸 0", color="tab:blue")
        a.hist(nz[tr], bins, alpha=0.6, label="보낸 1", color="tab:orange")
        a.axvline(0, color="k", lw=1)
        a.legend(fontsize=9)
    a.set_title(f"페이로드 비트 판정값 ({res['feature']}, 0 = 문턱, ±0.5 = 헤더 1/0 평균)", fontsize=10, loc="left")
    a = ax[1, 1]
    if valid:
        d = valid[0]
        t0s, t1s = d["s0"] * T, (d["s0"] + (len(HDR) + 40) * k) * T
        m = (t >= t0s - 1) & (t <= t1s + 1)
        for i, nm in enumerate(["B1", "B2", "B3", "B4"]):
            a.plot(t[m], X[m, i] - np.median(X[m, i]), lw=0.8, label=nm)
        a.legend(fontsize=8, ncol=4)
    a.set_title("같은 구간 이웃 버튼 (중앙값 뺀 count) - 송신이 옆 버튼으로 새는지", fontsize=10, loc="left")
    a.set_xlabel("시간 [s]")
    ber = res.get("ber")
    fig.suptitle(f"{plan['wave']} {plan['carrier_hz']:.1f} Hz, {plan['bps']:g} bps, 스캔 {1 / T:.2f} Hz (비트당 {k:.2f} 표본)   "
                 f"특징 {res['feature']}   " +
                 (f"BER {res['errors']}/{res['bits']} = {ber:.2e}" if ber is not None else "프레임 못 찾음"), fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_probe(pr, t, x, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    matplotlib.rcParams["font.family"] = ["AppleGothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    rows = pr["rows"]
    fc = [r["carrier_hz"] for r in rows]
    fig, ax = plt.subplots(3, 1, figsize=(12, 10))
    ax[0].plot(t, x, lw=0.6, color="0.3")
    T, plan = pr["T_s"], pr["plan"]
    t_s = pr["tau_s"] + t[0]
    ax[0].axvspan(t_s, t_s + plan["marker_s"], color="k", alpha=0.15, lw=0)
    ax[0].text(t_s + plan["marker_s"] / 2, np.max(x), "표지", ha="center", va="top", fontsize=7)
    c0 = t_s + 2 * plan["marker_s"]
    for ci, f in enumerate(plan["carriers_hz"]):
        for rr in range(plan["reps"]):
            b0 = c0 + (ci * 2 * plan["reps"] + 2 * rr) * plan["block_s"]
            ax[0].axvspan(b0, b0 + plan["block_s"], color=f"C{ci % 10}", alpha=0.2, lw=0)
        ax[0].text(c0 + (ci * 2 * plan["reps"] + plan["reps"]) * plan["block_s"], np.max(x),
                   f"{f / 1e3:g}k" if f >= 1e3 else f"{f:g}", ha="center", va="top", fontsize=7)
    ax[0].set_title("버튼 1 count 와 맞춘 송신 시간표 (회색 = 표지 켬, 색칠 = 반송파 켬)" +
                    ("  - 송신 로그 시각으로 맞춤" if pr.get("tau_hint") is not None else ""), fontsize=10, loc="left")
    ax[1].semilogx(fc, [r["level_counts"] for r in rows], "o-")
    ax[1].axhline(0, color="0.6", lw=0.8)
    ax[1].set_ylabel("켬 - 끔 평균 [count]"); ax[1].set_title("level: 켰을 때 평균 이동", fontsize=10, loc="left")
    ax[2].semilogx(fc, [r["energy_db"] for r in rows], "o-", color="tab:red")
    ax[2].axhline(0, color="0.6", lw=0.8)
    ax[2].set_ylabel("이웃 차 에너지 켬/끔 [dB]"); ax[2].set_xlabel("반송파 [Hz]")
    ax[2].set_title("energy: 켰을 때 표본 사이 흔들림", fontsize=10, loc="left")
    for a in ax[1:]:
        a.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ================================================================ 저장·불러오기
def save_capture(d, t, n, C, L, F, meta):
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "capture.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["n", "t_unix"] + [f"B{i}_count" for i in range(1, 5)] + [f"B{i}_lta" for i in range(1, 5)] + ["flags"])
        for i in range(len(t)):
            w.writerow([int(n[i]), f"{t[i]:.4f}"] + [int(v) for v in C[i]] + [int(v) for v in L[i]] + [int(F[i])])
    json.dump(meta, open(os.path.join(d, "meta.json"), "w"), indent=1, ensure_ascii=False)


def load_capture(d):
    a = np.genfromtxt(os.path.join(d, "capture.csv"), delimiter=",", names=True)
    meta = json.load(open(os.path.join(d, "meta.json")))
    C = np.vstack([a[f"B{i}_count"] for i in range(1, 5)]).T
    L = np.vstack([a[f"B{i}_lta"] for i in range(1, 5)]).T
    return a["t_unix"], C, L, a["flags"].astype(int), meta


def link_ns(a, tx_seconds=None):
    return argparse.Namespace(carrier=a.carrier, bps=a.bps, payload=a.payload, prbs=a.prbs, wave=a.wave,
                              seconds=tx_seconds if tx_seconds is not None else a.tx_seconds,
                              probe_carriers=a.probe_carriers, probe_block=a.probe_block, probe_reps=a.probe_reps)


def find_tx_log(a, t, probe):
    """이 녹화와 겹치는 송신 로그 (transceiver/data/capook_tx_*.json). 같은 맥이거나 두 노트북 시계가 인터넷으로 맞으면
    첫 비트 시각을 알 수 있다. --tx-log 로 직접 줄 수도 있다."""
    import glob
    paths = [a.tx_log] if getattr(a, "tx_log", None) else sorted(glob.glob(os.path.join(ROOT, "transceiver", "data", "capook_tx_*.json")))
    best = None
    for pth in paths:
        try:
            lg = json.load(open(pth))
        except Exception:
            continue
        fb = lg.get("first_bit_unix")
        if fb is None or (lg["plan"]["kind"] == "cap_ook_probe") != bool(probe):
            continue
        if a.tx_log or t[0] - 5 <= fb <= t[-1]:
            best = (pth, lg)
    return best


def run_analysis(d, t, C, meta, a):
    n, T, t0, jit = regularise(t, meta["scan_ms_set"] / 1e3)
    x, miss = on_grid(n, C[:, 0])
    Xg = np.column_stack([on_grid(n, C[:, i])[0] for i in range(4)])
    tg = np.arange(len(x)) * T
    link = link_ns(a)
    print(f"표본 {len(t)} 개 -> 격자 {len(x)} (빠짐 {int(miss.sum())}), 스캔 주기 {T * 1e3:.4f} ms ({1 / T:.3f} Hz), "
          f"도착 흔들림 {jit * 1e3:.1f} ms")
    txl = find_tx_log(a, t, a.probe)
    if txl:
        print(f"송신 로그: {os.path.relpath(txl[0], ROOT)} - 첫 비트가 녹화 {txl[1]['first_bit_unix'] - t0:.2f} s 쯤")
    if a.probe:
        hint = (txl[1]["first_bit_unix"] - t0) / T if txl else None
        pr = analyse_probe(x, T, link, tau_hint=hint)
        if "error" in pr:
            print(pr["error"]); return pr
        pr["tx_log"] = txl[0] if txl else None
        print(f"시간표 맞춤: 녹화 {pr['tau_s']:.2f} s 에서 시작 (점수 봉우리 {pr['align_peak']:.1f} σ"
              + (f", 송신 로그와 {pr['tau_s'] - hint * T:+.2f} s" if hint is not None else ", 송신 로그 없이 전체 탐색") + ")")
        mk = pr["marker"]
        print(f"표지 {pr['plan']['marker_hz']:.1f} Hz: level {mk['level_counts']:+.2f} count, energy {mk['energy_db']:+.1f} dB")
        print(f"{'반송파 Hz':>10s} {'level 이동':>10s} {'z':>6s} {'energy dB':>9s} {'흔들림 켬/끔 rms':>16s}")
        for r in pr["rows"]:
            print(f"{r['carrier_hz']:10.1f} {r['level_counts']:+10.2f} {r['level_z']:6.1f} {r['energy_db']:9.1f}"
                  f"   {r['rms_on']:5.2f} / {r['rms_off']:5.2f}")
        print(f"추천: level 이 가장 큰 반송파 {pr['best_level']:g} Hz, energy 가 가장 큰 반송파 {pr['best_energy']:g} Hz")
        json.dump(pr, open(os.path.join(d, "result.json"), "w"), indent=1, ensure_ascii=False)
        plot_probe(pr, tg, x, os.path.join(d, "result.png"))
        return pr
    res = analyse(x, T, link, a.feature)
    grid = res.pop("_grid"); res.pop("_feats")
    res["tx_log"] = txl[0] if txl else None
    if txl and res.get("bits"):
        sel = [f for f in res["features"][res["feature"]]["frames"] if f["valid"]]
        k_ = res["samples_per_bit"]
        exp = (txl[1]["first_bit_unix"] - t0) / T
        dif = [(f["s0"] - exp) / k_ - round((f["s0"] - exp) / (res["plan"]["frame_bits"] * k_)) * res["plan"]["frame_bits"] for f in sel]
        print(f"  찾은 프레임 위치 - 송신 로그 시각 = {np.median(dif):+.2f} 비트 (두 컴퓨터 시계 차 + USB 지연)")
    res["missing_samples"] = int(miss.sum())
    for name, v in res["features"].items():
        print(f"  [{name:6s}] 프레임 {v['n_valid']}/{len(v['frames'])} 유효, 비트 {v['bits']}, 오류 {v['errors']}"
              + (f", BER {v['ber']:.2e}, Q {v['q_mean']:.2f}, |r| {v['r_mean']:.2f}" if v["bits"] else
                 f"  (가장 센 헤더 상관 {v['anchor_r']:+.2f})" if v["anchor_r"] is not None else ""))
    if res.get("bits"):
        b = res["ber"]
        print(f"결과 ({res['feature']}): BER {res['errors']}/{res['bits']} = {b:.2e}"
              + (f" (오류 0 -> 95% 상한 {res['ber_upper95']:.1e})" if res["errors"] == 0 else "")
              + f",  Q {res['q']:.2f} -> 가우스 가정 BER {res['ber_from_q']:.1e}")
        if "contrast_counts" in res:
            lta = float(np.median(meta.get("b1_lta", [np.nan]))) if meta.get("b1_lta") else np.median(C[:, 0])
            print(f"  1 일 때 버튼 1 count 이동 {res['contrast_counts']:+.2f} ({100 * res['contrast_counts'] / lta:+.2f} % of {lta:.0f})")
        if "energy_ratio_db" in res:
            print(f"  1 일 때 이웃 표본 차 에너지 {res['energy_ratio_db']:+.1f} dB")
    else:
        print("프레임을 못 찾음 - 신호가 안 보이거나 송신 옵션(--bps --payload --prbs)이 다르다. --probe 로 반송파부터 보길 권함")
    json.dump(res, open(os.path.join(d, "result.json"), "w"), indent=1, ensure_ascii=False)
    plot_data(res, tg, Xg, os.path.join(d, "result.png"))
    return res


# ================================================================ main
def main():
    ap = argparse.ArgumentParser(description="CapTIvate 버튼 1 로 스텐트 OOK 받기 + BER")
    TX.add_link_args(ap)
    ap.add_argument("--wave", choices=("square", "sine"), default="square", help="기록용 (송신과 같게)")
    ap.add_argument("--tx-seconds", type=float, default=120, help="송신 --seconds 와 같게 (기본 120)")
    ap.add_argument("--seconds", type=float, help="녹화 길이 (기본 = 송신 시간표 + 25 s)")
    ap.add_argument("--scan-ms", type=int, default=20, help="보드 스캔 간격 (기본 20 ms = 50 Hz)")
    ap.add_argument("--feature", choices=("auto", "level", "energy"), default="auto")
    ap.add_argument("--no-recal", action="store_true", help="시작할 때 기준선 잡기 건너뜀")
    ap.add_argument("--label", default="stent_b1")
    ap.add_argument("--no-open", action="store_true", help="끝나고 그림을 열지 않음")
    ap.add_argument("--reanalyse", metavar="DIR", help="저장된 녹화 폴더를 다시 분석")
    ap.add_argument("--tx-log", help="송신 로그 json (기본: transceiver/data 에서 녹화와 겹치는 것을 찾음)")
    a = ap.parse_args()
    if a.reanalyse:                             # 녹화 때 옵션을 기본값으로 (명령줄에 준 것만 덮어쓴다)
        m = json.load(open(os.path.join(a.reanalyse, "meta.json")))
        lk = dict(m["link"]); lk["tx_seconds"] = lk.pop("seconds")
        ap.set_defaults(probe=m["probe"], **lk)
        a = ap.parse_args()

    if a.reanalyse:
        t, C, L, F, meta = load_capture(a.reanalyse)
        res = run_analysis(a.reanalyse, t, C, meta, a)
        if not a.no_open:
            subprocess.run(["open", os.path.join(a.reanalyse, "result.png")])
        return

    _, plan = (TX.probe_plan if a.probe else TX.build)(link_ns(a))
    secs = a.seconds or plan["total_s"] + 25
    import captivate_hid as CH
    br = CH.Bridge("I2C")
    orig = {}
    try:
        orig, got = setup_board(br, a)
        print(f"\n지금 송신 시작 -> 트랜시버 쪽: python cap_ook_tx.py" + (" --probe" if a.probe else "") +
              f"  (같은 옵션)   녹화 {secs:.0f} s (Ctrl+C 로 일찍 끝내도 분석함)\n")
        t, C, L, F = record(br, secs)
    finally:
        restore_board(br, orig)
        br.close()
    if len(t) < 10:
        sys.exit("받은 표본이 거의 없다")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    d = os.path.join(DATADIR, f"capook_{'probe_' if a.probe else ''}{a.label}_{stamp}")
    meta = dict(label=a.label, probe=a.probe, scan_ms_set=a.scan_ms, link=vars(link_ns(a)), plan=plan,
                board_orig=orig, board_set=got, b1_lta=float(np.median(L[:, 0])), start_unix=float(t[0]),
                recorded_s=float(t[-1] - t[0]))
    n, _, _, _ = regularise(t, a.scan_ms / 1e3)
    save_capture(d, t, n, C, L, F, meta)
    print(f"녹화 저장: {os.path.relpath(d, ROOT)}")
    run_analysis(d, t, C, meta, a)
    print(f"결과: {os.path.relpath(os.path.join(d, 'result.png'), ROOT)}")
    if not a.no_open:
        subprocess.run(["open", os.path.join(d, "result.png")])


if __name__ == "__main__":
    main()
