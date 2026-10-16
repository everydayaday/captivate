#!/usr/bin/env python3
"""
cap_ook_rx 분석부 합성 자가검증. 보드 없이 돈다.

    python test_cap_ook_rx.py

가짜 버튼 1 count 를 만든다: 보드 스캔 19.989 ms (32.768 kHz 크리스털로 655 틱), 송신 비트 시계 +40 ppm,
녹화 7.3 s 뒤 송신 시작, 정수 count, 느린 드리프트, 호스트 도착 시각 흔들림(0-3 ms, 가끔 몰림), 스캔 0.3 % 빠짐.
경우: level 형(1 일 때 count 가 Δ 만큼 내려감), energy 형(1 일 때 반송파가 스캔마다 다른 위상으로 찍힘),
약한 level (BER 이 가우스 예측과 맞는지), 잡음만 (프레임을 잘못 잡지 않는지), 반송파 고르기.
"""
import argparse, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cap_ook_rx as RX

T_BOARD = 655 / 32768.0
LEAD_S = 7.3


def link(**kw):
    a = argparse.Namespace(carrier=1012.5, bps=5.0, payload=127, prbs=7, wave="square", seconds=120,
                           probe_carriers=RX.TX.PROBE_CARRIERS, probe_block=1.0, probe_reps=3)
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def tx_wave(t, lk, ppm=40.0):
    """시각 t 에서 송신 상태 (0/1)와 반송파 위상"""
    _, plan = RX.TX.build(lk)
    bits, _ = RX.TX.frame_bits(lk.payload, lk.prbs)
    tb = (t - LEAD_S) * (1 + ppm * 1e-6)
    j = np.floor(tb / (plan["bit_us"] / 1e6)).astype(int)
    on = np.zeros(len(t))
    ok = (j >= 0) & (j < len(bits) * plan["n_frames"])
    on[ok] = np.array(bits)[j[ok] % len(bits)]
    return on


def synth(lk, kind, amp, sigma, seed=1, extra_s=25):
    rng = np.random.default_rng(seed)
    _, plan = RX.TX.build(lk)
    N = int((LEAD_S + plan["total_s"] + extra_s) / T_BOARD)
    tn = np.arange(N) * T_BOARD
    tm = tn + 60e-6                             # 버튼 1 측정 시각 (스캔 시작 뒤, 125 us 창 가운데쯤)
    on = tx_wave(tm, lk)
    x = 248 + 0.8 * np.sin(2 * np.pi * tn / 90) + rng.normal(0, sigma, N)
    if kind == "level":
        x -= amp * on
    else:
        x += amp * on * np.sin(2 * np.pi * lk.carrier * tm + 0.7)
    C = np.round(x)
    keep = rng.random(N) > 0.003
    host = 1.7e9 + tn + rng.exponential(1e-3, N)
    burst = rng.random(N) < 0.01               # 가끔 몇 ms 늦게 몰려 도착
    host[burst] += rng.uniform(3e-3, 8e-3, burst.sum())
    host = np.maximum.accumulate(host)
    return host[keep], C[keep], plan


def run(lk, C, host, probe=False):
    n, T, t0, jit = RX.regularise(host, 0.020)
    x, miss = RX.on_grid(n, C)
    if probe:
        return RX.analyse_probe(x, T, lk), T, miss.sum()
    res = RX.analyse(x, T, lk)
    res.pop("_grid"); res.pop("_feats")
    return res, T, miss.sum()


def main():
    ok = True
    lk = link()
    for name, kind, amp, sig, want in [("level 센 신호", "level", 3.0, 0.5, "zero"),
                                       ("energy 센 신호", "energy", 3.0, 0.5, "zero"),
                                       ("level 약한 신호", "level", 0.9, 0.6, "match")]:
        host, C, plan = synth(lk, kind, amp, sig)
        res, T, nmiss = run(lk, C, host)
        f = res["features"][res["feature"]]
        line = (f"{name:14s} 주기 {T * 1e3:.4f} ms (참 {T_BOARD * 1e3:.4f}), 빠짐 {nmiss}, 고른 특징 {res['feature']}, "
                f"프레임 {f['n_valid']}/{plan['n_frames']}, BER {res.get('errors')}/{res.get('bits')}")
        if want == "zero":
            fam = {"level": {"level"}, "energy": {"energy", "logenergy", "absdev"}}[kind]
            good = res["feature"] in fam and f["n_valid"] == plan["n_frames"] and res.get("errors") == 0
        else:
            pred = res["ber_from_q"]; got = res["ber"]
            line += f" = {got:.3f}, Q 예측 {pred:.3f}"
            good = res["feature"] == kind and f["n_valid"] >= plan["n_frames"] - 1 and 0.3 * pred <= got <= 3 * pred + 0.02
        ok &= good
        print(("통과 " if good else "실패 ") + line)
    # 잡음만: 유효 프레임이 없어야 한다
    host, C, plan = synth(lk, "level", 0.0, 0.6, seed=5)
    res, T, _ = run(lk, C, host)
    nv = max(v["n_valid"] for v in res["features"].values())
    good = nv == 0
    ok &= good
    print(("통과 " if good else "실패 ") + f"잡음만         유효 프레임 {nv} (0 이어야 함), 가장 센 헤더 상관 "
          + ", ".join(f"{k} {v['anchor_r']:+.2f}" for k, v in res["features"].items()))
    # 반송파 고르기: 1012.5 Hz 와 2012.5 Hz 만 반응 (energy), 정렬과 순위 확인
    pk = link(probe=True)
    _, pplan = RX.TX.probe_plan(pk)
    rng = np.random.default_rng(3)
    N = int((LEAD_S + pplan["total_s"] + 20) / T_BOARD)
    tn = np.arange(N) * T_BOARD
    x = 248 + rng.normal(0, 0.5, N)
    resp_of = lambda fc: {1012.534: 3.0, 2012.495: 1.5}.get(round(fc, 3), 0.0)
    m = (tn >= LEAD_S) & (tn < LEAD_S + pplan["marker_s"])          # 맨 앞 표지 (= --carrier)
    x[m] += resp_of(pplan["marker_hz"]) * np.sin(2 * np.pi * pplan["marker_hz"] * tn[m])
    for ci, fc in enumerate(pplan["carriers_hz"]):
        for rr in range(pplan["reps"]):
            b0 = LEAD_S + 2 * pplan["marker_s"] + (ci * 2 * pplan["reps"] + 2 * rr) * pplan["block_s"]
            m = (tn >= b0) & (tn < b0 + pplan["block_s"])
            x[m] += resp_of(fc) * np.sin(2 * np.pi * fc * tn[m] + 0.3)
    host = 1.7e9 + tn + rng.exponential(1e-3, N)
    for hint in (None, (LEAD_S + 0.4) / T_BOARD):       # 송신 로그 시각이 없을 때 / 0.4 s 틀린 로그가 있을 때
        n, T, t0, jit = RX.regularise(host, 0.020)
        x_, _ = RX.on_grid(n, np.round(x)[:len(host)])
        pr = RX.analyse_probe(x_, T, pk, tau_hint=hint)
        good = abs(pr["tau_s"] - LEAD_S) < 0.1 and abs(pr["best_energy"] - 1012.534) < 1
        ok &= good
        top = sorted(pr["rows"], key=lambda r: -r["energy_db"])[:3]
        print(("통과 " if good else "실패 ") + f"반송파 고르기 ({'로그 시각 있음' if hint else '로그 없음'}) 시작 {pr['tau_s']:.2f} s "
              f"(참 {LEAD_S}), 표지 {pr['marker']['energy_db']:+.1f} dB, energy 상위 " +
              ", ".join(f"{r['carrier_hz']:.0f} Hz {r['energy_db']:+.1f} dB" for r in top))
    print("모두 통과" if ok else "실패 있음")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
