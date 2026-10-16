#!/usr/bin/env python3
"""
CAPTIVATE-PGMR 의 HID 브리지로 CapTIvate MCU(CAPTIVATE-FR2633) 패킷을 받는다. Design Center 없이 맥에서 바로.

   python captivate_hid.py              # 브리지를 기본 데모(BULK I2C)에 맞추고, 들어오는 패킷을 풀어서 찍는다
   python captivate_hid.py --raw        # 풀지 않고 16진수로
   python captivate_hid.py --uart       # 펌웨어를 UART 통신으로 바꿔 구웠을 때

USB 쪽 (2026-10-14 실측, 브리지 VERSION=1.00.00.00, PLATFORM=MSP-EXP430PGMR):
  VID 0x2047 / PID 0x0965 "MCU COMM HID to Serial Bridge" 에 HID 인터페이스 2개
    0번 = 데이터 (보드 <-> PC, 바이너리)      1번 = 브리지 설정 콘솔 ("HB ..." ASCII 명령, 답도 ASCII)
  HID 보고서 64 바이트 = [3Fh, 길이 n, 데이터 n 바이트, 나머지는 쓰레기]
  공장 상태는 "MODE=PACKET INTERFACE=UART" 였다. 기본 데모는 BULK I2C(슬레이브 0x0A, 400 kHz)라서
  "HB MODE PACKET I2C" 를 보내야 데이터가 온다 (전원을 뽑으면 되돌아간다. SAVECONFIG 는 안 한다).
  PACKET 모드에서는 보고서 하나 = 패킷 하나 = 페이로드 + 체크섬 2 바이트(페이로드 합의 하위 16비트, 하위 바이트 먼저).

CapTIvate 패킷 (TI CapTIvate Technology Guide > Software Library > Communications Module > Protocol Layer):
  센서    00h [센서ID] [b0] [b1] [예약] [상태]   버튼군: b0 = 주(dominant) 버튼. 슬라이더/휠: b0|b1<<8 = 위치
                                                  (터치가 없으면 FFFFh)
  사이클  01h [센서ID] [사이클ID] [상태 24비트 LE: 하위 12 = 근접, 상위 12 = 터치, 비트 i = 사이클 안 i번 소자]
          + 소자마다 [LTA u16][count u16]
  범용    10h [개수] u16 × 개수
  파라미터 [CMD] [R/W] [센서ID] [4 바이트]       PC -> 보드 읽기(R/W=0)·쓰기(1), 보드 -> PC 는 결과 값
  상태 바이트: 비트0 터치, 1 이전 터치, 2 근접, 3 검출(디바운스 전), 4 음의 터치, 5 잡음, 6 최대 카운트 오류, 7 보정 오류
  자기용량(self) 센서는 손이 닿으면 count 가 LTA(장기 평균) 아래로 내려간다: 변화량 = LTA - count.
  센서 ID 는 Design Center 가 센서 이름을 알파벳순으로 정렬한 순서다 (BSWP: 0 keypad, 1 proximity, 2 slider, 3 wheel).

필요한 것: pip install hidapi (galvanic venv 에 설치돼 있다)
"""
import argparse, sys, time
import hid

VID, PID = 0x2047, 0x0965
REPORT_ID = 0x3F
I2C_ADDR = 0x0A

STATUS_BITS = ["touch", "prev_touch", "prox", "detect", "neg_touch", "noise", "max_count_err", "cal_err"]

# 파라미터 CMD (일부. 전체 표는 Protocol Layer > Format: Parameter Packets)
P_CONV_GAIN, P_CONV_COUNT, P_PROX_THR, P_TOUCH_THR = 0x80, 0x81, 0x82, 0x83
#   0x82 근접 임계값 = 근접 판정에 필요한 변화량 [count] (센서마다)
#   0x83 터치 임계값 = 보내는 바이트 3·4 = 사이클·소자 번호, 답 바이트 5 = 상대값 (LTA 의 x/128 만큼 내려가면 터치)
P_RESOLUTION = 0x92                     # 슬라이더/휠 해상도: 위치가 0 .. 해상도-1 로 나온다
P_FORCE_RECAL = 0x8E                    # 기준선(LTA) 다시 잡기. 손을 뗀 상태에서 보내야 한다
# 컨트롤러 파라미터 (센서 ID 안 씀, 값은 바이트 5·6). 보드 RAM 에만 있어서 리셋·전원을 끄면 원래대로
P_ACTIVE_PERIOD = 0xC2                  # 깨어 있을 때 스캔 간격 [ms] (데모 33)
P_WAKE_INTERVAL = 0xC4                  # 절전 중 몇 스캔마다 저절로 깨나 (데모 5 = 약 50 초)
P_INACTIVITY = 0xC5                     # 손이 없어진 뒤 몇 스캔 더 깨어 있나 (데모 32 = 약 1 초)
DEMO_WAKE_INTERVAL, DEMO_INACTIVITY = 5, 32


def u16(p, i):
    return p[i] | p[i + 1] << 8


def checksum(payload):
    s = sum(payload) & 0xFFFF
    return bytes([s & 0xFF, s >> 8])


def parse(p):
    """체크섬을 뗀 페이로드 하나 -> dict (모르는 모양이면 kind="unknown")"""
    if not p:
        return dict(kind="unknown", raw=p)
    cmd = p[0]
    if cmd == 0x00 and len(p) == 6:
        st = p[5]
        return dict(kind="sensor", sensor=p[1], b0=p[2], b1=p[3], pos=u16(p, 2), status=st,
                    flags={k: bool(st >> i & 1) for i, k in enumerate(STATUS_BITS)})
    if cmd == 0x01 and len(p) >= 10 and (len(p) - 6) % 4 == 0:
        st = p[3] | p[4] << 8 | p[5] << 16
        n = (len(p) - 6) // 4
        return dict(kind="cycle", sensor=p[1], cycle=p[2], n=n,
                    prox=[bool(st >> i & 1) for i in range(n)],
                    touch=[bool(st >> (12 + i) & 1) for i in range(n)],
                    lta=[u16(p, 6 + 4 * i) for i in range(n)],
                    count=[u16(p, 8 + 4 * i) for i in range(n)])
    if cmd == 0x10 and len(p) >= 2 and len(p) == 2 + 2 * p[1]:
        return dict(kind="general", values=[u16(p, 2 + 2 * i) for i in range(p[1])])
    if len(p) == 7 and cmd >= 0x60:
        return dict(kind="param", cmd=cmd, rw=p[1], sensor=p[2], data=bytes(p[3:7]), value=u16(p, 5))
    return dict(kind="unknown", raw=bytes(p))


class Bridge:
    """CAPTIVATE-PGMR HID 브리지. 데이터는 read_packet(), 설정은 hb()."""

    def __init__(self, interface="I2C"):
        devs = {d["interface_number"]: d["path"] for d in hid.enumerate(VID, PID)}
        if not {0, 1} <= devs.keys():
            raise RuntimeError("CAPTIVATE-PGMR HID 브리지를 못 찾았다 (USB 케이블·프로그래머 보드 확인)")
        self.dat = hid.device(); self.dat.open_path(devs[0])
        self.cfg = hid.device(); self.cfg.open_path(devs[1])
        self.bad_checksum = 0
        self.setup(interface)

    # ---------------------------------------------------------------- 설정 콘솔
    def hb(self, command, timeout_ms=500):
        """"HB ..." 명령 하나 -> 답 문자열 (예: "MODE=PACKET INTERFACE=I2C")"""
        b = command.encode()
        rep = bytes([REPORT_ID, len(b)]) + b
        self.cfg.write(rep + bytes(64 - len(rep)))
        r = self.cfg.read(64, timeout_ms)
        return bytes(r[2:2 + r[1]]).decode(errors="replace") if r else ""

    def setup(self, interface):
        self.version = self.hb("HB VERSION")
        if self.hb("HB MODE") != f"MODE=PACKET INTERFACE={interface}":
            self.hb(f"HB MODE PACKET {interface}")
        if interface == "I2C" and self.hb("HB I2CADDRESS") != f"I2CADDRESS=0x{I2C_ADDR:02X}":
            self.hb(f"HB I2CADDRESS {I2C_ADDR}")
        self.mode = self.hb("HB MODE")
        # 설정을 바꾸기 전에 쌓였을지 모르는 보고서는 버린다
        while self.dat.read(64, 1):
            pass

    # ---------------------------------------------------------------- 데이터
    def read_packet(self, timeout_ms=100):
        """패킷 하나(체크섬 뗀 bytes) 또는 None. 체크섬이 틀린 보고서는 세고 버린다."""
        r = self.dat.read(64, timeout_ms)
        if not r:
            return None
        n = r[1]
        if r[0] != REPORT_ID or n < 3 or n > 62:
            self.bad_checksum += 1
            return None
        body = bytes(r[2:2 + n])
        p, ck = body[:-2], body[-2:]
        if checksum(p) != ck:
            self.bad_checksum += 1
            return None
        return p

    def send_packet(self, payload):
        """PC -> 보드 패킷 (파라미터 읽기·쓰기). 브리지가 SYNC·길이·바이트 스터핑을 붙여 보드로 보낸다."""
        body = bytes(payload) + checksum(payload)
        rep = bytes([REPORT_ID, len(body)]) + body
        self.dat.write(rep + bytes(64 - len(rep)))

    def write_param(self, cmd, value, sensor=0, b3=0, b4=0):
        """파라미터 쓰기 (값은 바이트 5·6, 하위 바이트 먼저). 보드가 쓴 뒤의 값을 param 패킷으로 돌려준다."""
        self.send_packet([cmd, 1, sensor, b3, b4, value & 0xFF, value >> 8 & 0xFF])

    def request_param(self, cmd, sensor, b3=0, b4=0):
        """파라미터 읽기 요청. 답은 read_packet() 으로 kind="param" 패킷이 되어 돌아온다 (보드가 깨어 있을 때만)."""
        self.send_packet([cmd, 0, sensor, b3, b4, 0, 0])

    def close(self):
        self.dat.close(); self.cfg.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--raw", action="store_true", help="풀지 않고 16진수로 찍는다")
    ap.add_argument("--uart", action="store_true", help="브리지를 PACKET UART 로 (펌웨어가 UART 통신일 때)")
    ap.add_argument("--seconds", type=float, default=0, help="이 시간 뒤 끝낸다 (0 = Ctrl+C 까지)")
    a = ap.parse_args()

    br = Bridge("UART" if a.uart else "I2C")
    print(f"{br.version}  {br.mode}")
    print("패널에 손을 가까이 가져가면 보드가 깨어나 30 Hz 로 보낸다 (손이 없으면 절전, 약 50 초마다 한 번 깬다)")
    t0 = time.time()
    try:
        while not a.seconds or time.time() - t0 < a.seconds:
            p = br.read_packet()
            if p is None:
                continue
            t = time.time() - t0
            if a.raw:
                print(f"{t:8.3f}  {p.hex(' ')}")
                continue
            d = parse(p)
            if d["kind"] == "cycle":
                deltas = " ".join(f"{l - c:+4d}" for l, c in zip(d["lta"], d["count"]))
                tch = "".join("T" if x else ("p" if y else ".") for x, y in zip(d["touch"], d["prox"]))
                print(f"{t:8.3f}  사이클 s{d['sensor']} c{d['cycle']}  [{tch}]  LTA-count: {deltas}")
            elif d["kind"] == "sensor":
                on = ",".join(k for k, v in d["flags"].items() if v) or "-"
                print(f"{t:8.3f}  센서   s{d['sensor']}  b0={d['b0']} pos={d['pos']}  상태={on}")
            else:
                print(f"{t:8.3f}  {d}")
    except KeyboardInterrupt:
        pass
    finally:
        if br.bad_checksum:
            print(f"체크섬 틀린 보고서 {br.bad_checksum} 개 버림")
        br.close()


if __name__ == "__main__":
    main()
