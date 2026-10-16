# CapTIvate (MSP-CAPT-FR2633 + BSWP panel) workspace

TI's capacitive-touch development kit. The factory demo firmware is left as is; the raw values of the panel's 16 electrodes are streamed to a Mac and shown as a real-time **capacitance map**.
TI's CapTIvate Design Center is not needed.

The same board is also used as a **touchscreen receiver for a stent transmitter**: a stent driven by the TeensyTRX transceiver lies on the panel, and the scripts here measure the bit error rate (BER) of that link.

## Hardware connection (verified working as of 2026-10-14)

```
   ┌──────────────────────┐   ┌──────────────────────┐   ┌──────────────────────┐
   │  BSWP touch panel      │   │  FR2633 MCU board      │   │  PGMR programmer       │
   │  (copper electrodes    │══>│  (MCU, LED1/LED2,      │══>│  (flashing + PC comms  │──USB──> Mac
   │   only)                │   │   reset button)        │   │   bridge)              │
   │  8 buttons, slider,    │   │                        │   │  green LED on/blinking │
   │  wheel, proximity ring │   │                        │   │                        │
   └──────────────────────┘   └──────────────────────┘   └──────────────────────┘
        white connector             20-pin connector             micro-USB cable
```

The Mac sees two USB devices:
- `MSP Tools Driver`: used for flashing firmware.
- `MCU COMM HID to Serial Bridge`: the data path. It enumerates as HID, so no driver is needed.

## Running the capacitance map

Double-click **`run_bswp_map.command`** in Finder, or run `./run_bswp_map.command` in a terminal. It uses the Python in `~/.venvs/galvanic`.

The GUI labels are in Korean; English translations are given in parentheses below.

1. When the window opens, the top shows `3 초 뒤 - 패널에서 손을 떼세요` ("in 3 s, take your hand off the panel"). This captures the baseline once at start-up, so keep your hand away from the panel.
2. Then **bring your hand close to the panel once.** The demo firmware sleeps when no hand is present, and this wakes it up. Even without a hand it wakes up on its own within about 50 s.
3. Once awake, data keeps arriving **in real time at 30 Hz even after you take your hand away**, as long as `보드 계속 깨워 두기` ("keep the board awake") is checked (the default). The status line at the bottom shows `보드: 계속 깨움` ("board: kept awake") when this works.
   - The board stays awake even if you close and reopen the window. To return to the demo's original behaviour (sleep when no hand is present), uncheck the box, press RESET on the MCU board, or unplug and replug USB.
   - `화면 .. fps` in the status line is the actual redraw rate, normally 25-30.
4. **If an electrode stays coloured, or keeps a yellow or white outline while nothing is touching it, the baseline is off.** A ⚠ warning also appears at the bottom. Press **[기준선 다시 잡기]** ("re-capture baseline") and take your hand away within 3 s.

### Things to try
| Action | What you see |
|---|---|
| Hover a palm 3-5 cm above the panel | The outer **proximity ring P** turns orange first and its outline turns yellow (proximity). The number grows as the hand gets closer |
| Press one button | That circle turns red and gets a **white outline** (touch). The bottom line shows `주 버튼 N` ("dominant button N") |
| Press two buttons at once | Both get a white outline (multi-touch), but the dominant button points only at the stronger one |
| Press on the boundary between two buttons | Both change by about half. Worth checking which one is reported as dominant |
| Swipe the slider left to right | The colour moves S0→S1→S2→S3→S0. The ▼ marker follows the position (0-999) |
| Turn once around the wheel | W0→W1→W2 change in turn. The ● marker and the number in the centre (0-99) follow |
| Press through paper or a glove, or with a wet finger | See how the size of the change differs |

### Reading the display
- **Colour = (LTA − count) / LTA [%]**
  - count is the current measurement.
  - LTA is the long-term average (baseline) kept by the board.
  - All electrodes on this panel are self-capacitance electrodes, so a finger pulls the count below the baseline and the value becomes positive.
- Typical baselines: buttons ≈ 248, slider ≈ 598, wheel ≈ 470-496, proximity ring ≈ 779 counts. On a button, 1 count is 0.4 %.
- A **white outline** means the board decided "touch"; a **yellow outline** means "proximity". Both use the thresholds set in the board firmware.
- The plots on the right show the last 10 s of change: buttons 1-8 on top, slider, wheel and proximity ring at the bottom.
- The colour range defaults to `자동` (auto) and can be fixed from the menu at the top.
- Setting the **colour reference to `터치 기준 대비`** ("relative to touch threshold") colours each electrode as `change ÷ that electrode's touch threshold`. The board reports a touch the moment this reaches 1. Electrodes that naturally change a lot, such as the proximity ring, then no longer drown out the others.
- The **dotted lines in the plots on the right are the touch thresholds**, read directly from the board. Measured values:

| Electrode | Touch threshold (drop relative to LTA) | Proximity threshold |
|---|---|---|
| Buttons 1-8 | 7.8 % (≈19 counts) | 10 counts (4 %) |
| Slider S0-S3 | 14-15 % | 20 counts (3.3 %) |
| Wheel W0-W2 | 16 % | 20 counts (4 %) |
| Proximity ring P | 10 % | 12 counts (1.5 %) |

  A button only exceeds 7.8 % when the fingertip covers almost the whole circle. A fingernail or just the tip stops at 2-4 % and is not detected as a touch.

### Recording
Press **[기록]** ("record") to append one row per scan to `data/bswp_<date_time>.csv`. Press it again to stop. Columns:
- `t_unix`
- For each of the 16 electrodes: `<name>_count`, `<name>_lta`, `<name>_touch`, `<name>_prox`
  - Names: `B1`-`B8`, `S0`-`S3`, `W0`-`W2`, `P`
- `keypad_dominant`: zero-based. Add 1 to get the button number
- `slider_pos`, `wheel_pos`: empty when there is no touch
- `slider_res`, `wheel_res`
- `keypad_status`, `prox_status`, `slider_status`, `wheel_status`: status bytes. Bit 0 touch, 1 previous touch, 2 proximity, 3 detect, 4 negative touch, 5 noise, 6 max-count error, 7 calibration error.

## Troubleshooting
| Symptom | Cause and fix |
|---|---|
| The slider, wheel, etc. stay on with nothing touching them (⚠ warning at the bottom) | The baseline (LTA) is off. The board captures it at power-up; if a hand was near the panel at that moment, or the panel was moved afterwards (e.g. next to the laptop), the baseline is wrong. The board also halts baseline updates during proximity/touch, so it takes more than 2 minutes to recover by itself. → Press **[기준선 다시 잡기]** and take your hand away |
| Pressing a button does not give a white outline | Switch the colour reference to `터치 기준 대비` and press again to see whether it gets close to 1. If it stops at 0.3-0.5, the finger does not cover enough of the circle. Otherwise the baseline drifted while you were pressing → re-capture the baseline |
| The display stays at `아직 데이터 없음` ("no data yet") | The board is asleep. Bring a hand over the panel to wake it. Without a hand it wakes within about 50 s. After that it stays real-time |
| The display freezes 1 s after taking the hand away | `보드 계속 깨워 두기` is unchecked, so the board follows the demo's original behaviour (sleep when no hand). Check it and the board stays awake from the next wake-up |
| "HID 브리지를 못 찾았다" (HID bridge not found) or open failure on start | Check the USB cable. **If two instances of this program are running, the second cannot open the device.** Close the previous window |
| A real power reset is needed | Press the `MCU RESET` button on the MCU board (with the hand away from the panel). The board re-captures the baseline from scratch |

## Stent → touchscreen BER test in one command (`stent_ber.py`)

When the transceiver and the CapTIvate board are plugged into **the same laptop**, this one script does transmit, receive, the live window and the analysis.

```
~/.venvs/galvanic/bin/python stent_ber.py --electrode S1 --carrier 163000  # one carrier (sensor tuning + calibration ~20 s, 2 frames ~80 s)
~/.venvs/galvanic/bin/python stent_ber.py --carrier 100e3,150e3,200e3      # several in turn + comparison table and figure
~/.venvs/galvanic/bin/python stent_ber.py --carrier 163e3 --wave sine      # square vs sine
```

**What it does for each carrier**
1. Captures the baseline with the transmitter off and records 4 s of noise without transmission.
2. Starts transmitting. The script itself sends the bits, so it knows the bit timing exactly.
3. Shows a live window during transmission: the electrode count, the transmitted bits and the mean count per bit.
4. At the end it opens `data/stentber_*/result.png`:
   - (a) received raw signal and transmitted bits
   - (b) average step response at turn-on and turn-off: **how** the touchscreen sees the carrier
   - (c) distribution of the bit decision values
   - (d) transmitted vs received bits: **whether it decodes correctly**

**Two kinds of BER**
- **Blind:** synchronisation and decisions use only the received data. This is the real receiver performance.
- **Genie timing:** the transmit time is given to the decoder. This is an upper bound without synchronisation errors.

**Board settings changed only during the test** (restored to 250 afterwards)
- The button conversion count is raised from 250 to 500 (`--conv-count`).
- At 250 the count is almost stuck at one value (noise σ 0.07-0.12 counts), so changes smaller than one step (0.4 %) stay invisible even after averaging.
- At 500 one step is 0.2 % and the noise grows to σ 0.7 counts, so averaging reveals changes smaller than one step.
- 1000 cannot be used because the scan rate drops to 3.8 Hz.

**How the touchscreen picks up the signal (confirmed by measurement on 2026-10-15)**
- The sensor measures once by repeating, hundreds of times, a cycle of charging the electrode and transferring that charge to an internal capacitor (charge transfer).
- What survives in each transferred charge is only **the change in stent voltage between the charge instant and the transfer instant**.
  - So with the demo setting (charge-transfer frequency 2.016 MHz), carriers of 1 kHz, 100, 163 and 200 kHz **did not move the count by even 0.1**, although the voltage measured directly at the stent electrode E+ was 1.1 Vpp.
  - In contrast, a signal near the charge-transfer frequency f_t (within ±0.5 kHz) accumulates with the same phase every time, so the count **swings by ±30-50**. The noise before transmission is σ 0.7-1.
- f_t can be changed: **f_t = 2.016 MHz ÷ (L + 1)**, with frequency divider 2 and charge length = transfer length = L.
  - Measured: 0/0 → 2.016 MHz, 11/11 → 168.0 kHz, divider 3 with 5/5 → 168.0, divider 3 with 6/6 → 144.0 kHz
  - With charge ≠ transfer, or with divider 1, the frequency deviated from the formula or the response was weak.
- Available values between 100 and 200 kHz: 100.8, 106.1, 112.0, 118.6, 126.0, 134.4, 144.0, 155.1, 168.0, 183.3, 201.6 kHz
- So for each carrier `stent_ber.py`:
  1. Sets the receiving sensor's f_t to the nearest available value.
  2. Runs a calibration sweep (±2 %, 100 Hz steps, about 14 s) to find the frequency with the largest response.
  3. Moves the transmitter to that frequency and measures the BER.
  4. Restores the sensor settings at the end.
  - With `--no-tune` the demo f_t (2.016 MHz) is kept.
- Because the signal shows up as a "swing" rather than a "mean shift", the decision feature is `logenergy`: the log of the squared differences between neighbouring samples within a bit.
  - Plain energy let a few large swings pull the threshold up, giving 8/127 errors in the 2.016 MHz run. With logenergy it is 0/127.

**Results (stent on S1, square-wave OOK at 5 bps, scan 51.1 Hz = 10 samples per bit)**

| Carrier | Touch sensor f_t | Blind BER | Notes |
|---|---|---|---|
| 1 kHz, 100, 163, 200 kHz | 2.016 MHz (demo) | no sync | count change ≤ 0.12 |
| 2.016 MHz | 2.016 MHz (demo) | 0/127 (logenergy) | swing +27.8 dB |
| 163 kHz requested → **167.842 kHz** | 168.0 kHz (divider 2, 11/11) | **1/254** = 3.9e-3 | swing +22 dB, Q 5.1, sync error +0.02 bit |
| 163 kHz requested, **FSK**: 1 = 167.841 / 0 = 170.841 kHz (`--keying fsk`) | 168.0 kHz | **0/254** | constant transmit envelope, contrast +25.6 dB, Q 5.4 — 3 kHz off is "off" for the sensor |

- The lack of response on B1 at 1 kHz and 163 kHz earlier the same day had the same cause (f_t was not tuned then).
- Electrode selection: `--electrode S1` (B1-B8, S0-S3, W0-W2, P)
- Re-analysis: `--reanalyse data/stentber_...`

## FSK + DBPSK on one waveform (2026-10-15)

The tablet bit is carried by frequency (1 = 167.84 kHz, 0 = 170.84 kHz). On top of that, DBPSK phase flips for stent-to-stent communication are overlaid.

**Firmware change**
- Added the command `psk <symbol µs> [PRBS order]` to TeensyTRX.
  - Each schedule step (= one tablet bit) is divided into symbol periods.
  - When the PRBS bit (order 9 by default, seed all ones) is 1, the phase is flipped by 180°.
  - `psk 0` (the default) behaves as before.
- The original firmware is kept in `transceiver/firmware/TeensyTRX_backup_20261015/`.
- Verification: recording E+ and reading the phase difference between neighbouring symbols matched PRBS9 in 399/399 symbols (phase +85° / −96°).
- Caveat: every symbol restarts the phase at 0°/180°. If carrier × symbol length is not an integer, a constant phase rotation is added per symbol, and the in-body receiver must compensate for it.
- **Firmware changed at 17:00 on 2026-10-15 (default `cont`)**: the "restart" scheme above jittered the turn-on instant by 1 µs (60° at 168 kHz), so the transmitted waveform itself had DBPSK errors (transmit loopback BER about 1e-3).
  - Now both tones and 0°/180° are loaded into the registers before starting, and each symbol only changes FSEL/PSEL with a single control word. The phase is continuous and there is no gap between symbols.
  - In the same loopback, 10/100/1000 baud all give zero errors (`transceiver/data/psk_loopcont_tx_20261015_170829`).
  - The old scheme is available as `psk <symbol µs> 9 reset`. It is selected automatically when combined with OOK (the zero word).
  - So `stent_ber.py` results after this time were measured with a phase-continuous waveform on the tablet side as well.

**Examples**
```
~/.venvs/galvanic/bin/python stent_ber.py --electrode S1 --carrier 163000 --keying fsk --bps 5 --psk-baud 0,10,100,1000
~/.venvs/galvanic/bin/python stent_ber.py --electrode S1 --carrier 163000 --keying fsk --bps 10,20,25 --psk-baud 100 --frames 2
```

**Results** (figure `data/fsk_dbpsk_summary_20261015.png`, tablet BER with blind decoding)

| Test | Condition | Tablet BER |
|---|---|---|
| DBPSK overlay | none, 10, 100, 250 baud | 0/127 |
| | 1000 baud | 1/127 |
| | 5000 baud | 26/127 (swing +20 → +6 dB) |
| Tablet rate (DBPSK 100 baud overlaid, scan 51 Hz) | 5 / 10 / 20 bps | 0/127, 2/254, 2/254 |
| | 25 bps (2 samples per bit) | 8/254 (3 %) |
| Scan 85 Hz (12 ms) | 20 / 25 / 33 bps | 5/254, 6/254, 8/254 |

- Summary:
  - DBPSK does not affect tablet reception **up to 1000 baud**; at 5000 baud it breaks down.
  - The tablet rate stays **below 1 % BER up to 20 bps**; 25-33 bps gives about 3 %.
- With the scan period set to 10 ms, the board actually scans irregularly about every 11.1 ms, because one scan (slider 3.6 ms + other sensors + I2C transfer) takes longer than 10 ms. This breaks the analysis, which assumes uniform spacing. **12 ms (85 Hz) was the fastest setting that runs uniformly.**
  - With scan periods shorter than 15 ms, the sensor summary packets (C1) are turned off.
- With fewer than 4 samples per bit, the samples at the start of a bit are not discarded (`set_guard`). Decisions use the `absdev` feature: how far a single sample deviates from the baseline, on a log scale.

## Tablet rate × scan rate (2026-10-15, `plot_rate_sweep.py`)

Conditions: S1, FSK 167.84/170.84 kHz, DBPSK 100 baud overlaid, 635-762 bits per point (254 for 5 bps). Results in `data/rate_sweep_rateF51_rateF60_rateF68.png` and `.csv`.

| Tablet rate | 51 Hz (20 ms) | 60 Hz (17 ms) | 68 Hz (15 ms) |
|---|---|---|---|
| 5 bps | 0/254 | 0/254 | 0/254 |
| 10 bps | 0.79 % | 0.63 % | 0/635 |
| 12.5 bps | 2.6 % | 0.66 % | 0.26 % |
| 16.7 bps | 1.7 % | 1.05 % | 1.05 % |
| 20 bps | 3.3 % | 1.2 % | 0.79 % |
| 25 bps | 2.9 % | 3.4 % | 1.2 % |
| 33 bps | 6.7 % | 5.3 % | 3.7 % |
| 50 bps | 14.6 % | 6.4 % | 6.4 % |

- Re-plotted against scans per bit (= scan rate ÷ bit rate), the curves collapse onto each other. At about 4 or more scans per bit, the BER is around or below 1 %.
  - So roughly **maximum rate ≈ scan rate ÷ 4**. Measured: 51 Hz → 10 bps, 60 Hz → 12.5 bps, 68 Hz → 20 bps.
- If the carrier is about 160 Hz off the touch sensor's optimum, the BER gets 2-3× worse, so the calibration sweep is retried when it fails.
- At 85 Hz (12 ms), the PGMR HID bridge dropped out after a few minutes (`read error`). The sensor summary packets (C1) are always off during tests.
- Drop-outs occasionally happen at low load too. If it stays unstable after a drop-out, replug the PGMR USB.

## OOK vs FSK (2026-10-15, `plot_ook_fsk.py okfsk_ook okfsk_fsk`)

Conditions: S1, scan 68 Hz (15 ms), touch sensor f_t 168 kHz, no DBPSK overlay. At 15 bps a bit is 66.7 ms, which is not a multiple of the 10 ms symbol, so DBPSK was left out. Figure: `data/ook_vs_fsk_okfsk_ook_okfsk_fsk.png`.

| | 10 bps | 15 bps | on/off contrast |
|---|---|---|---|
| OOK (1 = 167.80 kHz on, 0 = off) | 0/635 | 0/762 | +22 / +18 dB |
| FSK (1 = 167.81, 0 = 170.81 kHz, always on) | 0/635 | 5/762 (0.66 %) | +19 / +19 dB |

- The count waveform received by the touchscreen looks the same for both: it swings by about ±80 during a 1 and stays quiet during a 0. To the touch sensor, FSK's '0' at 3 kHz offset looks exactly like "off".
- At 15 bps, FSK 5 errors vs OOK 0 is borderline (the confidence intervals overlap). Both are below 1 %.
- FSK keeps the carrier on, so DBPSK can be overlaid. With OOK the in-body link is interrupted during every '0'.

## What FSK actually improves (2026-10-15, `plot_fsk_advantage.py`, figure `data/fsk_advantage.png`)

Judged by tablet BER alone, FSK cannot beat OOK, because the tablet sees FSK's '0' (+3 kHz) as "off". So the differences on the transmit side were measured.

- **E+ recording (`transceiver/eplus_keying.py`):** TeensyTRX recorded its own electrode at 500 kS/s while transmitting. 10 bps, carrier 167.8 kHz.
  - **Envelope:** OOK alternates between on and off (49 % variation); FSK varies by 1 %.
  - **10-100 Hz envelope ripple** (the sensing band of cardiac devices, relative to carrier amplitude): OOK −16 dB, **FSK −50 dB**.
  - **Fraction of the overlaid DBPSK 100 baud symbols correctly decoded at E+:** OOK 41 % (327/799, lost during '0'), **FSK 100 % (799/799)**.
- **Tablet BER** (DBPSK overlaid, scan 68 Hz, `--label adv_ook / adv_fsk`):

  | | 10 bps | 20 bps |
  |---|---|---|
  | OOK | 0/635 | 5/762 |
  | FSK | 1/635 | 2/762 |

  Both schemes are at the same level.

## Hearing FSK's '0' with two electrodes (2026-10-15, `dual_ber.py`, figure `data/dual_ber.png`)

A single electrode only hears within ±0.5 kHz of its own charge-transfer frequency, so FSK's '0' (a different frequency) is simply "off" for it. Tuning the proximity ring P to the '0' frequency lets the tablet hear the '0' directly as well. No hardware is added: the same stent and transmitter are used, and two of the panel's existing electrodes are read at the same time.

```
TeensyTRX ── E+ wire ──> stent (lying on S1)
                            │  field spreads into the panel
                 ┌──────────┴──────────┐
                 ▼                     ▼
       S1: hears 168 kHz only    P: hears 155 kHz only
       (bit 1 → swings)          (bit 0 → swings)
```

- **Setup:** scan 68 Hz, blind decoding, 635-762 bits per rate
  - S1 listens to the '1' frequency with L = 11 (168.0 kHz).
  - P listens to the '0' frequency with L = 12 (155.1 kHz). P is the ring around the whole panel border, so it passes close to the stent on S1.
  - Calibration: '1' = 167.79 kHz (+28 dB), '0' = 154.88 kHz (+29 dB)
- **Decision rule**
  - FSK, two electrodes: z(S1) − z(P), where z is the swing feature (absdev) normalised by its interquartile range.
  - OOK, two electrodes (reference): S1 and P tuned to the same frequency, decision on z(S1) + z(P).
- **Run:** `~/.venvs/galvanic/bin/python dual_ber.py --mode fsk2 --label dual_fsk2` (OOK: `--mode ook2 --label dual_ook2`); figure: `plot_dual.py dual_fsk2 dual_ook2`. The figure shows only the two-electrode receivers; the single-electrode results are stored in each `result.json` under `best.single`.

  | bps (scans per bit) | OOK, S1 only | OOK, two electrodes | FSK, S1 only | FSK, two electrodes |
  |---|---|---|---|---|
  | 10 (6.8) | 0/635 | 0/635 | 1/635 | 0/635 |
  | 16.67 (4.1) | 3/762 | 0/762 | 12/762 | 1/762 |
  | 20 (3.4) | 3/762 | 2/762 | 7/762 | 1/762 |
  | 25 (2.7) | 12/762 | 2/762 | 21/762 | **0/762** |
  | 33.33 (2.0) | 46/762 | 17/762 | 59/762 | **10/762** |
  | Total | 64/3683 | 21/3683 | 100/3683 | **12/3683** |

- **How to read it**
  - With one electrode, FSK is worse than OOK: for S1, FSK's '0' is not silence but "a different tone it hears faintly".
  - With two electrodes, FSK is the best: 0/762 at 25 bps and 1.3 % at 33 bps (OOK with two electrodes: 2.2 %).
  - The difference from two-electrode OOK is within overlapping 95 % intervals, however. Most of the gain comes from using two electrodes; FSK adds a little on top.
  - This is the first combination that received 25 bps (2.7 scans per bit) below 1 % BER. With one electrode the limit was scan rate / 4 (68 Hz → about 17-20 bps).

## Stent → button 1 bit transfer (BER, separate TX and RX programs)

The earlier two-terminal workflow, kept for reference; `stent_ber.py` above replaces it when both devices are on the same laptop.

A stent connected to TeensyTRX lies on button 1 and sends OOK bits. CapTIvate receives them through the button 1 count and computes the BER.

```
  Mac ──USB── TeensyTRX (AD9833) ── E+ wire ──> stent  (just placed on the acrylic over button 1)
   │                                 E− (Teensy GND) does not need a separate wire
   │                                 (on the same Mac, both devices already share ground via USB)
   └──USB── CAPTIVATE-PGMR ── FR2633 ── BSWP panel (button 1 = top-left '1')
```

**Steps** (close the `bswp_map` window first; only one program can open the board's USB)
1. Put the stent on button 1 and connect the E+ wire.
2. **Pick a carrier first** (about 2 min); it is not known in advance which frequency button 1 can see.
   - Terminal 1 (receiver, start first):
     `cd captivate; ~/.venvs/galvanic/bin/python cap_ook_rx.py --probe --label probe1`
   - When "지금 송신 시작" ("start transmitting now") appears, terminal 2 (transmitter):
     `cd transceiver; ~/.venvs/galvanic/bin/python cap_ook_tx.py --probe`
   - At the end a table of 14 carriers is printed, and the carrier with the largest level (mean shift) and energy (swing) is recommended.
3. **Send bits on that carrier** (about 2.5 min, 508 payload bits).
   - Receiver: `~/.venvs/galvanic/bin/python cap_ook_rx.py --carrier <Hz> --label run1`
   - Transmitter: `~/.venvs/galvanic/bin/python cap_ook_tx.py --carrier <Hz>`
   - Give `--bps`, `--payload` and `--prbs` the same values on both sides. Also match the transmitter's `--seconds` with the receiver's `--tx-seconds`.
4. Results are saved in `data/capook_<label>_<time>/` and the figure (`result.png`) opens automatically.
   - Re-analyse with other options: `cap_ook_rx.py --reanalyse data/capook_...` (e.g. `--feature level`)

**What the receiver does**
- **Board settings are changed only during recording and restored afterwards.**
  - The scan period is reduced to 20 ms (measured 51.12 Hz).
  - The count filter of the button sensor is turned off, so it does not smooth away the swings caused by the transmitter.
  - Automatic recalibration is turned off, and the timeout and negative-touch threshold are raised to the maximum, so the board does not re-capture its baseline while the transmitter shakes the count.
- **The baseline is captured once with the stent in place and the transmitter off.**
- **Both features are evaluated**, since it is not known in advance how the carrier will appear in the count.
  - level: mean shift of the count. A square wave also has a higher mean voltage during a 1.
  - energy: squared difference between neighbouring samples, i.e. the swing caused by the carrier being sampled at a different phase in each scan.
  - The better one is chosen by the preamble + SFD correlation and the BER.
- **Frame search and decisions**
  - The transmitter repeats frames at an exact period set by the Teensy crystal, so a single grid that maximises the summed correlation over all frames is chosen.
  - The decision threshold follows the slow baseline drift using known zeros (header, trailing gap) and decided bits within ±16 bits.
- **Reported numbers:** BER (95 % upper bound 3/N when there are zero errors), Q and the Gaussian BER implied by that Q, and how much the count moved during a 1 (in counts and %).
- **Sample timing:** HID arrival times can only be late, so the scan period of the board is fitted to the least-delayed samples and every scan is numbered (missing scans are interpolated and counted).
- **Start time for the carrier probe:** found automatically from the transmit log (`transceiver/data/capook_tx_*.json`); the 3 s marker at the start of the schedule can also be used.

**Verification (`test_cap_ook_rx.py`, synthetic data without the board)**
- Includes board/Teensy clock mismatch, arrival jitter, 0.3 % missing scans, integer counts and drift.
- Results:
  - Strong signal: 0/508 for both level and energy
  - Weak signal: BER 3.0 % (ideal receiver 2.1 %, implementation loss about 0.5 dB)
  - Noise only: no false frame detection
  - Carrier probe: start time and best carrier found correctly
- Real board:
  - Receive stream: 51.1 Hz, no missing scans, B1 ±0.2-0.3 counts without transmission
  - Transmit: schedule upload and checksum verified. Combined transmit/receive runs were done later with `stent_ber.py` (sections above).

**Why the default carrier was 1012.5 Hz** (superseded: the 2026-10-15 measurements above showed that the sensor only responds near its charge-transfer frequency)
- To measure button 1 once, CapTIvate transfers charge 250 times over about 125 µs (divider 2, ~2 MHz).
- The reasoning was that carriers above a few kHz would average out within this window (0 at 8 kHz, −36 dB at 163 kHz).
- 1012.5 Hz aliases to about 12 Hz at a 50 Hz scan rate (near a quarter of the sampling rate), which makes neighbouring-sample differences large.
- This was only a calculation, which is why `--probe` was recommended first. Frequencies near 2 MHz (the charge-transfer frequency) were included in the probe as well.

## Files
| File | Purpose |
|---|---|
| `run_bswp_map.command` | Double-click launcher. Uses the Python in `~/.venvs/galvanic` |
| `bswp_map.py` | Capacitance map GUI + CSV recording |
| `captivate_hid.py` | HID bridge setup and CapTIvate packet parsing. Run on its own (`python captivate_hid.py`) it prints packets as text, or as hex with `--raw` |
| `stent_ber.py` | Stent → touchscreen BER test: transmit, receive, live window and analysis in one command (same laptop) |
| `cap_ook_rx.py` | Stent OOK receiver + BER (section above). The transmitter is `../transceiver/cap_ook_tx.py` |
| `test_cap_ook_rx.py` | Synthetic self-test of the receiver analysis |
| `dual_ber.py` | Two-electrode (S1 + P) tablet BER, FSK/OOK |
| `plot_rate_sweep.py` | BER vs bit rate and scan rate figure |
| `plot_ook_fsk.py`, `plot_fsk_advantage.py`, `plot_dual.py` | OOK vs FSK comparison figures |
| `data/` | Recorded CSVs and `capook_*`, `stentber_*`, `dual_*` recording/result folders |

**Dependencies outside this folder:** transmission and part of the analysis import code from sibling folders. With this repository alone, only `bswp_map.py` and `captivate_hid.py` run standalone.
- `../transceiver/cap_ook_tx.py`, `../transceiver/trx_recip.py`: TeensyTRX transmitter
- `../transceiver/firmware/TeensyTRX/`: firmware
- `../dds_signal_gen/ook_rx.py`: PRBS

## Communication notes (measured 2026-10-14)
- The HID bridge has two HID interfaces.
  - Interface 0 is for data.
  - Interface 1 is a console configured with `HB ...` text commands.
- Report format: `[3Fh, length, data…]`.
- The bridge ships in `PACKET UART` mode, but the default demo uses BULK I2C (address 0x0A), so the program switches it to `HB MODE PACKET I2C` every time it starts. This is not saved, so it reverts when USB is unplugged.
- Each scan delivers 9 packets, in sensor order: buttons (2 cycles + sensor) → proximity (cycle + sensor) → slider (cycle + sensor) → wheel (cycle + sensor).
- Sensor IDs: 0 buttons, 1 proximity, 2 slider, 3 wheel. Design Center assigned them in alphabetical order of the sensor names.
- Slider and wheel resolution are queried with parameter packet `92h`. Measured: **slider 1000, wheel 100**.
- Touch thresholds come from `83h` (per cycle and element), in units of 1/128 of LTA.
- Proximity thresholds come from `82h` (per sensor), in counts.
- Baseline re-capture is `8Eh` (Force Re-Calibrate). Even when the board is asleep, an I2C write wakes it up to process the request.
  - The board then only wakes briefly to process it and goes back to sleep; it does not start scanning.
  - While asleep, replies come late and in bursts.
- Rendering:
  - The panel is one image of 0.4 mm pixels (`imshow`), the same approach as ThermoVein's `maxtouch_cap_heatmap.py`.
  - Static axes, legends and colour bars are cached as the background and only the changing parts are redrawn (blit). Everything is redrawn only when the axes or colour range change.
  - At least 15 ms is left between frames so that clicks get processed.
  - Measured: 16 ms per frame, 27 fps, median event latency 6 ms.
  - The old version redrew 30 shapes every frame (40-50 ms). Redrawing every 40 ms left no time for clicks, and the window resized whenever the length of the status text changed.
- One baseline re-capture request (`8Eh`) recalibrates the whole panel. The board pauses scanning for about 0.5 s meanwhile.
- Keeping the board awake uses controller parameters:
  - `C5` (inactivity timeout) = 65535
  - `C4` (wake-up interval during sleep) = 1
  - The original values are 32 / 5. In the same group, `C2` is the active scan period (33 ms) and `C3` the scan period during sleep (100 ms).
  - These live only in the board's RAM and revert on reset. They are deliberately not restored when the window closes.
  - Measured: after changing them and waking the board once, data arrived at 30 Hz without gaps for 60 s with no hand present.
- Demo settings (measured):
  - Conversion count: buttons 250, proximity 800, slider 600, wheel 500
  - LTA filter beta: 7
  - LTA halt during proximity/touch: on
- The wheel ● marker assumes "position 0 = centre of W0 (top), increasing in the W0→W1→W2 direction (clockwise)". If turning the wheel shows the opposite direction, fix the single `ang = ...` line in `bswp_map.py`.
- Packet format source: TI CapTIvate Technology Guide
  - Software Library > Communications Module > Protocol Layer
  - CAPTIVATE-PGMR > HID Bridge

## Possible next steps
- **Disabling sleep / changing the scan rate permanently**: requires reflashing the firmware. Open the BSWP example in Code Composer Studio and change `ui16ActiveModeScanPeriod` (currently 33 ms) and the wake-on-proximity setting in `CAPT_UserConfig.c`. (For experiments, the scan period `C2` and keep-awake `C4`/`C5` are already changed at run time through parameter packets, as described above.)
- **Custom electrodes**: remove the panel and wire electrodes to the MCU board connector pins (CAP0.0-CAP3.3). With the same demo firmware they appear under the names of their original positions.
