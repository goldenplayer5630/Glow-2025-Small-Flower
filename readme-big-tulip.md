# RS-485 Flower Node (UNO + WS2811) — README

A single “flower” node for your RS-485 show network.
It drives a DC actuator (IBT-2/H-bridge) and two WS2811 LED rings (inner/outer), speaks a tiny line-based protocol, and acknowledges commands deterministically.

---

## Features

* RS-485 half-duplex protocol on Arduino **UNO** hardware UART (9600 bps)
* **OPEN/CLOSE** default to *override* (software limits ignored; hardware protections & global timeout stay active)
* **RGB base color** per ring (`RGB`, `RGBIN`, `RGBOUT`)
* **LED intensity** (0–120 UI scale) per ring (`LED`, `LEDIN`, `LEDOUT`, aliases `LEDSET*`)
* **Non-blocking intensity ramps** (`LEDRAMP*`)
* **STATUS** snapshot with motion, limit inputs, PWM, LED state
* Minimal replies: **ACK** (or **PONG**) only for direct (non-broadcast) commands

---

## Hardware

### Pin map (UNO)

| Function                    | Pin          | Notes                                   |
| --------------------------- | ------------ | --------------------------------------- |
| RS-485 DE/RE (direction)    | `D2`         | HIGH=TX, LOW=RX                         |
| RS-485 UART                 | `D0/D1` (HW) | `Serial` (MAX485 RO↔RX, DI↔TX)          |
| Motor OUT (extend PWM)      | `D5` (RPWM)  | IBT-2 / H-bridge input                  |
| Motor OUT (retract PWM)     | `D6` (LPWM)  | IBT-2 / H-bridge input                  |
| Extend limits (A/B/C)       | `D8/D10/D12` | **Active-LOW**, suggest pull-ups        |
| Retract limits (A/B/C)      | `D9/D11/D13` | **Active-LOW**, suggest pull-ups        |
| Inner WS2811 data           | `D3`         | `COLOR_ORDER=BRG`                       |
| Outer WS2811 data           | `D4`         | `COLOR_ORDER=BRG`                       |
| Ultrasonic TRIG/ECHO (opt.) | `A2/A3`      | Disabled by default (`USE_US_SAFETY=0`) |

> **Power**: WS2811 rings need a solid 5 V rail with proper common ground to the UNO and the IBT-2 supply. Inject power to rings where needed. Level-shift data if your strips are 5 V TTL sensitive.

### RS-485 bus

* Topology: **daisy chain** (no stars), 120 Ω termination at **both ends**, biasing at one point.
* Broadcast addressing uses ID `0` (nodes **do not** reply).
* Each node has a unique `myID` (`const int myID = 1;`).

---

## Build & Setup

1. **Libraries**

   * [FastLED](https://github.com/FastLED/FastLED)

2. **Config constants (top of sketch)**

   * `myID` → your node address
   * `NUM_LEDS_INNER` / `NUM_LEDS_OUTER`
   * `LED_PIN_INNER` / `LED_PIN_OUTER` (default `3`/`4`)
   * `COLOR_ORDER` (default `BRG` for WS2811)
   * `DEBUG` → set to **0** to keep the bus clean

3. **Upload**

   * Disconnect the MAX485 from `D0/D1` (or power it down) while flashing, since these are the UNO’s programming pins.

4. **Power-on**

   * Node starts **closed** (logic) with LEDs off (intensity `0` but base RGB = white).
   * Software limit checks are ignored for **OPEN/CLOSE** (override), but a **global safety timeout** still stops the motor:

     * `motorSafetyTimeoutMs = 15000` ms.

---

## Protocol

### Frame format

```
<ID>/<COMMAND>[ :VALUE | ,VALUE ]
```

* `ID=0` → broadcast (no replies)
* For non-broadcast: node replies **only** `ID/ACK` or `ID/PONG`
* **Line terminated** by `\n` or `\r\n`

### Commands

#### Link/health

| Command  | Value | Reply     | Notes                                      |
| -------- | ----- | --------- | ------------------------------------------ |
| `PING`   | –     | `ID/PONG` | Direct only                                |
| `STATUS` | –     | full line | Direct only; detailed snapshot (see below) |

#### Motion (override by default)

| Command          | Value            | Reply    | Notes                                                         |
| ---------------- | ---------------- | -------- | ------------------------------------------------------------- |
| `OPEN` or `EXT`  | `[speed 0..255]` | `ID/ACK` | Starts **extend** with **override** (ignores software limits) |
| `CLOSE` or `RET` | `[speed 0..255]` | `ID/ACK` | Starts **retract** with **override**                          |
| `STOP`           | –                | `ID/ACK` | Stops motor immediately                                       |
| `SPEED`          | `0..255`         | `ID/ACK` | Sets default speed for subsequent OPEN/CLOSE                  |

> Override means software limit checks are bypassed; **hardware** protection & **timeout** still apply.

#### LEDs — Base RGB (0..255)

Sets the **base color** that intensity scales against.

| Command  | Value   | Reply                 |
| -------- | ------- | --------------------- |
| `RGB`    | `r,g,b` | `ID/ACK` (both rings) |
| `RGBIN`  | `r,g,b` | `ID/ACK` (inner only) |
| `RGBOUT` | `r,g,b` | `ID/ACK` (outer only) |

#### LEDs — Intensity set (0..120 → mapped to 0..255)

Scales **current base RGB**.

| Command                    | Value    | Reply            |
| -------------------------- | -------- | ---------------- |
| `LED` **/ `LEDSET`**       | `0..120` | `ID/ACK` (both)  |
| `LEDIN` **/ `LEDSETIN`**   | `0..120` | `ID/ACK` (inner) |
| `LEDOUT` **/ `LEDSETOUT`** | `0..120` | `ID/ACK` (outer) |

#### LEDs — Intensity ramp (non-blocking)

Value in `0..120`, duration in ms. Ramps intensity while preserving base RGB.

| Command      | Value    | Reply            |
| ------------ | -------- | ---------------- |
| `LEDRAMP`    | `val,ms` | `ID/ACK` (both)  |
| `LEDRAMPIN`  | `val,ms` | `ID/ACK` (inner) |
| `LEDRAMPOUT` | `val,ms` | `ID/ACK` (outer) |

---

## Examples

### Motion

```
1/OPEN           -> 1/ACK
1/CLOSE:200      -> 1/ACK
1/STOP           -> 1/ACK
```

### Health

```
1/PING           -> 1/PONG
1/STATUS         -> 1/STATUS:motion=IDLE,speed=200,rpwm=0,lpwm=0,extend=0,0,0,retract=1,0,0,iRGB=255,255,255,iBright=0,oRGB=255,255,255,oBright=0,de_re=LOW,uptime=123456
```

### LEDs

```
1/RGB:33,212,55      -> 1/ACK        (set both bases)
1/RGBIN:33,232,55    -> 1/ACK
1/LED:15             -> 1/ACK        (both rings to ~15/120 of base)
1/LEDOUT:60          -> 1/ACK
1/LEDRAMP:120,1500   -> 1/ACK        (both to full over 1.5s)
```

> The parser also accepts comma in place of colon after the command (`RGB,33,212,55`) for convenience.

---

## STATUS format (example)

```
ID/STATUS:
  motion=IDLE|EXT|RET,
  speed=<default 0..255>,
  rpwm=<0..255>,
  lpwm=<0..255>,
  extend=A,B,C (1=active),
  retract=A,B,C (1=active),
  iRGB=R,G,B,
  iBright=0..255,
  oRGB=R,G,B,
  oBright=0..255,
  de_re=HIGH|LOW,
  uptime=<ms>
```

Use extend/retract bits to infer “open/closed” endpoints.

---

## Operational Notes

* **Silence the bus**: set `#define DEBUG 0`. With debug off, nodes only emit `ACK`, `PONG`, or `STATUS` on request.
* **Broadcast** (`ID=0`): nodes **never reply**.
* **Timeout safety**: motion auto-stops after `15 s` (`motorSafetyTimeoutMs`), even in override.
* **LED scale**: 0..120 UI is mapped to 0..255 internally with rounding; scaling is applied to the **current base RGB**.

---

## Troubleshooting

* **Can’t upload**: Temporarily disconnect MAX485 from `D0/D1` or power it down; those are the UNO’s programming lines.
* **No responses**:

  * Check `DE/RE` pin wiring (`D2`) and idle level `LOW` for RX.
  * Verify termination/biasing and common ground on the RS-485 bus.
  * Ensure correct `myID` and that you’re not broadcasting (`ID != 0`) when expecting replies.
* **LED flicker**: Add a large electrolytic (≥1000 µF) near strips, a small series resistor (≈220 Ω) on data line, and a solid ground.

---

## Configuration Quick Reference

* `myID` — node address
* `NUM_LEDS_INNER / OUTER` — pixel counts
* `LED_PIN_INNER / OUTER` — data pins (default `3`/`4`)
* `COLOR_ORDER` — WS2811 default `BRG`
* `defaultSpeed` — initial motor PWM
* `motorSafetyTimeoutMs` — 15000 ms
* `USE_US_SAFETY` — 0 (off) / 1 (on)

---

## License

Choose your preferred license. Example: MIT.

---

## Changelog (this variant)

* OPEN/CLOSE use **override** by default
* Added `RGB`, `RGBIN`, `RGBOUT` base color commands
* `LED*` & `LEDRAMP*` scale intensity **against base RGB**
* Replies trimmed to **ACK/PONG** only (except `STATUS`)
* Optional ultrasonic stall safety left available behind a flag
