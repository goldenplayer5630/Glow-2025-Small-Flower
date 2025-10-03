# 🌸 Flower Controller – Serial Command Reference

Each flower is controlled by an Arduino Nano with a unique **ID**.
Commands are sent over serial in the format:

```
<ID>/<COMMAND>[ :VALUE[,DURATION] ]
```

* `<ID>` → the numeric ID of the Nano (e.g., `12`)
* `0` as the ID means **broadcast to all flowers**
* Commands are **case sensitive**
* Each command must end with a newline `\n`

---

## 🔧 Initialization

```text
12/INIT
```

* Runs the startup sequence: wiggle motor, blink LEDs, move to closed position.

---

## 🔧 Motor Control

```text
12/OPEN
12/CLOSE
```

* `OPEN` → runs motor until the **open hall sensor** is triggered.
* `CLOSE` → runs motor until the **close hall sensor** is triggered.
* **No LED changes** happen with these commands.

---

## 💡 LED Control (Instant)

```text
12/LED:50
12/LED:120
12/LED:0
```

* Sets LED brightness instantly to the given value (`0–120`).
* `0` turns off all LEDs.

---

## 💡 LED Control (Ramped)

```text
12/LEDRAMP:120,1500
12/LEDRAMP:0,1500
```

* Smoothly fades LEDs to the target brightness over the given duration (ms).
* Example: `12/LEDRAMP:120,1500` → ramp up to 120 brightness over 1.5 seconds.

---

## 🌸 Combined Motor + LED Control

```text
12/OPENLEDRAMP:120,5000
12/CLOSELEDRAMP:0,5000
```

* `OPENLEDRAMP:<brightness>,<time>`
  → Starts motor **open** and ramps LEDs to target brightness over time.

* `CLOSELEDRAMP:<brightness>,<time>`
  → Starts motor **close** and ramps LEDs to target brightness over time.

* Example: `12/OPENLEDRAMP:120,5000` → open flower while LEDs ramp up to 120 brightness over 5 seconds.

---

## ✅ Notes

* Motor and LED ramps run **in parallel** thanks to a non-blocking state machine.
* Sending a new command while another is in progress will **override** the current action.
* Use `0/COMMAND` to broadcast to all flowers.

---

