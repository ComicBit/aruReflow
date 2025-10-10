# Reflow Oven Controller (Arduino)

A compact and robust **reflow oven controller** for Arduino featuring:

- Dual **NTC 100k B3950** thermistors (A0 & A1)
- PID temperature control (`PID_v2`) with auto-tuning (`sTune`)
- 16x2 I²C LCD display (`LiquidCrystal_I2C`)
- Two buttons for mode selection and control
- Buzzer feedback
- Active-LOW SSR output (PWM control)

Implements a standard reflow profile:  
**Preheat → Soak → Reflow → Cooldown**  
plus a **PID auto-tuning** mode.

---

## ✨ Features

- Dual temperature sensing with averaging and smoothing
- PID control with auto-tune
- Simple 2-button UI with buzzer feedback
- LCD display with temperature, setpoint, state, and timing
- Built-in safety temperature limit
- Modular reflow phase logic

---

## 🧰 Hardware

### Bill of Materials

- Arduino Uno / Nano  
- 2 × NTC Thermistors (100 kΩ @ 25 °C, B3950)  
- 2 × 100 kΩ resistors  
- SSR (zero-cross AC type)  
- 16x2 I²C LCD (0x27 by default)  
- Piezo buzzer  
- 2 × momentary buttons

### Pinout

| Function              | Pin   | Notes                                               |
|------------------------|-------|-----------------------------------------------------|
| Button: Select         | D10   | `INPUT_PULLUP`                                      |
| Button: Start/Stop     | D9    | `INPUT_PULLUP`                                      |
| SSR (PWM)              | D4    | Active-LOW                                          |
| Buzzer                 | D6    | `tone()` used                                       |
| Thermistor 1           | A0    | Voltage divider                                    |
| Thermistor 2           | A1    | Voltage divider                                    |
| I²C LCD (SDA/SCL)      | A4/A5 | Address 0x27 by default (change in code if needed) |

---

## 📦 Required Libraries

Install via Arduino Library Manager or manually:

- `LiquidCrystal_I2C`  
- `PID_v2`  
- `sTune`  
- `Wire` (built-in)

---

## ⚙️ Build & Flash

1. Open the sketch in Arduino IDE  
2. Select the correct board and port  
3. Adjust the analog reference and `VCC` if needed  
4. Upload the sketch

---

## 🕹️ Usage

### Modes

| Mode | Name          | Description                     |
|------|---------------|-----------------------------------|
| 0    | Idle          | Standby mode                      |
| 1    | Reflow Mode   | Runs full temperature profile     |
| 2    | PID Tuning    | Auto-tunes PID parameters         |

### Buttons

- **Select (D10)** — cycles through modes  
- **Start/Stop (D9)** — starts or cancels the selected mode

### LCD

- Top line: temperature and SSR state  
- Bottom line: setpoint, PWM, time  
- Transition messages during phase changes  
- Beeps on start, stop, and completion

---

## 🔥 Default Reflow Profile

| Phase    | Setpoint (°C) | Time (s) |
|----------|---------------|----------|
| Preheat  | 100           | 60       |
| Soak     | 150           | 90       |
| Reflow   | 180           | 40       |
| Cooldown | 0            | 10       |

- `cooldown_temp = 40` °C  
- `tempLimit = 200` °C  

---

## 🧠 PID & Auto-Tune

- PID output range: `0–255`  
- SSR is active-LOW, so the PWM is inverted
- PID tuning uses `sTune`:
  1. Select **PID Tuning**
  2. Press Start with an empty oven
  3. Auto-tune runs and applies new Kp, Ki, Kd
  4. Results are printed over Serial (9600 baud)

---

## 🧪 Sensor Calibration

- Thermistor model: `100k / B3950`  
- Series resistor: 100k  
- Adjust constants in the code if your hardware differs.  
- Optional 2-point calibration (0 °C & 100 °C) for improved accuracy.

---

## 🧾 Serial Output

- Baud rate: **9600**  
- Logs phase transitions, PID tuning results, and emergency stops.

---

## 🛡️ Safety

- Use properly rated SSR and fusing.  
- Keep mains wiring enclosed and grounded.  
- Set a safe `tempLimit` for your oven.

---

## 🧭 Troubleshooting

| Issue                     | Cause                               | Fix                                                |
|---------------------------|--------------------------------------|----------------------------------------------------|
| LCD not displaying         | Wrong I²C address                   | Change address (e.g. 0x3F)                         |
| SSR stuck ON/OFF           | SSR logic mismatch                  | Adjust inversion in code                           |
| Tuning unstable            | Parameters too aggressive           | Adjust `outputStep` or `inputSpan`                 |
| Overshoot                  | PID values too high                | Retune or lower gains                              |

---

## ⚡ Quick Config Reference

- **Phases:** `preheat_setpoint`, `soak_setpoint`, `reflow_setpoint`  
- **Limits:** `tempLimit`, `cooldown_temp`  
- **PID tuning:** `inputSpan`, `outputStep`, `settleTimeSec`  

---

## 📜 License

Add your preferred license here (e.g., MIT).

---

## 🙏 Acknowledgments

- Brett Beauregard — PID library  
- `sTune` library authors  
- Reflow community projects for inspiration

---
