# HeatsinkLab Web GUI — User Manual

## Requirements

- Python 3.10 or newer
- Windows, macOS, or Linux

---

## Installation

### 1. Install Python
Download from https://www.python.org/downloads/ — check **"Add Python to PATH"** during install.

### 2. Get the project files
Either clone the repository or download and extract the ZIP from GitHub.

### 3. Install Python packages
Open a terminal/command prompt in the project folder and run:
```
pip install fastapi uvicorn pyserial
```

---

## Starting the server

Navigate to `tools/web_gui/` and run:
```
python server.py
```

You will see output like:
```
INFO:     Uvicorn running on http://0.0.0.0:8765
```

Open your browser and go to: **http://localhost:8765**

> The server must keep running in the background while you use the GUI. Do not close the terminal window.

---

## Connecting to the device

1. Plug in the ESP32-S3 via USB
2. Click the **refresh button** (↻) next to the Port dropdown
3. Select the correct COM port (e.g. `COM3` on Windows, `/dev/ttyUSB0` on Linux)
4. Select baud rate **115200**
5. Click **Connect**

The status pill in the top bar will turn green when connected.

> If you select the wrong port, click **Disconnect** and try again.

---

## Normal Mode — Controls Overview

### Controller Parameters
| Parameter | Description |
|-----------|-------------|
| KP / KI / KD | PID tuning gains |
| SP | Setpoint temperature (°C) |
| SPBIAS | Offset added to SP |
| ALPHA | Low-pass filter strength (0=slow, 1=raw) |
| MAXSTEP | Max PWM change per cycle |
| ENTERCNT | Stability count before entering HOLD |
| EXITCNT | Drift count before exiting HOLD |
| FAN | Fan speed (0–100%) |
| MANPWM | PWM value used in MANUAL mode |

### Modes
- **AUTO** — PID runs continuously
- **MANUAL** — Fixed PWM (set via MANPWM)
- **SMART** — PID until stable, then locks PWM in HOLD. Automatically re-engages PID if temp drifts.

### Chart series
Check/uncheck series in the chart settings bar. Default visible: Smooth temp, Setpoint, PWM.
- **PID Debug** button reveals P/I/D/PID Output series for tuning.

### CSV Logging
Click **Start CSV** to begin logging all telemetry to a file in the `logs/` folder.
Click **Stop CSV** to close the file.

---

## Tester Mode

Click **🔬 Tester** in the top bar to enter Tester Mode. The normal controls sidebar is replaced by the tester panel. The graph remains visible.

### Setup
1. **Ambient (°C)** — enter the current room temperature (used to calculate R_th and k)
2. **Fan speed (%)** — set fan speed for this test run, then click **Set**
   - 0% = natural convection
   - Higher = forced airflow
3. **Test temperatures** — click **+ Add** to add each setpoint you want to test (e.g. 40, 50, 60, 70°C)
4. **Soak time** — how many seconds to wait after the system enters HOLD before recording the result (default 60s)

### Running a test
1. Click **▶ Start Test**
2. The system will automatically:
   - Set SMART mode and start CSV logging
   - Set the first temperature setpoint
   - Wait for the heater to stabilize (Enter% reaches 100%)
   - Hold for the soak time
   - Record the result
   - Move to the next setpoint
3. Progress is shown in the **Live** panel
4. Results appear in the **Results** table as each step completes
5. When all steps are done: "✓ Test complete!" is shown

> If HOLD is lost during soak (temperature drifts), the soak countdown resets and waits for re-entry.

### Stopping early
Click **■ Stop** at any time to abort the test.

### Downloading results

After the test completes (or at any point):

- **↓ Results CSV** — downloads a summary table:
  ```
  SP_degC, Temp_actual, Ambient_degC, EqPWM, Power_W, R_th, k
  ```
  - `EqPWM` — the converged equilibrium PWM at that setpoint
  - `Power_W` — heater power at equilibrium
  - `R_th` — thermal resistance (°C/W): how much temp rises per watt
  - `k` — cooling coefficient (W/°C): inverse of R_th

- **↓ Raw CSV** — downloads the full raw telemetry log (all fields, every 0.5s sample)

---

## Heatsink Section (Normal Mode)

The **Heatsink** panel in the sidebar shows live calculated characteristics:
- **R_th** (°C/W) — thermal resistance
- **k** (W/°C) — cooling coefficient
- **T_max** (°C) — estimated max temperature at full power
- **τ** — time constant (shown as — until a step test is added)

These update automatically when the system is in HOLD and EqPWM has converged.

---

## Virtual Mode (for testing without hardware)

Select **VIRTUAL** from the port dropdown and click **Connect**. A simulated ESP32 with a thermal physics model will appear. Use the **⚡ VIRTUAL MODE** panel to adjust speed (1×–50×) and physics parameters.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Can't find COM port | Try a different USB cable, check Device Manager |
| GUI won't load | Make sure `server.py` is running and you're on http://localhost:8765 |
| INA226 reads zero | Check I2C wiring (SDA=pin 15, SCL=pin 16) |
| Temperature stuck | Check thermocouple wiring and connections |
| SMART mode never enters HOLD | Setpoint may be too far from stable range; check KP/KI/KD tuning |
