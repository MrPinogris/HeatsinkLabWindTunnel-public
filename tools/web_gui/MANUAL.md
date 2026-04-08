# HeatsinkLab Wind Tunnel — User Manual

## Requirements
- Windows 10 or 11
- Python 3.10 or newer ([python.org](https://python.org) or `winget install Python.Python.3.12`)
- No other software needed

## Installation
1. Download and unzip the project from GitHub
2. Open a Command Prompt in the `tools/web_gui` folder
3. Run: `python -m pip install -r requirements.txt`
4. Or simply double-click `start.bat`

## Starting the server
Double-click `start.bat` — a browser window will open at http://localhost:8000

## Connecting to the device
1. Plug in the ESP32-S3 via USB
2. Select the correct COM port from the dropdown
3. Click Connect — the telemetry chart will start updating

> **No hardware?** Select **VIRTUAL** from the port dropdown to use the built-in simulator.

## Normal Mode — Controls
- **Setpoint**: Target temperature in °C
- **SMART mode**: Automatically ramps to setpoint and holds
- **MANUAL mode**: Direct PWM control
- **Fan PWM**: Controls the cooling fan speed

## Run Metadata
Before starting CSV recording you can optionally fill in:
- **Run ID** — a short label (e.g. `run_001`) written into the CSV header
- **Heatsink** — identifies the heatsink under test (e.g. `HS_finned_A`)

These values are saved as comment lines at the top of the CSV file:
```
# schema_version: 1
# run_id: run_001
# heatsink_id: HS_finned_A
# start_time: 2026-03-29T...
```

## CSV Column Customization
Clicking **Start CSV** opens a dialog where you can select exactly which columns to record. Columns are grouped by category (Basic, Temperature, PID Details, Smart Mode, Fan). An Excel Power Query import formula is generated automatically based on your selection.

## Phase Markers
While recording, click **📌 Marker** to stamp the current moment with a label. The marker:
- Adds an orange dashed vertical line to the chart
- Writes a row to the CSV with the label in the `event` column

## Presets
The **Presets** section in the sidebar lets you save and restore named configurations:
- Three built-in presets are provided (Default, Aggressive, Conservative)
- **Save Current** — saves all parameter values under a name you choose
- **Load** — applies the selected preset immediately
- **Delete** — removes a saved preset

Presets are stored in `tools/web_gui/presets.json`.

## Expert / Student Mode
Click **🎓 Student** in the top bar to hide advanced parameters (ALPHA, MAXSTEP, ENTERCNT, EXITCNT, BIAS, SPBIAS, PID debug series, and advanced telemetry fields). The toggle state is remembered across sessions.

## Performance Metrics
The **Performance Metrics** panel computes step-response statistics from the current chart history:

| Metric | Meaning |
|--------|---------|
| Rise Time | Time to reach 90% of the setpoint step |
| Settle Time | Time until temperature stays within ±2°C of setpoint for 10 s |
| Overshoot % | Peak exceedance relative to step size |
| SS Error °C | Mean steady-state error over the last 20 samples |
| IAE (°C·s) | Integral of absolute error — lower is better |

Click **Compute** or enable **Auto on SP change** to trigger automatically when the setpoint changes by more than 2°C.

## Graph Cursor / Probe Tool
Click **Cursor: OFF** in the chart controls to enable a crosshair overlay. While active:
- Hovering over the chart shows all visible series values at that time
- Clicking the chart **pins** the tooltip; click again to unpin

## Tester Mode
For running systematic heatsink characterisation tests:
1. Click **Tester** in the top bar
2. Set the ambient temperature (room temperature in °C)
3. Optionally set fan speed (0 = natural convection)
4. Add test temperatures (e.g. 40, 50, 60, 70°C) using the [+] button
5. Set soak time (how long to wait at each temperature once stable, default 60s)
6. Click **▶ Start Test** — the system will automatically step through each temperature
7. When complete, download:
   - **Raw CSV**: all telemetry data during the test
   - **Results CSV**: one row per temperature with EqPWM, Power, R_th, k

## Understanding the Results CSV
| Column | Meaning |
|--------|---------|
| SP_degC | Setpoint temperature |
| Temp_actual | Measured heatsink temperature at equilibrium |
| Ambient_degC | Room temperature you entered |
| EqPWM | Equilibrium PWM (0-255) — heater duty cycle at balance |
| Power_W | Heater power consumption in Watts |
| R_th | Thermal resistance °C/W — lower is better cooling |
| k | Cooling coefficient W/°C |

## Troubleshooting
- **ModuleNotFoundError: serial** → use `start.bat` instead of running `python server.py` directly
- **Cannot connect to COM port** → check Device Manager, try a different USB cable
- **Temperature reads 0** → check NTC thermistor wiring
- **Presets not saving** → ensure the `tools/web_gui` folder is writable
