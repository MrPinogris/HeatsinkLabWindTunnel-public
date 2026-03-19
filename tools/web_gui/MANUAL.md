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

## Normal Mode — Controls
- **Setpoint**: Target temperature in °C
- **SMART mode**: Automatically ramps to setpoint and holds
- **MANUAL mode**: Direct PWM control
- **Fan PWM**: Controls the cooling fan speed

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
