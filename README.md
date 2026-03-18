# HeatsinkLab Wind Tunnel

ESP32-S3 based heatsink characterisation platform with PID temperature control, INA226 power monitoring, and a browser-based GUI.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | ESP32-S3 DevKit C1 |
| Temperature | MAX6675 thermocouple (SPI) |
| Current/Power | INA226 (I2C, addr 0x40) |
| Heater MOSFET | Pin 4 (PWM 500 Hz, 8-bit) |
| Fan PWM | Pin 5 (PWM 500 Hz, 8-bit) |
| I2C SDA | Pin 15 |
| I2C SCL | Pin 16 |
| Thermocouple SCK | Pin 13 |
| Thermocouple CS | Pin 12 |
| Thermocouple SO | Pin 11 |

### INA226 Wiring
```
PSU +  →  Heater  →  INA226 IN+
                      INA226 IN-  →  MOSFET drain
                                      MOSFET source  →  GND  →  PSU -
INA226 VCC  →  3.3 V
INA226 GND  →  GND (common with ESP32)
INA226 SDA  →  Pin 15
INA226 SCL  →  Pin 16
```

---

## Flashing the ESP32

Requires [PlatformIO](https://platformio.org/).

```bash
# Install PlatformIO CLI or use the VS Code extension
pio run -t upload
```

---

## Web GUI

Runs in any browser. No extra software needed beyond Python.

### Install
```bash
cd tools/web_gui
pip install -r requirements.txt
```

### Run
```bash
python server.py
```

Browser opens automatically at **http://localhost:8765**

### Features
- Live dual-axis chart (temperature + PWM)
- PID parameter tuning with sliders
- AUTO / MANUAL / SMART control modes
- INA226 current, voltage and power readout
- CSV data logging
- Color-coded serial log

---

## Control Modes

| Mode | Behaviour |
|---|---|
| AUTO | PID loop controls heater PWM to reach setpoint |
| MANUAL | Fixed PWM set by `MANPWM` parameter |
| SMART | PID until stable, then holds last PWM value |
