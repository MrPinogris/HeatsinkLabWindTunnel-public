#!/usr/bin/env python3
"""FastAPI + WebSocket backend for HeatsinkLab PID Web GUI."""

from __future__ import annotations

import asyncio
import csv
import datetime as dt
import json
import re
import threading
import time
from pathlib import Path
from typing import Any

import serial
import serial.tools.list_ports
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

# ---------------------------------------------------------------------------
# Regex patterns (identical to pid_gui.py)
# ---------------------------------------------------------------------------
TELEMETRY_RE = re.compile(
    r"Rawtemp\s+([-+]?\d*\.?\d+)\D+"
    r"Temp:\s*([-+]?\d*\.?\d+)\D+"
    r"Smooth:\s*([-+]?\d*\.?\d+)\D+"
    r"PWM:\s*([-+]?\d+)\D+"
    r"(?:P:\s*([-+]?\d*\.?\d+)\D+)?"
    r"(?:I:\s*([-+]?\d*\.?\d+)\D+)?"
    r"(?:D:\s*([-+]?\d*\.?\d+)\D+)?"
    r"(?:OUT:\s*([-+]?\d*\.?\d+)\D+)?"
    r"(?:BIAS:\s*([-+]?\d*\.?\d+)\D+)?"
    r"(?:SPBIAS:\s*([-+]?\d*\.?\d+)\D+)?"
    r"SP:\s*([-+]?\d*\.?\d+)\D+"
    r"(?:EFFSP:\s*([-+]?\d*\.?\d+)\D+)?"
    r"FAN:\s*([-+]?\d*\.?\d+)\D+"
    r"FANPWM:\s*([-+]?\d+)"
    r"(?:\D+MODE:\s*(AUTO|MANUAL|SMART)\D+STATE:\s*(PID|HOLD|MANUAL)\D+MANPWM:\s*([-+]?\d*\.?\d+)\D+HOLDPWM:\s*([-+]?\d*\.?\d+)\D+ENTPROG:\s*([-+]?\d*\.?\d+)\D+EXTPROG:\s*([-+]?\d*\.?\d+)\D+EABS:\s*([-+]?\d*\.?\d+))?"
    r"(?:\D+RUN:\s*(ON|OFF)\D+FANINV:\s*([01]))?"
    r"(?:\D+V:\s*([-+]?\d*\.?\d+)\s*V\D+I:\s*([-+]?\d*\.?\d+)\s*A\D+W:\s*([-+]?\d*\.?\d+)\s*W)?"
    r"(?:\D+EQPWM:\s*([-+]?\d*\.?\d+))?"
)

CFG_RE = re.compile(
    r"CFG\s+KP:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"KI:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"KD:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"BIAS:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"SPBIAS:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"SP:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"ALPHA:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"MAXSTEP:\s*([-+]?\d+)\s*\|\s*"
    r"ENTCNT:\s*([-+]?\d+)\s*\|\s*"
    r"EXTCNT:\s*([-+]?\d+)\s*\|\s*"
    r"FAN:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"FANINV:\s*([01])\s*\|\s*"
    r"MODE:\s*(AUTO|MANUAL|SMART)\s*\|\s*"
    r"MANPWM:\s*([-+]?\d*\.?\d+)\s*\|\s*"
    r"RUN:\s*(ON|OFF)"
)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
HERE = Path(__file__).parent
PROJECT_ROOT = HERE.parent.parent
LOGS_DIR = PROJECT_ROOT / "logs"
STATE_PATH = HERE / "pid_gui_state.json"
PRESETS_PATH = HERE / "presets.json"
STATIC_DIR = HERE / "static"

# ---------------------------------------------------------------------------
# App state
# ---------------------------------------------------------------------------
app = FastAPI(title="HeatsinkLab PID Web GUI")

# Mount static files
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


class AppState:
    def __init__(self) -> None:
        self.serial_conn: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()

        self.connected = False
        self.port = ""
        self.baud = 115200

        self.awaiting_handshake = False
        self.handshake_ok = False
        self.handshake_deadline = 0.0

        self.csv_logging = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_path: Path | None = None
        self.csv_start_monotonic = 0.0
        self.csv_columns: list[str] | None = None   # None = all columns
        self.pending_run_id: str = ""
        self.pending_heatsink_id: str = ""

        self.websockets: list[WebSocket] = []
        self.ws_lock = asyncio.Lock()

        self.loop: asyncio.AbstractEventLoop | None = None
        self._virt: "VirtualMCU | None" = None

        # Latest telemetry snapshot
        self.last_telemetry: dict[str, Any] = {}
        self.last_cfg: dict[str, Any] = {}

        # Heatsink characterization
        self.ambient_temp: float = 20.0
        self._prev_eq_pwm: float = 0.0

        # Saved UI state
        self.saved_state: dict[str, str] = self._load_state()

    # ------------------------------------------------------------------
    # State persistence
    # ------------------------------------------------------------------
    def _load_state(self) -> dict[str, str]:
        if not STATE_PATH.exists():
            return {}
        try:
            data = json.loads(STATE_PATH.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                return {str(k): str(v) for k, v in data.items()}
        except (OSError, json.JSONDecodeError):
            pass
        return {}

    def save_state(self, state: dict[str, str]) -> None:
        self.saved_state.update(state)
        try:
            STATE_PATH.write_text(json.dumps(self.saved_state, indent=2), encoding="utf-8")
        except OSError:
            pass

    # ------------------------------------------------------------------
    # WebSocket broadcast
    # ------------------------------------------------------------------
    async def broadcast(self, msg: dict) -> None:
        if not self.websockets:
            return
        text = json.dumps(msg)
        dead: list[WebSocket] = []
        async with self.ws_lock:
            for ws in list(self.websockets):
                try:
                    await ws.send_text(text)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.websockets.remove(ws)

    def broadcast_sync(self, msg: dict) -> None:
        """Thread-safe broadcast from serial reader thread."""
        if self.loop and not self.loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.broadcast(msg), self.loop)

    # ------------------------------------------------------------------
    # Serial
    # ------------------------------------------------------------------
    def connect(self, port: str, baud: int) -> tuple[bool, str]:
        if self.connected:
            return False, "Already connected"
        # Signal any lingering reader thread to exit (don't join — it's a daemon)
        self.reader_stop.set()
        self.reader_thread = None
        # Close any leftover serial port
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

        # Virtual port — no real serial needed
        if port == "VIRTUAL":
            self.port = "VIRTUAL"
            self.baud = baud
            self.connected = True
            self._virt = VirtualMCU(self)
            self._virt.start()
            self.broadcast_sync({"type": "handshake_ok"})
            return True, "Connected to VIRTUAL simulator"

        try:
            self.serial_conn = serial.Serial()
            self.serial_conn.port = port
            self.serial_conn.baudrate = baud
            self.serial_conn.timeout = 0.2
            self.serial_conn.dtr = False
            self.serial_conn.rts = False
            self.serial_conn.open()
            self.port = port
            self.baud = baud
            self.connected = True
            self.awaiting_handshake = True
            self.handshake_ok = False
            self.handshake_deadline = time.monotonic() + 2.5
            self.reader_stop.clear()
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            # Request current config
            self._write_serial("GET")
            return True, f"Connected to {port} @ {baud}"
        except Exception as exc:
            self.serial_conn = None
            self.connected = False
            return False, str(exc)

    def disconnect(self) -> None:
        if self._virt:
            self._virt.stop()
            self._virt = None
        self.reader_stop.set()
        self.connected = False
        self.awaiting_handshake = False
        self.handshake_ok = False
        if self.serial_conn:
            try:
                self.serial_conn.dtr = False
                self.serial_conn.rts = False
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        self.stop_csv_logging()

    def _write_serial(self, command: str) -> tuple[bool, str]:
        if not self.serial_conn or not self.serial_conn.is_open:
            return False, "Not connected"
        try:
            self.serial_conn.write((command + "\n").encode("utf-8"))
            return True, "OK"
        except Exception as exc:
            return False, str(exc)

    def send_command(self, command: str) -> tuple[bool, str]:
        self.broadcast_sync({"type": "log", "text": f"> {command}"})
        if self._virt:
            self._virt.handle_command(command)
            return True, "OK"
        ok, msg = self._write_serial(command)
        return ok, msg

    # ------------------------------------------------------------------
    # Reader loop (runs in background thread)
    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        while not self.reader_stop.is_set():
            if not self.serial_conn:
                break
            try:
                line = self.serial_conn.readline().decode("utf-8", errors="replace").strip()
            except Exception as exc:
                self.broadcast_sync({"type": "log", "text": f"Serial read error: {exc}"})
                self.broadcast_sync({"type": "disconnected", "reason": str(exc)})
                self.connected = False
                break

            if not line:
                continue

            self.broadcast_sync({"type": "log", "text": line})
            self._handle_line(line)

            # Handshake check
            if self.awaiting_handshake:
                if line.startswith("CFG ") or line.startswith("Rawtemp "):
                    self.handshake_ok = True
                    self.awaiting_handshake = False
                    self.broadcast_sync({"type": "handshake_ok"})
                elif time.monotonic() > self.handshake_deadline:
                    self.awaiting_handshake = False
                    self.connected = False
                    self.reader_stop.set()
                    if self.serial_conn:
                        try:
                            self.serial_conn.close()
                        except Exception:
                            pass
                        self.serial_conn = None
                    self.broadcast_sync({"type": "handshake_fail"})
                    self.broadcast_sync({"type": "disconnected", "reason": "handshake_fail"})
                    break

    def _handle_line(self, line: str) -> None:
        m = TELEMETRY_RE.search(line)
        if m:
            raw = float(m.group(1))
            temp = float(m.group(2))
            smooth = float(m.group(3))
            pwm = float(m.group(4))
            p_term = float(m.group(5) or 0.0)
            i_term = float(m.group(6) or 0.0)
            d_term = float(m.group(7) or 0.0)
            out = float(m.group(8) or pwm)
            bias = float(m.group(9) or 0.0)
            spbias = float(m.group(10) or 0.0)
            sp = float(m.group(11))
            effsp = float(m.group(12) or sp + spbias)
            fan_speed = float(m.group(13))
            fan_pwm = float(m.group(14))
            ctrl_mode = m.group(15) or "AUTO"
            ctrl_state = m.group(16) or ("MANUAL" if ctrl_mode == "MANUAL" else "PID")
            manpwm = float(m.group(17) or 0.0)
            holdpwm = float(m.group(18) or 0.0)
            enter_prog = float(m.group(19) or 0.0)
            exit_prog = float(m.group(20) or 0.0)
            abs_err = float(m.group(21) or 0.0)
            run_state = m.group(22) or "ON"
            fan_inv = m.group(23) or "0"
            ina_voltage = float(m.group(24) or 0.0)
            ina_current = float(m.group(25) or 0.0)
            ina_power   = float(m.group(26) or 0.0)
            eq_pwm = float(m.group(27) or 0.0)

            now_iso = dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="milliseconds")

            telemetry: dict[str, Any] = {
                "type": "telemetry",
                "timestamp": now_iso,
                "ts": time.time(),
                "raw_temp": raw,
                "temp": temp,
                "smooth": smooth,
                "pwm": pwm,
                "p_term": p_term,
                "i_term": i_term,
                "d_term": d_term,
                "pid_out": out,
                "bias": bias,
                "spbias": spbias,
                "sp": sp,
                "effsp": effsp,
                "fan_speed": fan_speed,
                "fan_pwm": fan_pwm,
                "ctrl_mode": ctrl_mode,
                "ctrl_state": ctrl_state,
                "manpwm": manpwm,
                "holdpwm": holdpwm,
                "enter_prog": enter_prog,
                "exit_prog": exit_prog,
                "abs_err": abs_err,
                "run_state": run_state,
                "fan_inv": fan_inv,
                "ina_voltage": ina_voltage,
                "ina_current": ina_current,
                "power_w": ina_power,
                "eq_pwm": eq_pwm,
            }
            self.last_telemetry = telemetry
            self.broadcast_sync(telemetry)

            # Heatsink characterization — update when eq_pwm is converging
            # (converged = eq_pwm changed less than 0.5 PWM units since last call)
            eq_converged = abs(eq_pwm - self._prev_eq_pwm) < 0.5 and eq_pwm > 1.0
            self._prev_eq_pwm = eq_pwm
            if eq_converged:
                amb = self.ambient_temp
                delta_t = smooth - amb
                power_w = ina_power
                # If INA not connected, estimate from eq_pwm ratio and supply voltage
                if power_w <= 0.0 and eq_pwm > 0 and ina_voltage > 0 and ina_current > 0 and pwm > 0:
                    power_w = (eq_pwm / pwm) * ina_power
                if delta_t > 2.0 and power_w > 0.05:
                    r_th = delta_t / power_w
                    k = power_w / delta_t
                    # Estimate max power at full PWM from current operating point
                    max_power = (ina_power / max(pwm / 255.0, 0.01)) if pwm > 0 and ina_power > 0 else 15.0
                    t_max = amb + r_th * max_power
                    self.broadcast_sync({
                        "type": "heatsink_chars",
                        "r_th": round(r_th, 3),
                        "k": round(k, 4),
                        "t_max": round(t_max, 1),
                        "tau": None,
                        "airspeed": None,
                    })

            # CSV
            if self.csv_logging and self.csv_writer:
                elapsed = max(0.0, time.monotonic() - self.csv_start_monotonic)
                try:
                    self.csv_writer.writerow({
                        "timestamp_iso": now_iso,
                        "elapsed_s": f"{elapsed:.3f}",
                        "raw_temp_c": raw,
                        "temp_filtered_c": temp,
                        "temp_smooth_c": smooth,
                        "pwm": int(round(pwm)),
                        "p_term": p_term,
                        "i_term": i_term,
                        "d_term": d_term,
                        "pid_out": out,
                        "pid_bias": bias,
                        "setpoint_bias_c": spbias,
                        "setpoint_c": sp,
                        "effective_setpoint_c": effsp,
                        "fan_speed_pct": fan_speed,
                        "fan_pwm_raw": int(round(fan_pwm)),
                        "mode": ctrl_mode,
                        "state": ctrl_state,
                        "manual_pwm_cmd": manpwm,
                        "hold_pwm": holdpwm,
                        "enter_progress_pct": enter_prog,
                        "exit_progress_pct": exit_prog,
                        "abs_error_c": abs_err,
                        "run_state": run_state,
                        "fan_inverted": 1 if fan_inv == "1" else 0,
                        "event": "",
                    })
                    if self.csv_file:
                        self.csv_file.flush()
                except OSError:
                    self.stop_csv_logging()
            return

        c = CFG_RE.search(line)
        if c:
            cfg: dict[str, Any] = {
                "type": "cfg",
                "kp": c.group(1),
                "ki": c.group(2),
                "kd": c.group(3),
                "bias": c.group(4),
                "spbias": c.group(5),
                "sp": c.group(6),
                "alpha": c.group(7),
                "maxstep": c.group(8),
                "entercnt": c.group(9),
                "exitcnt": c.group(10),
                "fan": c.group(11),
                "fan_inv": c.group(12),
                "ctrl_mode": c.group(13),
                "manpwm": c.group(14),
                "run": c.group(15),
            }
            self.last_cfg = cfg
            self.broadcast_sync(cfg)

    # ------------------------------------------------------------------
    # CSV logging
    # ------------------------------------------------------------------
    CSV_FIELDNAMES = [
        "timestamp_iso", "elapsed_s", "raw_temp_c", "temp_filtered_c",
        "temp_smooth_c", "pwm", "p_term", "i_term", "d_term", "pid_out",
        "pid_bias", "setpoint_bias_c", "setpoint_c", "effective_setpoint_c",
        "fan_speed_pct", "fan_pwm_raw", "mode", "state", "manual_pwm_cmd",
        "hold_pwm", "enter_progress_pct", "exit_progress_pct", "abs_error_c",
        "run_state", "fan_inverted", "event",
    ]

    def start_csv_logging(self) -> tuple[bool, str]:
        if self.csv_logging:
            return False, "Already logging"
        if not self.connected:
            return False, "Not connected"
        LOGS_DIR.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = LOGS_DIR / f"serial_{stamp}.csv"
        try:
            self.csv_file = open(path, "w", newline="", encoding="utf-8")
            # Write metadata comment header
            start_iso = dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")
            self.csv_file.write(f"# schema_version: 1\n")
            self.csv_file.write(f"# run_id: {self.pending_run_id}\n")
            self.csv_file.write(f"# heatsink_id: {self.pending_heatsink_id}\n")
            self.csv_file.write(f"# start_time: {start_iso}\n")
            fieldnames = self.csv_columns or self.CSV_FIELDNAMES
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames, extrasaction="ignore")
            self.csv_writer.writeheader()
            self.csv_file.flush()
        except OSError as exc:
            self.csv_file = None
            self.csv_writer = None
            return False, str(exc)
        self.csv_path = path
        self.csv_start_monotonic = time.monotonic()
        self.csv_logging = True
        return True, str(path)

    def stop_csv_logging(self) -> None:
        self.csv_logging = False
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except OSError:
                pass
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.csv_columns = None  # reset for next session


# ---------------------------------------------------------------------------
# Virtual MCU simulator
# ---------------------------------------------------------------------------
import random

class VirtualMCU:
    """Simulates the ESP32 heater controller over a fake serial connection."""

    def __init__(self, app_state: "AppState") -> None:
        self._state = app_state
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # Simulated controller state
        self.sp = 60.0
        self.kp = 8.0
        self.ki = 0.06
        self.kd = 0.0
        self.bias = 0.0
        self.spbias = 0.0
        self.alpha = 0.1
        self.maxstep = 20
        self.entercnt = 30
        self.exitcnt = 10
        self.fan = 0.4
        self.fan_inv = 0
        self.mode = "SMART"
        self.manpwm = 30.0
        self.run = "ON"

        # Physics state
        self._temp = 20.0
        self._smooth = 20.0
        self._pwm = 0.0
        self._i_term = 0.0
        self._holdpwm = 0.0
        self._ctrl_state = "PID"
        self._enter_prog = 0.0
        self._exit_prog = 0.0
        self._t = 0.0

        self.speed = 1  # 1=realtime, 5=5x faster, etc.

        # Ambient / heater physics constants
        # Calibrated from real CSV: reaches 60°C in ~163s at PWM=255 from 21.5°C
        self._ambient = 20.0
        self._thermal_mass = 62.5    # J/°C  (gives initial rate 0.24°C/s at 15W)
        self._heater_power = 15.0    # max watts at PWM=255
        self._cooling_coeff = 0.002  # natural convection, small (heatsink can reach ~140°C at full power)

    def start(self) -> None:
        self._stop.clear()
        # Send initial CFG line
        self._broadcast_cfg()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def handle_command(self, cmd: str) -> None:
        cmd = cmd.strip().upper()
        if cmd == "GET":
            self._broadcast_cfg()
            return
        parts = cmd.split()
        if len(parts) == 3 and parts[0] == "SET":
            key, val = parts[1], parts[2]
            try:
                if key == "SP":       self.sp      = float(val)
                elif key == "KP":     self.kp      = float(val)
                elif key == "KI":     self.ki      = float(val)
                elif key == "KD":     self.kd      = float(val)
                elif key == "BIAS":   self.bias     = float(val)
                elif key == "SPBIAS": self.spbias   = float(val)
                elif key == "ALPHA":  self.alpha    = float(val)
                elif key == "MAXSTEP":self.maxstep  = int(float(val))
                elif key == "ENTERCNT":self.entercnt= int(float(val))
                elif key == "EXITCNT": self.exitcnt = int(float(val))
                elif key == "FAN":    self.fan      = float(val) / 100.0
                elif key == "FANINV": self.fan_inv  = int(val)
                elif key == "MODE":
                    if val in ("AUTO", "MANUAL", "SMART"):
                        self.mode = val
                elif key == "MANPWM": self.manpwm   = float(val)
                elif key == "RUN":
                    if val in ("ON", "OFF"):
                        self.run = val
                self._state.broadcast_sync({"type": "log", "text": f"OK {key} set to {val}"})
                self._broadcast_cfg()
            except ValueError:
                self._state.broadcast_sync({"type": "log", "text": f"ERR bad value for {key}"})

    def _broadcast_cfg(self) -> None:
        line = (
            f"CFG KP: {self.kp:.3f} | KI: {self.ki:.3f} | KD: {self.kd:.3f} | "
            f"BIAS: {self.bias:.2f} | SPBIAS: {self.spbias:.2f} | SP: {self.sp:.2f} | "
            f"ALPHA: {self.alpha:.3f} | MAXSTEP: {self.maxstep} | ENTCNT: {self.entercnt} | "
            f"EXTCNT: {self.exitcnt} | FAN: {self.fan*100:.1f} | FANINV: {self.fan_inv} | "
            f"MODE: {self.mode} | MANPWM: {self.manpwm:.2f} | RUN: {self.run}"
        )
        self._state.broadcast_sync({"type": "log", "text": line})
        self._state._handle_line(line)

    def _loop(self) -> None:
        dt_s = 1.0  # fixed physics step (1 simulated second per tick)
        prev_err = 0.0
        while not self._stop.is_set():
            time.sleep(max(0.05, 1.0 / self.speed))
            if self._stop.is_set():
                break

            effsp = self.sp + self.spbias
            err = effsp - self._temp

            # PID
            p = self.kp * err
            self._i_term += self.ki * err * dt_s
            self._i_term = max(-255.0, min(255.0, self._i_term))
            d = self.kd * (err - prev_err) / dt_s
            prev_err = err
            pid_out = p + self._i_term + d + self.bias

            if self.run == "OFF":
                self._pwm = 0.0
                self._ctrl_state = "PID"
            elif self.mode == "MANUAL":
                self._pwm = self.manpwm
                self._ctrl_state = "MANUAL"
            elif self.mode == "SMART":
                # simple smart: hold when stable, PID otherwise
                abs_err = abs(err)
                if abs_err < 1.0:
                    self._enter_prog = min(1.0, self._enter_prog + 1.0 / self.entercnt)
                else:
                    self._enter_prog = 0.0
                if self._enter_prog >= 1.0:
                    self._holdpwm = self._pwm
                    self._ctrl_state = "HOLD"
                    self._pwm = self._holdpwm
                else:
                    self._ctrl_state = "PID"
                    self._pwm = max(0.0, min(255.0, pid_out))
            else:  # AUTO
                self._pwm = max(0.0, min(255.0, pid_out))
                self._ctrl_state = "PID"

            # Physics: heater warms, ambient cools
            power_w = (self._pwm / 255.0) * self._heater_power
            cooling = self._cooling_coeff * (self._temp - self._ambient)
            delta_t = (power_w / self._thermal_mass - cooling) * dt_s
            noise = random.gauss(0, 0.05)
            self._temp += delta_t + noise
            self._smooth = self.alpha * self._temp + (1 - self.alpha) * self._smooth

            # Simulated power (heater only, ~12V)
            current_a = (self._pwm / 255.0) * (self._heater_power / 12.0)
            current_a = max(0.0, current_a + random.gauss(0, 0.003))
            voltage_v = 12.0
            power_calc = voltage_v * current_a

            fan_pwm_val = int((1.0 - self.fan) * 255) if self.fan_inv else int(self.fan * 255)
            abs_err_val = abs(effsp - self._temp)

            line = (
                f"Rawtemp {self._temp + random.gauss(0,0.1):.2f} C | "
                f"Temp: {self._temp:.2f} C | Smooth: {self._smooth:.2f} C | "
                f"PWM: {int(self._pwm)} | P: {p:.2f} | I: {self._i_term:.2f} | "
                f"D: {d:.2f} | OUT: {pid_out:.2f} | BIAS: {self.bias:.2f} | "
                f"SPBIAS: {self.spbias:.2f} | SP: {self.sp:.2f} | EFFSP: {effsp:.2f} | "
                f"FAN: {self.fan*100:.1f} | FANPWM: {fan_pwm_val} | "
                f"MODE: {self.mode} | STATE: {self._ctrl_state} | "
                f"MANPWM: {self.manpwm:.2f} | HOLDPWM: {self._holdpwm:.2f} | "
                f"ENTPROG: {self._enter_prog*100:.1f} | EXTPROG: {self._exit_prog*100:.1f} | "
                f"EABS: {abs_err_val:.2f} | RUN: {self.run} | FANINV: {self.fan_inv} | "
                f"V: {voltage_v:.3f} V | I: {current_a:.4f} A | W: {power_calc:.3f} W"
            )
            self._state.broadcast_sync({"type": "log", "text": line})
            self._state._handle_line(line)


# ---------------------------------------------------------------------------
# Preset helpers
# ---------------------------------------------------------------------------
def _load_presets() -> dict:
    if not PRESETS_PATH.exists():
        return {}
    try:
        return json.loads(PRESETS_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _save_presets(data: dict) -> None:
    PRESETS_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")


# Singleton
state = AppState()


# ---------------------------------------------------------------------------
# Startup / shutdown
# ---------------------------------------------------------------------------
@app.on_event("startup")
async def on_startup() -> None:
    state.loop = asyncio.get_event_loop()


@app.on_event("shutdown")
async def on_shutdown() -> None:
    state.disconnect()


# ---------------------------------------------------------------------------
# REST endpoints
# ---------------------------------------------------------------------------
@app.get("/")
async def root() -> FileResponse:
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.get("/api/ports")
async def list_ports() -> JSONResponse:
    ports = [{"device": p.device, "description": p.description}
             for p in serial.tools.list_ports.comports()]
    ports.insert(0, {"device": "VIRTUAL", "description": "Virtual simulator (no hardware needed)"})
    return JSONResponse({"ports": ports})


@app.post("/api/connect")
async def connect(payload: dict) -> JSONResponse:
    port = payload.get("port", "")
    baud = int(payload.get("baud", 115200))
    loop = asyncio.get_event_loop()
    try:
        ok, msg = await asyncio.wait_for(
            loop.run_in_executor(None, lambda: state.connect(port, baud)),
            timeout=5.0
        )
    except asyncio.TimeoutError:
        state.connected = False
        state.serial_conn = None
        ok, msg = False, f"Timed out opening {port} — wrong port or device busy"
    if ok:
        await state.broadcast({"type": "connected", "port": port, "baud": baud})
    return JSONResponse({"ok": ok, "msg": msg})


@app.post("/api/virtual/speed")
async def set_virtual_speed(payload: dict) -> JSONResponse:
    speed = int(payload.get("speed", 1))
    speed = max(1, min(50, speed))
    if state._virt:
        state._virt.speed = speed
        return JSONResponse({"ok": True, "speed": speed})
    return JSONResponse({"ok": False, "msg": "Not in virtual mode"})


@app.post("/api/ambient")
async def set_ambient(payload: dict) -> JSONResponse:
    try:
        state.ambient_temp = float(payload.get("ambient", 20.0))
    except (ValueError, TypeError) as e:
        return JSONResponse({"ok": False, "msg": str(e)})
    return JSONResponse({"ok": True, "ambient": state.ambient_temp})


@app.post("/api/virtual/physics")
async def set_virtual_physics(payload: dict) -> JSONResponse:
    if not state._virt:
        return JSONResponse({"ok": False, "msg": "Not in virtual mode"})
    v = state._virt
    try:
        if "heater_power" in payload:  v._heater_power  = float(payload["heater_power"])
        if "thermal_mass" in payload:  v._thermal_mass  = float(payload["thermal_mass"])
        if "cooling_coeff" in payload: v._cooling_coeff = float(payload["cooling_coeff"])
        if "ambient"       in payload: v._ambient       = float(payload["ambient"])
    except (ValueError, TypeError) as e:
        return JSONResponse({"ok": False, "msg": str(e)})
    return JSONResponse({"ok": True})


@app.get("/api/virtual/physics")
async def get_virtual_physics() -> JSONResponse:
    if not state._virt:
        return JSONResponse({"ok": False, "msg": "Not in virtual mode"})
    v = state._virt
    return JSONResponse({"ok": True, "heater_power": v._heater_power,
                         "thermal_mass": v._thermal_mass,
                         "cooling_coeff": v._cooling_coeff,
                         "ambient": v._ambient})


@app.post("/api/disconnect")
async def disconnect() -> JSONResponse:
    state.disconnect()
    await state.broadcast({"type": "disconnected", "reason": "user"})
    return JSONResponse({"ok": True})


@app.post("/api/command")
async def send_command(payload: dict) -> JSONResponse:
    cmd = payload.get("command", "").strip()
    if not cmd:
        return JSONResponse({"ok": False, "msg": "Empty command"})
    ok, msg = state.send_command(cmd)
    return JSONResponse({"ok": ok, "msg": msg})


@app.post("/api/csv/start")
async def start_csv() -> JSONResponse:
    ok, msg = state.start_csv_logging()
    return JSONResponse({"ok": ok, "msg": msg})


@app.post("/api/csv/stop")
async def stop_csv() -> JSONResponse:
    path = str(state.csv_path) if state.csv_path else ""
    state.stop_csv_logging()
    return JSONResponse({"ok": True, "path": path})


@app.get("/api/csv/download")
async def download_csv():
    """Stream the most recently completed (or active) CSV log file."""
    from fastapi.responses import FileResponse as FR
    # Prefer the active file; fall back to most recent in logs/
    if state.csv_path and state.csv_path.exists():
        return FR(str(state.csv_path), media_type="text/csv",
                  filename=state.csv_path.name)
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    csvs = sorted(LOGS_DIR.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not csvs:
        return JSONResponse({"ok": False, "msg": "No CSV files found"}, status_code=404)
    return FR(str(csvs[0]), media_type="text/csv", filename=csvs[0].name)


@app.get("/api/state")
async def get_ui_state() -> JSONResponse:
    return JSONResponse(state.saved_state)


@app.post("/api/state")
async def save_ui_state(payload: dict) -> JSONResponse:
    state.save_state({str(k): str(v) for k, v in payload.items()})
    return JSONResponse({"ok": True})


# ---------------------------------------------------------------------------
# Preset endpoints
# ---------------------------------------------------------------------------
@app.get("/api/presets")
async def list_presets() -> JSONResponse:
    return JSONResponse({"presets": list(_load_presets().keys())})


@app.post("/api/preset/save")
async def save_preset(payload: dict) -> JSONResponse:
    name = str(payload.get("name", "")).strip()
    if not name:
        return JSONResponse({"ok": False, "msg": "Name required"})
    data = _load_presets()
    data[name] = {k: v for k, v in payload.items() if k != "name"}
    _save_presets(data)
    return JSONResponse({"ok": True, "name": name})


@app.post("/api/preset/load")
async def load_preset(payload: dict) -> JSONResponse:
    name = str(payload.get("name", "")).strip()
    data = _load_presets()
    if name not in data:
        return JSONResponse({"ok": False, "msg": "Not found"}, status_code=404)
    return JSONResponse({"ok": True, "preset": data[name]})


@app.delete("/api/preset/{name}")
async def delete_preset(name: str) -> JSONResponse:
    data = _load_presets()
    if name not in data:
        return JSONResponse({"ok": False, "msg": "Not found"}, status_code=404)
    del data[name]
    _save_presets(data)
    return JSONResponse({"ok": True})


# ---------------------------------------------------------------------------
# CSV metadata / column selection / event markers
# ---------------------------------------------------------------------------
@app.post("/api/csv/meta")
async def set_csv_meta(payload: dict) -> JSONResponse:
    state.pending_run_id = str(payload.get("run_id", ""))
    state.pending_heatsink_id = str(payload.get("heatsink_id", ""))
    return JSONResponse({"ok": True})


@app.post("/api/csv/columns")
async def set_csv_columns(payload: dict) -> JSONResponse:
    cols = payload.get("columns")
    if not isinstance(cols, list) or not cols:
        state.csv_columns = None
        return JSONResponse({"ok": True, "columns": state.CSV_FIELDNAMES})
    valid = [c for c in cols if c in state.CSV_FIELDNAMES]
    if not valid:
        return JSONResponse({"ok": False, "msg": "No valid columns"})
    state.csv_columns = valid
    return JSONResponse({"ok": True, "columns": valid})


@app.post("/api/csv/event")
async def csv_event(payload: dict) -> JSONResponse:
    label = str(payload.get("label", "marker"))[:64]
    ts_iso = dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="milliseconds")
    ts_float = time.time()
    if state.csv_logging and state.csv_writer and state.last_telemetry:
        t = state.last_telemetry
        elapsed = max(0.0, time.monotonic() - state.csv_start_monotonic)
        cols = state.csv_columns or state.CSV_FIELDNAMES
        row = {c: "" for c in cols}
        row.update({
            "timestamp_iso": ts_iso,
            "elapsed_s": f"{elapsed:.3f}",
            "raw_temp_c": t.get("raw_temp", ""),
            "temp_filtered_c": t.get("temp", ""),
            "temp_smooth_c": t.get("smooth", ""),
            "event": label,
        })
        try:
            state.csv_writer.writerow(row)
            if state.csv_file:
                state.csv_file.flush()
        except OSError:
            pass
    await state.broadcast({"type": "marker", "label": label, "ts": ts_float})
    return JSONResponse({"ok": True, "label": label})


@app.get("/api/status")
async def get_status() -> JSONResponse:
    return JSONResponse({
        "connected": state.connected,
        "port": state.port,
        "baud": state.baud,
        "csv_logging": state.csv_logging,
        "csv_path": str(state.csv_path) if state.csv_path else "",
        "handshake_ok": state.handshake_ok,
    })


# ---------------------------------------------------------------------------
# WebSocket
# ---------------------------------------------------------------------------
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    async with state.ws_lock:
        state.websockets.append(websocket)

    # Send current status immediately on connect
    await websocket.send_text(json.dumps({
        "type": "status",
        "connected": state.connected,
        "port": state.port,
        "baud": state.baud,
        "csv_logging": state.csv_logging,
        "handshake_ok": state.handshake_ok,
    }))

    # Send last telemetry snapshot if available
    if state.last_telemetry:
        await websocket.send_text(json.dumps(state.last_telemetry))
    if state.last_cfg:
        await websocket.send_text(json.dumps(state.last_cfg))

    try:
        while True:
            # Keep connection alive; client messages not expected but drain anyway
            try:
                await asyncio.wait_for(websocket.receive_text(), timeout=30)
            except asyncio.TimeoutError:
                # Send ping to keep alive
                await websocket.send_text(json.dumps({"type": "ping"}))
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        async with state.ws_lock:
            if websocket in state.websockets:
                state.websockets.remove(websocket)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import webbrowser

    print("HeatsinkLab PID Web GUI")
    print("Open http://localhost:8765 in your browser")
    webbrowser.open("http://localhost:8765")
    uvicorn.run(app, host="0.0.0.0", port=8765, log_level="info")
