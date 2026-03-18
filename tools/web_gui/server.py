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

        self.websockets: list[WebSocket] = []
        self.ws_lock = asyncio.Lock()

        self.loop: asyncio.AbstractEventLoop | None = None

        # Latest telemetry snapshot
        self.last_telemetry: dict[str, Any] = {}
        self.last_cfg: dict[str, Any] = {}

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
        ok, msg = self._write_serial(command)
        self.broadcast_sync({"type": "log", "text": f"> {command}"})
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
            }
            self.last_telemetry = telemetry
            self.broadcast_sync(telemetry)

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
        "run_state", "fan_inverted",
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
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.CSV_FIELDNAMES)
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


@app.get("/api/state")
async def get_ui_state() -> JSONResponse:
    return JSONResponse(state.saved_state)


@app.post("/api/state")
async def save_ui_state(payload: dict) -> JSONResponse:
    state.save_state({str(k): str(v) for k, v in payload.items()})
    return JSONResponse({"ok": True})


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
