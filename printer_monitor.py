"""Ender-3 remote monitor: webcam MJPEG stream + pause/stop over USB serial.

Run:  uv run printer_monitor.py
Config (env vars): PM_SERIAL, PM_BAUD, PM_CAMERA, PM_HOST, PM_PORT, PM_FPS, PM_TOKEN
"""

import collections
import hmac
import os
import re
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import cv2
import serial
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, Response, StreamingResponse

SERIAL_PORT = os.environ.get("PM_SERIAL", "COM7")
BAUD = int(os.environ.get("PM_BAUD", "115200"))
CAMERA_INDEX = int(os.environ.get("PM_CAMERA", "1"))
HOST = os.environ.get("PM_HOST", "127.0.0.1")
HTTP_PORT = int(os.environ.get("PM_PORT", "8090"))
FPS = float(os.environ.get("PM_FPS", "5"))
TOKEN = os.environ.get("PM_TOKEN", "")
POLL_SECONDS = 5

HERE = Path(__file__).parent

# Marlin replies, e.g. "ok T:21.5 /0.0 B:21.0 /0.0 @:0 B@:0" and "SD printing byte 1234/56789"
TEMP_RE = re.compile(r"T:([\d.]+)\s*/([\d.]+).*?B:([\d.]+)\s*/([\d.]+)")
SD_RE = re.compile(r"SD printing byte (\d+)/(\d+)")

# Button name -> G-code lines. EMERGENCY_PARSER is enabled in this firmware,
# so M112 is acted on immediately even while the command queue is full.
ACTIONS = {
    "pause": ["M25"],  # pause SD print (hotend stays hot)
    "resume": ["M24"],
    "stop": ["M524", "M104 S0", "M140 S0", "M107"],  # abort SD print, heaters + fan off
    "emergency": ["M112"],  # kill; needs a reset/power cycle afterwards
}


class Printer:
    """Holds COM port open for the life of the process.

    Opening the port can reset the Ender-3 board, which would kill a running
    SD print, so a dropped connection is never reopened automatically — only
    by an explicit /api/reconnect.
    """

    def __init__(self):
        self.ser = None
        self.write_lock = threading.Lock()
        self.log = collections.deque(maxlen=200)
        self.generation = 0
        self.state = {
            "connected": False,
            "error": None,
            "hotend": None,
            "hotend_target": None,
            "bed": None,
            "bed_target": None,
            "sd_printing": None,
            "progress": None,
            "updated": None,
        }

    def _note(self, line):
        self.log.append(f"{time.strftime('%H:%M:%S')} {line}")

    def connect(self):
        self.disconnect()
        ser = serial.Serial()
        ser.port = SERIAL_PORT
        ser.baudrate = BAUD
        ser.timeout = 1
        ser.dtr = False  # best effort: keep the board from resetting on open
        ser.rts = False
        try:
            ser.open()
        except serial.SerialException as e:
            self.state.update(connected=False, error=str(e))
            self._note(f"!! open failed: {e}")
            return False
        self.ser = ser
        self.generation += 1
        self.state.update(connected=True, error=None)
        self._note(f"-- connected {SERIAL_PORT} @ {BAUD}")
        threading.Thread(target=self._reader, args=(self.generation,), daemon=True).start()
        threading.Thread(target=self._poller, args=(self.generation,), daemon=True).start()
        return True

    def disconnect(self, error=None):
        self.generation += 1
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.state.update(connected=False, error=error)

    def send(self, line):
        with self.write_lock:
            if self.ser is None:
                raise RuntimeError("printer not connected")
            try:
                self.ser.write((line + "\n").encode("ascii"))
            except serial.SerialException as e:
                self._note(f"!! write failed: {e}")
                self.disconnect(error=str(e))
                raise RuntimeError(str(e)) from e
        if line not in ("M105", "M27"):
            self._note(f">> {line}")

    def _reader(self, gen):
        while gen == self.generation:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, AttributeError, TypeError) as e:
                if gen == self.generation:
                    self._note(f"!! read failed: {e}")
                    self.disconnect(error=str(e))
                return
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                self._handle(line)

    def _handle(self, line):
        m = TEMP_RE.search(line)
        if m:
            h, ht, b, bt = map(float, m.groups())
            self.state.update(hotend=h, hotend_target=ht, bed=b, bed_target=bt, updated=time.time())
            return
        m = SD_RE.search(line)
        if m:
            done, total = map(int, m.groups())
            self.state.update(sd_printing=True, progress=round(100 * done / total, 1) if total else None)
            return
        if "Not SD printing" in line:
            self.state.update(sd_printing=False, progress=None)
            return
        if line == "ok" or line.startswith("echo:busy"):
            return
        self._note(f"<< {line}")

    def _poller(self, gen):
        time.sleep(2)  # give the board a moment in case the open did reset it
        while gen == self.generation:
            try:
                self.send("M105")
                self.send("M27")
            except RuntimeError:
                return
            time.sleep(POLL_SECONDS)


class Camera:
    """Grabs frames continuously and keeps the newest one as JPEG."""

    def __init__(self):
        self.jpeg = None
        self.ok = False
        self.cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            if not cap.isOpened():
                self.ok = False
                time.sleep(5)
                continue
            next_at = 0.0
            # grab() every frame so the driver buffer never goes stale; only
            # decode + encode at FPS.
            while cap.grab():
                now = time.time()
                if now < next_at:
                    continue
                next_at = now + 1 / FPS
                ok, frame = cap.retrieve()
                if not ok:
                    break
                cv2.putText(frame, time.strftime("%Y-%m-%d %H:%M:%S"), (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with self.cond:
                        self.jpeg = buf.tobytes()
                        self.ok = True
                        self.cond.notify_all()
            cap.release()
            self.ok = False
            time.sleep(2)

    def frames(self):
        last = None
        while True:
            with self.cond:
                self.cond.wait(timeout=5)
                frame = self.jpeg
            if frame is None or frame is last:
                continue
            last = frame
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")


printer = Printer()
camera = None


@asynccontextmanager
async def lifespan(app):
    global camera
    camera = Camera()
    printer.connect()
    yield
    printer.disconnect()


app = FastAPI(lifespan=lifespan)


@app.middleware("http")
async def check_token(request: Request, call_next):
    if not TOKEN:
        return await call_next(request)
    from_query = request.query_params.get("token")
    given = from_query or request.cookies.get("pm_token") or ""
    if not hmac.compare_digest(given.encode(), TOKEN.encode()):
        return JSONResponse({"error": "unauthorized"}, status_code=401)
    if from_query and request.method == "GET":
        # Remember the token in a cookie and drop it from the URL.
        resp = RedirectResponse(str(request.url.remove_query_params("token")), status_code=303)
        resp.set_cookie("pm_token", TOKEN, max_age=365 * 86400, httponly=True, samesite="strict")
        return resp
    return await call_next(request)


@app.get("/")
def index():
    return FileResponse(HERE / "index.html")


@app.get("/stream")
def stream():
    return StreamingResponse(camera.frames(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/snapshot.jpg")
def snapshot():
    if camera.jpeg is None:
        raise HTTPException(503, "no camera frame yet")
    return Response(camera.jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/status")
def status():
    return {**printer.state, "camera": camera.ok, "log": list(printer.log)[-40:], "now": time.time()}


@app.post("/api/action/{name}")
def action(name: str):
    if name not in ACTIONS:
        raise HTTPException(404, f"unknown action {name}")
    try:
        for line in ACTIONS[name]:
            printer.send(line)
    except RuntimeError as e:
        raise HTTPException(503, str(e))
    return {"sent": ACTIONS[name]}


@app.post("/api/reconnect")
def reconnect():
    if not printer.connect():
        raise HTTPException(503, printer.state["error"])
    return {"connected": True}


if __name__ == "__main__":
    uvicorn.run(app, host=HOST, port=HTTP_PORT)
