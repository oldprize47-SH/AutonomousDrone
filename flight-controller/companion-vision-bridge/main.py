#
# AVC UNO Q - Helipad vision bridge (container side)
#
# This is the AVC app entrypoint. It starts /app/helipad/track_helipad.py in the
# same Docker container, reads its compact latest telemetry JSON, and forwards it
# to the MCU through Bridge.notify("set_helipad_target", valid, x, y, xy, gnd).
#
# The original helipad source tree remains preserved at:
#   /home/arduino/helipad/deploy_unoq
# Runtime copy visible inside the container:
#   /app/helipad
#

from arduino.app_utils import *
import json
import os
import shlex
import subprocess
import sys
import threading
import time

HELIPAD_DIR = os.environ.get("AVC_HELIPAD_DIR", "/app/helipad")
TRACKER = os.environ.get("AVC_HELIPAD_TRACKER", os.path.join(HELIPAD_DIR, "track_helipad.py"))
TELEM_PATH = os.environ.get("AVC_HELIPAD_TELEMETRY", os.path.join(HELIPAD_DIR, "helipad_latest.json"))
POLL_HZ = float(os.environ.get("AVC_HELIPAD_POLL_HZ", "50"))
STALE_S = float(os.environ.get("AVC_HELIPAD_STALE_S", "0.50"))
PRINT_HZ = float(os.environ.get("AVC_HELIPAD_PRINT_HZ", "2"))
RESTART_DELAY_S = float(os.environ.get("AVC_HELIPAD_RESTART_DELAY_S", "5"))
EXTRA_ARGS = os.environ.get("AVC_HELIPAD_ARGS", "")

_proc = None
_next_restart = 0.0
_last_frame = None
_last_print = 0.0
_last_status = "init"
_last_packet_t = 0.0
_last_sent_valid = None
_last_valid_reported = None
_stale_invalid_sent = False


def _build_cmd():
    cmd = [
        sys.executable,
        TRACKER,
        "--headless",
        "--no-log",
        "--telemetry", "latest",
        "--telemetry-format", "stm",
        "--telemetry-path", TELEM_PATH,
    ]
    if EXTRA_ARGS.strip():
        cmd.extend(shlex.split(EXTRA_ARGS))
    return cmd


def _reader_thread(proc):
    try:
        for line in proc.stdout:
            print("[HELIPAD] " + line.rstrip(), flush=True)
    except Exception as exc:
        print(f"[AVC] tracker stdout reader stopped: {type(exc).__name__}: {exc}", flush=True)


def _start_tracker(force=False):
    global _proc, _next_restart, _last_status
    now = time.monotonic()
    if _proc is not None and _proc.poll() is None:
        return
    if not force and now < _next_restart:
        return
    if _proc is not None:
        rc = _proc.poll()
        print(f"[AVC] helipad tracker exited rc={rc}; restarting after {RESTART_DELAY_S:.1f}s", flush=True)
    if not os.path.isfile(TRACKER):
        _last_status = "tracker_missing"
        print(f"[AVC] helipad tracker missing: {TRACKER}", flush=True)
        _next_restart = now + RESTART_DELAY_S
        return
    os.makedirs(os.path.dirname(TELEM_PATH), exist_ok=True)
    cmd = _build_cmd()
    print("[AVC] starting helipad tracker: " + " ".join(shlex.quote(x) for x in cmd), flush=True)
    try:
        _proc = subprocess.Popen(
            cmd,
            cwd=HELIPAD_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=os.environ.copy(),
        )
        threading.Thread(target=_reader_thread, args=(_proc,), daemon=True).start()
        _last_status = "tracker_started"
    except Exception as exc:
        _last_status = "tracker_start_failed"
        print(f"[AVC] helipad tracker start failed: {type(exc).__name__}: {exc}", flush=True)
        _next_restart = now + RESTART_DELAY_S


def _read_latest_packet():
    try:
        st = os.stat(TELEM_PATH)
        if time.time() - st.st_mtime > STALE_S:
            return None
        with open(TELEM_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except Exception as exc:
        print(f"[AVC] telemetry read failed: {type(exc).__name__}: {exc}", flush=True)
        return None


def _num(value, default=0.0):
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _notify_target(valid, x_m, y_m, xy_m, gnd_m, reason=""):
    global _last_sent_valid, _last_valid_reported
    Bridge.notify("set_helipad_target", int(valid), float(x_m), float(y_m), float(xy_m), float(gnd_m))
    _last_sent_valid = int(valid)
    if valid and _last_valid_reported != 1:
        print(f"[AVC] first valid helipad target: x={x_m:.3f} y={y_m:.3f} xy={xy_m:.3f} gnd={gnd_m:.3f}", flush=True)
    if (not valid) and _last_valid_reported == 1:
        print(f"[AVC] helipad target invalidated{': ' + reason if reason else ''}", flush=True)
    _last_valid_reported = int(valid)


def _send_stale_invalid(reason):
    global _stale_invalid_sent
    if _stale_invalid_sent or _last_sent_valid != 1:
        return False
    try:
        # MCU side gates steering on valid==1; zeros are a benign invalid payload.
        _notify_target(0, 0.0, 0.0, 0.0, 0.0, reason=reason)
        _stale_invalid_sent = True
        return True
    except Exception as exc:
        print(f"[AVC] Bridge.notify stale invalid failed: {type(exc).__name__}: {exc}", flush=True)
        return False


def _forward_packet(pkt):
    global _last_frame, _last_packet_t, _stale_invalid_sent
    if not pkt:
        return False
    frame = pkt.get("frame")
    if frame == _last_frame:
        return False
    _last_frame = frame
    _last_packet_t = time.monotonic()
    _stale_invalid_sent = False

    valid = 1 if bool(pkt.get("valid")) else 0
    x_m = _num(pkt.get("helipad_x_m"))
    y_m = _num(pkt.get("helipad_y_m"))
    xy_m = _num(pkt.get("helipad_xy_distance_m"))
    gnd_m = _num(pkt.get("drone_ground_distance_m"))

    try:
        _notify_target(valid, x_m, y_m, xy_m, gnd_m)
        return True
    except Exception as exc:
        print(f"[AVC] Bridge.notify set_helipad_target failed: {type(exc).__name__}: {exc}", flush=True)
        return False


def setup():
    print("[AVC] helipad vision bridge starting")
    print(f"[AVC] tracker={TRACKER}")
    print(f"[AVC] telemetry={TELEM_PATH} poll={POLL_HZ:.0f}Hz stale={STALE_S:.2f}s")
    _start_tracker(force=True)


def loop():
    global _last_print, _last_status
    t0 = time.perf_counter()

    _start_tracker()
    pkt = _read_latest_packet()
    if pkt is None:
        _send_stale_invalid("telemetry stale/missing")
    else:
        _forward_packet(pkt)

    now = time.perf_counter()
    if PRINT_HZ > 0 and (now - _last_print) >= (1.0 / PRINT_HZ):
        _last_print = now
        proc_state = "none" if _proc is None else ("running" if _proc.poll() is None else f"exit={_proc.poll()}")
        pkt_age = None if _last_packet_t <= 0 else (time.monotonic() - _last_packet_t)
        frame = None if pkt is None else pkt.get("frame")
        valid = None if pkt is None else pkt.get("valid")
        x = None if pkt is None else pkt.get("helipad_x_m")
        y = None if pkt is None else pkt.get("helipad_y_m")
        g = None if pkt is None else pkt.get("drone_ground_distance_m")
        age_txt = "None" if pkt_age is None else f"{pkt_age:.2f}s"
        print(f"[AVC] tracker={proc_state} frame={frame} valid={valid} "
              f"x={x} y={y} gnd={g} sent={_last_sent_valid} age={age_txt}", flush=True)

    sleep_s = (1.0 / max(POLL_HZ, 1.0)) - (time.perf_counter() - t0)
    if sleep_s > 0:
        time.sleep(sleep_s)


setup()
App.run(user_loop=loop)
