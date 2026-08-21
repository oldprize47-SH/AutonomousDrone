import importlib.util
import json
import os
import sys
import time
import types


def load_bridge(tmp_path, monkeypatch):
    tracker = tmp_path / "missing_tracker.py"
    telemetry = tmp_path / "latest.json"
    monkeypatch.setenv("AVC_HELIPAD_DIR", str(tmp_path))
    monkeypatch.setenv("AVC_HELIPAD_TRACKER", str(tracker))
    monkeypatch.setenv("AVC_HELIPAD_TELEMETRY", str(telemetry))

    class FakeBridge:
        calls = []

        @classmethod
        def notify(cls, *args):
            cls.calls.append(args)

    class FakeApp:
        @staticmethod
        def run(user_loop):
            FakeApp.user_loop = user_loop

    app_utils = types.ModuleType("arduino.app_utils")
    app_utils.Bridge = FakeBridge
    app_utils.App = FakeApp
    app_utils.__all__ = ["Bridge", "App"]
    arduino = types.ModuleType("arduino")
    arduino.__path__ = []
    monkeypatch.setitem(sys.modules, "arduino", arduino)
    monkeypatch.setitem(sys.modules, "arduino.app_utils", app_utils)

    path = os.path.join(os.path.dirname(__file__), "..", "bridge", "main.py")
    spec = importlib.util.spec_from_file_location("portfolio_bridge_main", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, FakeBridge, telemetry


def test_build_command_is_headless_compact_latest(tmp_path, monkeypatch):
    module, _bridge, _telemetry = load_bridge(tmp_path, monkeypatch)
    cmd = module._build_cmd()
    assert "--headless" in cmd
    assert "--no-log" in cmd
    assert cmd[cmd.index("--telemetry") + 1] == "latest"
    assert cmd[cmd.index("--telemetry-format") + 1] == "stm"


def test_forward_packet_notifies_exact_target_contract(tmp_path, monkeypatch):
    module, bridge, _telemetry = load_bridge(tmp_path, monkeypatch)
    packet = {
        "frame": 7,
        "valid": True,
        "helipad_x_m": 0.1,
        "helipad_y_m": -0.2,
        "helipad_xy_distance_m": 0.224,
        "drone_ground_distance_m": 0.8,
    }
    assert module._forward_packet(packet) is True
    assert bridge.calls[-1] == ("set_helipad_target", 1, 0.1, -0.2, 0.224, 0.8)


def test_stale_valid_target_is_invalidated_once(tmp_path, monkeypatch):
    module, bridge, _telemetry = load_bridge(tmp_path, monkeypatch)
    module._last_sent_valid = 1
    assert module._send_stale_invalid("stale") is True
    assert module._send_stale_invalid("stale again") is False
    assert bridge.calls == [("set_helipad_target", 0, 0.0, 0.0, 0.0, 0.0)]


def test_latest_packet_rejects_stale_file(tmp_path, monkeypatch):
    module, _bridge, telemetry = load_bridge(tmp_path, monkeypatch)
    telemetry.write_text(json.dumps({"frame": 1, "valid": True}), encoding="utf-8")
    module.STALE_S = 0.1
    old = time.time() - 2.0
    os.utime(telemetry, (old, old))
    assert module._read_latest_packet() is None
