import importlib.util
import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_capture_module(monkeypatch):
    fake_rs = types.SimpleNamespace()
    monkeypatch.setitem(sys.modules, "numpy", types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "cv2", types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, "pyrealsense2", fake_rs)
    path = ROOT / "tools" / "data-preparation" / "capture_frames.py"
    spec = importlib.util.spec_from_file_location("portfolio_capture_frames", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_select_depth_frame_respects_runtime_flag(monkeypatch):
    module = load_capture_module(monkeypatch)

    class Frames:
        def __init__(self):
            self.calls = 0

        def get_depth_frame(self):
            self.calls += 1
            return "depth-frame"

    frames = Frames()
    assert module.select_depth_frame(frames, False) is None
    assert frames.calls == 0
    assert module.select_depth_frame(frames, True) == "depth-frame"
    assert frames.calls == 1
