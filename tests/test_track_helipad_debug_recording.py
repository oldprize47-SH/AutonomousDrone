import importlib.util
import math
import sys
import types
from pathlib import Path


def load_track_module():
    root = Path(__file__).resolve().parents[1]
    deploy = root / "vision"
    sys.path.insert(0, str(deploy))

    cv2 = types.ModuleType("cv2")
    cv2.FONT_HERSHEY_SIMPLEX = 0
    cv2.LINE_AA = 16
    cv2.VideoWriter_fourcc = lambda *args: 0
    cv2.VideoWriter = lambda *args, **kwargs: None
    cv2.rectangle = lambda *args, **kwargs: None
    cv2.circle = lambda *args, **kwargs: None
    cv2.putText = lambda *args, **kwargs: None
    sys.modules.setdefault("cv2", cv2)

    np = types.ModuleType("numpy")
    np.uint8 = object
    np.float32 = object
    np.zeros = lambda *args, **kwargs: None
    np.array = lambda value, *args, **kwargs: value
    np.asanyarray = lambda value, *args, **kwargs: value
    sys.modules.setdefault("numpy", np)

    ncnn = types.ModuleType("ncnn")
    ncnn.Net = object
    ncnn.Mat = types.SimpleNamespace(PixelType=types.SimpleNamespace(PIXEL_RGB=0))
    sys.modules.setdefault("ncnn", ncnn)

    rs = types.ModuleType("pyrealsense2")
    sys.modules.setdefault("pyrealsense2", rs)

    inference = types.ModuleType("inference")
    inference.generate_anchors = lambda: []
    inference.decode_boxes = lambda *args, **kwargs: []
    inference.nms = lambda *args, **kwargs: []
    inference.INPUT_SIZE = 512
    sys.modules.setdefault("inference", inference)

    spec = importlib.util.spec_from_file_location("track_helipad", deploy / "track_helipad.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bbox_debug_metrics_reports_geometry_and_jump():
    m = load_track_module()
    out = m.bbox_debug_metrics((10, 20, 30, 40), (13, 24, 30, 40), 640)

    assert out["bbox_x"] == 13
    assert out["bbox_y"] == 24
    assert out["bbox_w"] == 30
    assert out["bbox_h"] == 40
    assert out["bbox_cx"] == 28.0
    assert out["bbox_cy"] == 44.0
    assert out["bbox_area"] == 1200
    assert math.isclose(out["bbox_jump_px"], 5.0)
    assert math.isclose(out["bbox_jump_frac"], 5.0 / 640)


def test_bbox_debug_metrics_handles_missing_bbox():
    m = load_track_module()
    out = m.bbox_debug_metrics((10, 20, 30, 40), None, 640)

    for key in ("bbox_x", "bbox_y", "bbox_w", "bbox_h", "bbox_cx", "bbox_cy", "bbox_area"):
        assert out[key] is None
    assert out["bbox_jump_px"] is None
    assert out["bbox_jump_frac"] is None


def test_debug_event_flags_detects_lost_jump_det_miss_and_state_change():
    m = load_track_module()
    flags = m.debug_event_flags(
        prev_state="CONFIRMED",
        state="CANDIDATE",
        prev_src="trk",
        src="trk_pred",
        det_miss_latch=True,
        bbox_metrics={"bbox_jump_frac": 0.20},
        jump_threshold_frac=0.15,
    )

    assert flags == {
        "event_lost": 0,
        "event_center_jump": 1,
        "event_det_miss": 1,
        "event_state_change": 1,
    }


def test_debug_event_flags_detects_transition_to_lost():
    m = load_track_module()
    flags = m.debug_event_flags(
        prev_state="CANDIDATE",
        state="LOST",
        prev_src="det",
        src="lost",
        det_miss_latch=False,
        bbox_metrics={"bbox_jump_frac": None},
        jump_threshold_frac=0.15,
    )

    assert flags["event_lost"] == 1
    assert flags["event_det_miss"] == 0


def test_video_recorder_rate_limits_and_writes_without_main_loop_wait(tmp_path):
    m = load_track_module()
    writers = []

    class Frame:
        shape = (480, 640, 3)

    class FakeWriter:
        def __init__(self):
            self.writes = 0
            self.released = False

        def isOpened(self):
            return True

        def write(self, _frame):
            self.writes += 1

        def release(self):
            self.released = True

    def fake_writer(*_args, **_kwargs):
        writer = FakeWriter()
        writers.append(writer)
        return writer

    m.cv2.VideoWriter = fake_writer
    rec = m.VideoRecorder(str(tmp_path / "debug.avi"), fps=10, queue_size=1)
    try:
        assert rec.submit(0, 0.0, Frame()) is True
        assert rec.submit(1, 0.05, Frame()) is False  # rate-limited, not a queue drop
        assert rec.submitted == 1
        assert rec.dropped == 0
    finally:
        rec.close()

    assert rec.error is None
    assert rec.written == 1
    assert writers and writers[0].released


def test_landing_gate_blocks_tracker_only_prediction_from_go():
    m = load_track_module()
    gate = m.LandingGate(
        diag=100.0,
        K=3,
        need_valid=2,
        det_need=1,
        ttl_trk=1,
        jitter_frac=0.5,
        area_ratio_max=2.0,
        recent_det_max=3,
    )
    bbox = (10, 10, 20, 20)

    state, go, _nv, _nd = gate.update(bbox, "det", 0.9, det_age=0)
    assert state == "CANDIDATE"
    assert go is False

    state, go, _nv, _nd = gate.update(bbox, "det", 0.9, det_age=0)
    assert state == "CONFIRMED"
    assert go is True

    state, go, _nv, _nd = gate.update(bbox, "trk_pred", 0.9, det_age=1)
    assert state == "CONFIRMED"
    assert go is False


def test_compact_packet_valid_requires_fresh_confirmed_depth_measurement():
    m = load_track_module()
    args = types.SimpleNamespace(
        cam_tilt_forward_deg=0.0,
        cam_offset_forward_m=0.1,
        cam_offset_right_m=0.045,
        cam_offset_down_m=0.09,
    )
    meas = {
        "range_m": 0.8,
        "camera_depth_m": 0.8,
        "drone_ground_distance_m": 0.75,
        "helipad_distance_m": 0.82,
        "helipad_x_m": 0.1,
        "helipad_y_m": -0.2,
        "helipad_xy_distance_m": 0.224,
        "center_distance_m": 0.224,
        "measurement_model": "center_pixel_level",
        "center_calibrated": True,
    }
    packet = m.make_vision_packet(
        1, 1.0, "CONFIRMED", True, "det", 0.9, (10, 10, 20, 20),
        1.0, -1.0, meas, 12, 3, 30.0, 20.0, 0, 100.0, None, args,
    )
    assert packet["valid"] is True
    assert packet["stm"]["valid"] is True
    assert packet["stm"]["frame_id"] == "body_frd_level"

    stale = m.make_vision_packet(
        2, 1.1, "CONFIRMED", False, "trk_pred", 0.9, (10, 10, 20, 20),
        1.0, -1.0, meas, 12, 3, 30.0, 20.0, 1, 100.0, None, args,
    )
    assert stale["valid"] is False
    assert stale["stm"]["valid"] is False

    tracker_without_go = m.make_vision_packet(
        3, 1.2, "CONFIRMED", False, "trk", 0.9, (10, 10, 20, 20),
        1.0, -1.0, meas, 12, 3, 30.0, 20.0, 2, 100.0, None, args,
    )
    assert tracker_without_go["landing_go_advisory"] is False
    assert tracker_without_go["valid"] is False
    assert tracker_without_go["stm"]["valid"] is False
