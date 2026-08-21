"""
드론 착륙 헬리패드: RealSense(color+depth) + detection(SSDLite 512) + LK tracking
                    + 관측각(body frame) + depth 거리 + 착륙 GO 안전 상태머신.

정책(3-model 협업): "검출은 빠르게, 추적은 부드럽게, 착륙 GO는 매우 늦게."
- 카메라: pyrealsense2 pipeline (color BGR + depth Z16, align to color).
- detection: SSDLite 512 NCNN, conf 0.5(precision 우선). 매 N=5프레임 + 조건부 재검출.
- tracking: LK optical flow. 재검출 실패 예측(trk_pred)은 TTL 후 fail-closed, GO 금지.
- 관측각: 검출 중심 픽셀 → body frame az/el (스트랩다운, intrinsic).
- 거리: bbox 영역 depth median (중근거리 정밀). depth 무효(원거리)면 None → bbox/LiDAR 보조는 상위에서.
- 착륙 GO: CONFIRMED + detector 재확인 + temporal + 중심/면적 안정 시만.

사용:
  python3 track_helipad.py                              # 기본: MP 검출 + headless + 30fps + run.csv
  python3 track_helipad.py --calibrate-center           # 1회: 드론 중심 픽셀 보정 저장
  DISPLAY=:0 python3 track_helipad.py --show            # TV 화면
  python3 track_helipad.py --no-log                     # CSV 로그 없이 실행
  python3 track_helipad.py --record debug.avi            # 10fps overlay 영상 + 30fps CSV 로그
"""
import os, sys, time, argparse, csv, json, math, queue, gc, ctypes, threading
import multiprocessing as mp
from collections import deque
import numpy as np
import cv2
import ncnn
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inference import generate_anchors, decode_boxes, nms, INPUT_SIZE

# camera_intrinsics.txt (RealSense D435 color, 640x480)
FX, FY, CX, CY = 614.17, 614.55, 323.33, 234.98

# Body frame convention for exported landing measurements: FRD
#   +X forward, +Y right, +Z down; origin = drone control reference point.
# Defaults match the current measured RGB-lens mount after straight-down remount.
DEFAULT_CAM_OFFSET_FORWARD_M = 0.10
DEFAULT_CAM_OFFSET_RIGHT_M = 0.045
DEFAULT_CAM_OFFSET_DOWN_M = 0.09
DEFAULT_CAM_TILT_FORWARD_DEG = 0.0
DEFAULT_CENTER_CALIB_FILE = "helipad_center_calib.json"



class Detector:
    def __init__(self, param, binf, threads=3):
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.use_fp16_packed = True
        self.net.opt.use_fp16_storage = True
        self.net.opt.use_fp16_arithmetic = True
        self.net.opt.num_threads = threads
        self.net.load_param(param)
        self.net.load_model(binf)
        self.anchors = generate_anchors()

    def detect(self, frame, conf=0.5):
        h, w = frame.shape[:2]
        inp = cv2.cvtColor(cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE)), cv2.COLOR_BGR2RGB)
        mat = ncnn.Mat.from_pixels(inp, ncnn.Mat.PixelType.PIXEL_RGB, INPUT_SIZE, INPUT_SIZE)
        mat.substract_mean_normalize([0.485*255, 0.456*255, 0.406*255],
                                     [1/(0.229*255), 1/(0.224*255), 1/(0.225*255)])
        ex = self.net.create_extractor()
        ex.input("input", mat)
        _, mb = ex.extract("boxes")
        _, ms = ex.extract("scores")
        bp = np.array(mb); sp = np.array(ms)
        ld = sp[:, 1] - sp[:, 0]
        mask = ld > np.log(conf / (1.0 - conf))
        if mask.sum() == 0:
            return None
        fs = 1.0 / (1.0 + np.exp(-ld[mask]))
        dec = decode_boxes(bp[mask], self.anchors[mask])
        keep = nms(dec, fs, max_dets=1)
        if not keep:
            return None
        k = keep[0]; b = dec[k]
        return (int(b[0]*w), int(b[1]*h), int((b[2]-b[0])*w), int((b[3]-b[1])*h)), float(fs[k])


class LKTracker:
    def __init__(self):
        self.prev = None; self.pts = None; self.bbox = None

    def init(self, frame, bbox):
        x, y, w, h = bbox
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        x0, y0 = max(0, x), max(0, y); x1, y1 = min(g.shape[1], x+w), min(g.shape[0], y+h)
        if x1 <= x0 or y1 <= y0:
            return False
        roi = g[y0:y1, x0:x1]                       # bbox 영역만 스캔(전체 640x480 대신) → goodFeaturesToTrack 부하 대폭↓
        pts = cv2.goodFeaturesToTrack(roi, 60, 0.01, 4)
        if pts is not None:
            pts[:, :, 0] += x0; pts[:, :, 1] += y0  # ROI 좌표 → 전체 프레임 좌표로 복원
        self.pts = pts; self.prev = g; self.bbox = bbox
        return self.pts is not None and len(self.pts) >= 4

    def update(self, frame):
        if self.pts is None or self.prev is None:
            return False, self.bbox
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev, g, self.pts, None, winSize=(21, 21), maxLevel=3)
        if nxt is None:
            return False, self.bbox
        st = st.reshape(-1)
        gn = nxt.reshape(-1, 2)[st == 1]; go = self.pts.reshape(-1, 2)[st == 1]
        if len(gn) < 4:
            return False, self.bbox
        dx = float(np.median(gn[:, 0] - go[:, 0])); dy = float(np.median(gn[:, 1] - go[:, 1]))
        d_old = np.linalg.norm(go - go.mean(0), axis=1).mean() + 1e-6
        d_new = np.linalg.norm(gn - gn.mean(0), axis=1).mean()
        s = float(np.clip(d_new / d_old, 0.85, 1.18))
        x, y, w, h = self.bbox
        cx, cy = x + w/2 + dx, y + h/2 + dy; w, h = w*s, h*s
        self.bbox = (int(cx - w/2), int(cy - h/2), int(w), int(h))
        self.pts = gn.reshape(-1, 1, 2).astype(np.float32); self.prev = g
        return True, self.bbox


class NanoTracker:
    """NanoTrack(Siamese, OpenCV TrackerNano). LKTracker와 동일 인터페이스(init/update).
    onnx는 1회만 로드하고, 재검출 시 init()을 재호출해 template을 갱신(매 프레임 재생성 방지).
    TrackerNano.update()는 후보를 항상 내므로(ok=True 경향) getTrackingScore()로 별도 reject(min_score)."""
    def __init__(self, backbone, neckhead, min_score=0.0):
        if not (os.path.isfile(backbone) and os.path.isfile(neckhead)):
            raise FileNotFoundError(f"NanoTrack 모델 파일 없음: {backbone} / {neckhead}")
        if not hasattr(cv2, "TrackerNano_create"):
            raise RuntimeError("이 OpenCV 빌드에 TrackerNano가 없습니다(video/DNN 모듈 필요).")
        p = cv2.TrackerNano_Params()
        p.backbone = backbone
        p.neckhead = neckhead
        self.t = cv2.TrackerNano_create(p)
        self.min_score = min_score
        self.last_score = None
        self.bbox = None

    def init(self, frame, bbox):
        b = tuple(int(v) for v in bbox)
        if b[2] <= 0 or b[3] <= 0:
            return False
        self.t.init(frame, b)
        self.bbox = b
        return True

    def update(self, frame):
        ok, box = self.t.update(frame)
        self.last_score = float(self.t.getTrackingScore()) if ok else 0.0
        if ok and self.min_score > 0 and self.last_score < self.min_score:
            ok = False   # low-confidence drift → 실패 처리(재검출 유도)
        if ok:
            self.bbox = tuple(int(v) for v in box)
        return bool(ok), self.bbox


class LandingGate:
    """착륙 GO 안전 판정. SEARCH→CANDIDATE→CONFIRMED. tracker-only(trk_pred)는 TTL fail-closed + GO 금지."""
    def __init__(self, diag, K=15, need_valid=12, det_need=3, ttl_trk=8,
                 jitter_frac=0.05, area_ratio_max=1.5, recent_det_max=6):
        self.K = K; self.need = need_valid; self.det_need = det_need; self.ttl = ttl_trk
        self.jit = jitter_frac * diag; self.area_max = area_ratio_max; self.recent_det_max = recent_det_max
        self.recent = deque(maxlen=K); self.det_recent = deque(maxlen=K)
        self.state = "SEARCH"; self.prev = None; self.trk_only_age = 0; self.last_det_age = 999

    def _stable(self, b):
        if self.prev is None or b is None:
            return False if b is None else True
        c1 = (self.prev[0]+self.prev[2]/2, self.prev[1]+self.prev[3]/2)
        c2 = (b[0]+b[2]/2, b[1]+b[3]/2)
        d = ((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2) ** 0.5
        a1 = max(self.prev[2]*self.prev[3], 1); a2 = max(b[2]*b[3], 1)
        return d < self.jit and (max(a1, a2)/min(a1, a2)) < self.area_max

    def update(self, bbox, src, conf, det_age=0):
        if src == "trk_pred":
            self.trk_only_age += 1
            if self.trk_only_age > self.ttl:
                bbox = None; src = "lost"
        else:
            self.trk_only_age = 0
        is_det = (src == "det")
        # 비동기 detection은 결과가 det_age 프레임 지연 → GO 증거 나이를 실제 나이로(0 아님) 반영
        self.last_det_age = det_age if is_det else self.last_det_age + 1
        stable = self._stable(bbox)
        valid = (bbox is not None) and (src in ("det", "trk")) and ((conf >= 0.5) if is_det else True) and stable
        self.recent.append(1 if valid else 0)
        self.det_recent.append(1 if is_det else 0)
        self.prev = bbox
        nv = sum(self.recent); nd = sum(self.det_recent)
        if bbox is None and nv == 0 and self.state != "SEARCH":
            self.state = "LOST"
        if self.state in ("SEARCH", "LOST"):
            if is_det:
                self.state = "CANDIDATE"
        elif self.state == "CANDIDATE":
            if nv >= self.need and nd >= self.det_need:
                self.state = "CONFIRMED"
            elif nv == 0:
                self.state = "SEARCH"
        elif self.state == "CONFIRMED":
            if nv < self.need // 2:
                self.state = "CANDIDATE"
        go = (self.state == "CONFIRMED" and self.trk_only_age == 0
              and self.last_det_age <= self.recent_det_max
              and nv >= self.need and nd >= self.det_need and stable)
        return self.state, go, nv, nd


class RealSenseCam:
    """pyrealsense2 pipeline: color(BGR) + depth(Z16, align to color)."""
    def __init__(self, w=640, h=480, fps=30):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        self.scale = prof.get_device().first_depth_sensor().get_depth_scale()
        # align(전체 정렬 32ms) 제거 → bbox 1점만 역투영으로 거리. deproject용 intrin/extrin 저장
        cprof = prof.get_stream(rs.stream.color).as_video_stream_profile()
        dprof = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        self.cintr = cprof.get_intrinsics()
        self.dintr = dprof.get_intrinsics()
        self.c2d = cprof.get_extrinsics_to(dprof)
        self.d2c = dprof.get_extrinsics_to(cprof)
        for _ in range(10):
            self.pipe.wait_for_frames()

    def read(self):
        fs = self.pipe.wait_for_frames()    # align 없이(32ms 절감); 거리는 depth_distance_m에서 bbox 1점만 역투영
        c = fs.get_color_frame(); d = fs.get_depth_frame()
        if not c:
            return None, None
        return np.asanyarray(c.get_data()), d   # depth_frame(rs) 반환 — 1점 deproject용(asanyarray 안 함)

    def stop(self):
        self.pipe.stop()


def round_or_none(value, ndigits=3):
    if value is None:
        return None
    return round(float(value), ndigits)


def load_center_calib(path):
    """Load one-time drone-center pixel calibration.

    The calibrated pixel is measured by holding the drone origin over the helipad center.
    It absorbs camera mount offset and small assembly bias, so later horizontal offsets are
    relative to the drone control origin instead of the RGB lens center.
    """
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        u = float(data["center_u_px"])
        v = float(data["center_v_px"])
        if not math.isfinite(u) or not math.isfinite(v):
            return None
        return {
            "center_u_px": u,
            "center_v_px": v,
            "samples": int(data.get("samples", 0)),
            "file": path,
        }
    except FileNotFoundError:
        return None
    except Exception as e:
        print(f"[WARN] center calibration load failed: {path}: {e}", flush=True)
        return None


def save_center_calib(path, samples, args):
    us = [s["u"] for s in samples]
    vs = [s["v"] for s in samples]
    ranges = [s["range_m"] for s in samples if s.get("range_m") is not None]
    data = {
        "schema": 1,
        "created_unix_s": round(time.time(), 3),
        "center_u_px": round(sum(us) / len(us), 3),
        "center_v_px": round(sum(vs) / len(vs), 3),
        "samples": len(samples),
        "avg_range_m": round(sum(ranges) / len(ranges), 3) if ranges else None,
        "width": int(args.width),
        "height": int(args.height),
        "note": "Hold drone control origin over helipad center; this pixel is treated as body-origin ground intercept.",
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    return data


def target_depth_measurement(depth_frame, bbox, cam, tilt_forward_deg=DEFAULT_CAM_TILT_FORWARD_DEG,
                             cam_offset_forward_m=DEFAULT_CAM_OFFSET_FORWARD_M,
                             cam_offset_right_m=DEFAULT_CAM_OFFSET_RIGHT_M,
                             cam_offset_down_m=DEFAULT_CAM_OFFSET_DOWN_M,
                             center_calib=None):
    """Return target center depth plus level-body offset estimate.

    Camera frame (RealSense color/depth convention): +X right, +Y image-down, +Z optical-forward.
    Body frame exported here: +X forward, +Y right, +Z down. The estimate assumes the drone is
    level; STM roll/pitch should be applied by the landing controller for flight use.
    """
    u = float(bbox[0] + bbox[2] / 2.0)
    v = float(bbox[1] + bbox[3] / 2.0)
    out = {
        "center_u_px": round_or_none(u, 2),
        "center_v_px": round_or_none(v, 2),
        "depth_x_px": None,
        "depth_y_px": None,
        "range_m": None,
        "camera_depth_m": None,
        "cam_x_m": None,
        "cam_y_m": None,
        "cam_z_m": None,
        "helipad_x_m": None,
        "helipad_y_m": None,
        "helipad_xy_distance_m": None,
        "helipad_distance_m": None,
        "drone_ground_distance_m": None,
        "body_forward_m_level": None,
        "body_right_m_level": None,
        "body_down_m_level": None,
        "ground_offset_m_level": None,
        "center_distance_m": None,
        "center_calibrated": False,
        "center_ref_u_px": None,
        "center_ref_v_px": None,
        "measurement_model": "mount_offset_level",
    }
    if depth_frame is None:
        return out
    try:
        dp = rs.rs2_project_color_pixel_to_depth_pixel(
            depth_frame.get_data(), cam.scale, 0.1, 6.0,
            cam.dintr, cam.cintr, cam.c2d, cam.d2c, [u, v])
        dx = int(round(dp[0])); dy = int(round(dp[1]))
        if not (0 <= dx < depth_frame.get_width() and 0 <= dy < depth_frame.get_height()):
            return out
        dist = float(depth_frame.get_distance(dx, dy))
        out["depth_x_px"] = dx
        out["depth_y_px"] = dy
        if dist <= 0.0:
            return out

        # Deproject in the depth imager, then transform into the RGB/color camera frame that bbox uses.
        p_depth = rs.rs2_deproject_pixel_to_point(cam.dintr, [float(dx), float(dy)], dist)
        p_color = rs.rs2_transform_point_to_point(cam.d2c, p_depth)
        xc, yc, zc = (float(p_color[0]), float(p_color[1]), float(p_color[2]))

        theta = math.radians(float(tilt_forward_deg))
        body_forward = cam_offset_forward_m + (-math.cos(theta) * yc + math.sin(theta) * zc)
        body_right = cam_offset_right_m + xc
        body_down = cam_offset_down_m + (math.sin(theta) * yc + math.cos(theta) * zc)
        ground_offset = math.hypot(body_forward, body_right)
        measurement_model = "mount_offset_level"

        if center_calib is not None:
            u0 = float(center_calib["center_u_px"])
            v0 = float(center_calib["center_v_px"])
            p0_color = rs.rs2_deproject_pixel_to_point(cam.cintr, [u0, v0], zc)
            rel_x = xc - float(p0_color[0])
            rel_y = yc - float(p0_color[1])
            rel_z = zc - float(p0_color[2])

            # Calibrated horizontal offset: target ground point relative to the saved
            # body-origin ground-intercept pixel. This is the field STM should consume
            # for level-ground landing correction; STM attitude can further rotate/fuse it.
            body_forward = -math.cos(theta) * rel_y + math.sin(theta) * rel_z
            body_right = rel_x
            ground_offset = math.hypot(body_forward, body_right)
            measurement_model = "center_pixel_level"
        helipad_distance = math.sqrt(body_forward**2 + body_right**2 + body_down**2)

        out.update({
            "range_m": round_or_none(dist, 3),
            "camera_depth_m": round_or_none(dist, 3),
            "cam_x_m": round_or_none(xc, 3),
            "cam_y_m": round_or_none(yc, 3),
            "cam_z_m": round_or_none(zc, 3),
            "helipad_x_m": round_or_none(body_forward, 3),
            "helipad_y_m": round_or_none(body_right, 3),
            "helipad_xy_distance_m": round_or_none(ground_offset, 3),
            "helipad_distance_m": round_or_none(helipad_distance, 3),
            "drone_ground_distance_m": round_or_none(body_down, 3),
            "body_forward_m_level": round_or_none(body_forward, 3),
            "body_right_m_level": round_or_none(body_right, 3),
            "body_down_m_level": round_or_none(body_down, 3),
            "ground_offset_m_level": round_or_none(ground_offset, 3),
            "center_distance_m": round_or_none(ground_offset, 3),
            "center_calibrated": bool(center_calib is not None),
            "center_ref_u_px": None if center_calib is None else round_or_none(center_calib["center_u_px"], 2),
            "center_ref_v_px": None if center_calib is None else round_or_none(center_calib["center_v_px"], 2),
            "measurement_model": measurement_model,
        })
        return out
    except Exception:
        return out


def depth_distance_m(depth_frame, bbox, cam):
    """Backward-compatible distance-only helper."""
    return target_depth_measurement(depth_frame, bbox, cam)["range_m"]


class TelemetrySink:
    """Optional RealSense -> landing-controller advisory telemetry sink."""

    def __init__(self, args):
        self.mode = args.telemetry
        self.format = args.telemetry_format
        self.interval = 0.0 if args.telemetry_hz <= 0 else 1.0 / float(args.telemetry_hz)
        self.next_t = 0.0
        self.file = None
        self.serial = None
        self.latest_path = None
        if self.mode == "jsonl":
            if not args.telemetry_path:
                raise ValueError("--telemetry jsonl requires --telemetry-path")
            self.file = open(args.telemetry_path, "a", encoding="utf-8", buffering=1)
        elif self.mode == "latest":
            # Latest-overwrite: write a single JSON object to a fixed path, replaced
            # atomically every send. The Uno Q bridge app (python/main.py) reads this
            # one object instead of tailing an ever-growing JSONL. Default path matches
            # the host file the container bridge app maps to /app/helipad.json.
            self.latest_path = args.telemetry_path or os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "helipad_latest.json")
            d = os.path.dirname(os.path.abspath(self.latest_path))
            if d:
                os.makedirs(d, exist_ok=True)
        elif self.mode == "serial":
            if not args.serial_port:
                raise ValueError("--telemetry serial requires --serial-port")
            import serial  # optional dependency, only needed for UART telemetry
            self.serial = serial.Serial(args.serial_port, args.serial_baud, timeout=0, write_timeout=0)

    def send(self, packet):
        if self.mode == "off":
            return
        now = time.perf_counter()
        if self.interval > 0.0 and now < self.next_t:
            return
        self.next_t = now + self.interval
        if self.format == "stm":
            packet = packet.get("stm", packet)
        line = json.dumps(packet, ensure_ascii=False, separators=(",", ":"))
        if self.mode == "stdout":
            print("VISION " + line, flush=True)
        elif self.mode == "jsonl":
            self.file.write(line + "\n")
        elif self.mode == "latest":
            # Atomic replace: write a temp file then os.replace() so a reader never
            # sees a half-written object. mtime of the final path tracks freshness.
            tmp = self.latest_path + ".tmp"
            try:
                with open(tmp, "w", encoding="utf-8") as f:
                    f.write(line + "\n")
                os.replace(tmp, self.latest_path)
            except OSError as exc:
                print(f"[WARN] latest telemetry write failed: {exc}", flush=True)
        elif self.mode == "serial":
            self.serial.write((line + "\n").encode("utf-8"))

    def close(self):
        if self.file:
            self.file.close()
        if self.serial:
            self.serial.close()


STATE_COL = {"SEARCH": (160,160,160), "LOST": (0,0,255), "CANDIDATE": (0,255,255), "CONFIRMED": (0,165,255)}


def bbox_debug_metrics(prev_bbox, bbox, frame_width):
    """Per-frame bbox geometry/jump metrics for 30fps robustness analysis."""
    keys = ["bbox_x", "bbox_y", "bbox_w", "bbox_h", "bbox_cx", "bbox_cy", "bbox_area"]
    if bbox is None:
        return {**{k: None for k in keys}, "bbox_jump_px": None, "bbox_jump_frac": None}

    x, y, bw, bh = (int(v) for v in bbox)
    cx = x + bw / 2.0
    cy = y + bh / 2.0
    out = {
        "bbox_x": x,
        "bbox_y": y,
        "bbox_w": bw,
        "bbox_h": bh,
        "bbox_cx": round(cx, 2),
        "bbox_cy": round(cy, 2),
        "bbox_area": int(max(bw, 0) * max(bh, 0)),
        "bbox_jump_px": None,
        "bbox_jump_frac": None,
    }
    if prev_bbox is not None and frame_width > 0:
        px, py, pw, ph = prev_bbox
        pcx = float(px) + float(pw) / 2.0
        pcy = float(py) + float(ph) / 2.0
        jump = math.hypot(cx - pcx, cy - pcy)
        out["bbox_jump_px"] = round(jump, 2)
        out["bbox_jump_frac"] = jump / float(frame_width)
    return out


def debug_event_flags(prev_state, state, prev_src, src, det_miss_latch, bbox_metrics,
                      jump_threshold_frac=0.15):
    """Binary event flags used to align 10fps video with full-rate CSV evidence."""
    jump_frac = None if bbox_metrics is None else bbox_metrics.get("bbox_jump_frac")
    return {
        "event_lost": int(prev_src not in (None, "lost") and src == "lost"),
        "event_center_jump": int(jump_frac is not None and jump_frac > jump_threshold_frac),
        "event_det_miss": int(bool(det_miss_latch) and src in ("trk_pred", "lost")),
        "event_state_change": int(prev_state is not None and state != prev_state),
    }


def draw_debug_overlay(frame, bbox, state, go, src, avg_fps, nv, need_valid, meas, az, el, rng,
                       calibrate_center=False, center_calib=None, record_drops=None):
    """Draw the same human-debug overlay for HDMI preview and recorded debug video."""
    if bbox is not None:
        x, y, bw, bh = bbox
        col = (0,255,0) if go else STATE_COL.get(state, (200,200,200))
        cv2.rectangle(frame, (x, y), (x+bw, y+bh), col, 3 if go else 2)
        cv2.circle(frame, (x+bw//2, y+bh//2), 4, (0,0,255), -1)
        fr = (meas or {}).get("body_forward_m_level")
        rr = (meas or {}).get("body_right_m_level")
        cd = (meas or {}).get("center_distance_m")
        off = f" F{fr:+.2f} R{rr:+.2f} C{cd:.2f}" if fr is not None and rr is not None and cd is not None else ""
        info = f"az{az:+.0f} el{el:+.0f}" + (f" {rng}m" if rng else " ?m") + off
        cv2.putText(frame, info, (x, max(y-8, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    banner = "LANDING GO" if go else state
    bcol = (0,255,0) if go else STATE_COL.get(state, (200,200,200))
    suffix = " CALIB" if calibrate_center else (" CENTER" if center_calib else " MOUNT")
    line = f"{banner}{suffix}  FPS:{avg_fps:.0f}  src:{src} v:{nv}/{need_valid}"
    if record_drops is not None:
        line += f" rec_drop:{record_drops}"
    cv2.putText(frame, line, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bcol, 2)
    return frame


class VideoRecorder:
    """Non-blocking debug video writer. Queue-full drops recording frames, never main-loop frames."""

    def __init__(self, path, fps=10.0, codec="MJPG", queue_size=2):
        self.path = path or ""
        self.fps = float(fps)
        self.codec = (codec or "MJPG")[:4]
        self.queue_size = max(1, int(queue_size))
        self.interval = 0.0 if self.fps <= 0 else 1.0 / self.fps
        self.next_t = 0.0
        self.submitted = 0
        self.written = 0
        self.dropped = 0
        self.error = None
        self._q = None
        self._stop = None
        self._thread = None
        if self.enabled:
            out_dir = os.path.dirname(os.path.abspath(self.path))
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            self._q = queue.Queue(maxsize=self.queue_size)
            self._stop = threading.Event()
            self._thread = threading.Thread(target=self._run, name="debug-video-writer", daemon=True)
            self._thread.start()

    @property
    def enabled(self):
        return bool(self.path) and self.fps > 0

    def is_due(self, now=None):
        if not self.enabled or self.error:
            return False
        now = time.perf_counter() if now is None else now
        return now >= self.next_t

    def submit(self, frame_idx, t_mono, frame):
        if not self.is_due(t_mono):
            return False
        self.next_t = t_mono + self.interval
        try:
            self._q.put_nowait((int(frame_idx), float(t_mono), frame))
            self.submitted += 1
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        writer = None
        try:
            fourcc = cv2.VideoWriter_fourcc(*self.codec)
            while not self._stop.is_set() or not self._q.empty():
                try:
                    _frame_idx, _t_mono, frame = self._q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if writer is None:
                    h, w = frame.shape[:2]
                    writer = cv2.VideoWriter(self.path, fourcc, self.fps, (int(w), int(h)))
                    if hasattr(writer, "isOpened") and not writer.isOpened():
                        raise RuntimeError(f"VideoWriter open failed: {self.path}")
                writer.write(frame)
                self.written += 1
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            if writer is not None:
                writer.release()

    def close(self):
        if not self.enabled:
            return
        self._stop.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive() and not self.error:
            self.error = "writer thread did not stop within 5s"


def make_vision_packet(frame_idx, t_mono, state, go, src, score, bbox, az, el, meas, nv, nd,
                       fps_avg, proc_ms, det_age, detect_ms, trk_score, args):
    bbox_obj = None
    if bbox is not None:
        bbox_obj = {"x": int(bbox[0]), "y": int(bbox[1]), "w": int(bbox[2]), "h": int(bbox[3])}
    target_detected = bbox is not None
    target_fresh = src in ("det", "trk")
    measurement_valid = bool(
        target_detected and target_fresh and state == "CONFIRMED" and
        meas is not None and meas.get("range_m") is not None and
        meas.get("center_distance_m") is not None
    )
    camera_depth_m = None if meas is None else meas.get("camera_depth_m", meas.get("range_m"))
    drone_ground_distance_m = None if meas is None else meas.get("drone_ground_distance_m", meas.get("body_down_m_level"))
    helipad_x_m = None if meas is None else meas.get("helipad_x_m", meas.get("body_forward_m_level"))
    helipad_y_m = None if meas is None else meas.get("helipad_y_m", meas.get("body_right_m_level"))
    helipad_xy_distance_m = None if meas is None else meas.get("helipad_xy_distance_m", meas.get("center_distance_m"))
    helipad_distance_m = None if meas is None else meas.get("helipad_distance_m")
    measurement_model = None if meas is None else meas.get("measurement_model")
    stm_payload = {
        "type": "helipad_stm",
        "schema": 1,
        "frame": int(frame_idx),
        "t_mono_s": round_or_none(t_mono, 3),
        "valid": bool(measurement_valid),
        "frame_id": "body_frd_level",
        "drone_ground_distance_m": drone_ground_distance_m,
        "helipad_distance_m": helipad_distance_m,
        "helipad_x_m": helipad_x_m,
        "helipad_y_m": helipad_y_m,
        "helipad_xy_distance_m": helipad_xy_distance_m,
        "camera_depth_m": camera_depth_m,
        "measurement_model": measurement_model,
        "center_calibrated": False if meas is None else bool(meas.get("center_calibrated")),
    }
    return {
        "type": "helipad_vision",
        "schema": 1,
        "frame": int(frame_idx),
        "t_mono_s": round_or_none(t_mono, 3),
        "valid": bool(measurement_valid),
        "target_detected": bool(target_detected),
        "target_fresh": bool(target_fresh),
        "vision_confirmed": bool(state == "CONFIRMED"),
        "landing_go_advisory": bool(go),
        "state": state,
        "src": src,
        "bbox": bbox_obj,
        "center_u_px": None if meas is None else meas.get("center_u_px"),
        "center_v_px": None if meas is None else meas.get("center_v_px"),
        "conf": round_or_none(score, 3),
        "az_deg": az,
        "el_deg": el,
        "range_m": None if meas is None else meas.get("range_m"),
        "camera_depth_m": camera_depth_m,
        "drone_ground_distance_m": drone_ground_distance_m,
        "helipad_distance_m": helipad_distance_m,
        "helipad_x_m": helipad_x_m,
        "helipad_y_m": helipad_y_m,
        "helipad_xy_distance_m": helipad_xy_distance_m,
        "offset_forward_m": helipad_x_m,
        "offset_right_m": helipad_y_m,
        "center_distance_m": helipad_xy_distance_m,
        "body_forward_m_level": None if meas is None else meas.get("body_forward_m_level"),
        "body_right_m_level": None if meas is None else meas.get("body_right_m_level"),
        "body_down_m_level": None if meas is None else meas.get("body_down_m_level"),
        "ground_offset_m_level": None if meas is None else meas.get("ground_offset_m_level"),
        "measurement_model": measurement_model,
        "center_calibrated": False if meas is None else bool(meas.get("center_calibrated")),
        "center_ref_u_px": None if meas is None else meas.get("center_ref_u_px"),
        "center_ref_v_px": None if meas is None else meas.get("center_ref_v_px"),
        "stm": stm_payload,
        "quality": {
            "nvalid": int(nv),
            "need_valid": 12,
            "ndet": int(nd),
            "fps": round_or_none(fps_avg, 1),
            "proc_ms": round_or_none(proc_ms, 1),
            "det_age_frames": int(det_age),
            "detect_ms": round_or_none(detect_ms, 1),
            "trk_score": round_or_none(trk_score, 3),
        },
        "camera_mount": {
            "frame": "body_frd",
            "tilt_forward_deg": round_or_none(args.cam_tilt_forward_deg, 2),
            "offset_forward_m": round_or_none(args.cam_offset_forward_m, 3),
            "offset_right_m": round_or_none(args.cam_offset_right_m, 3),
            "offset_down_m": round_or_none(args.cam_offset_down_m, 3),
            "estimate": "level_body_only; STM roll/pitch/yaw must fuse this before final landing",
        },
    }


def observation_angles(bbox):
    """body frame 관측각(deg): 방위각 우+, 고각 위+."""
    u = bbox[0] + bbox[2]/2.0; v = bbox[1] + bbox[3]/2.0
    az = math.degrees(math.atan2(u - CX, FX))
    el = math.degrees(math.atan2(-(v - CY), FY))
    return round(az, 2), round(el, 2)


def center_jump(b1, b2, w):
    if b1 is None or b2 is None:
        return False
    c1 = (b1[0]+b1[2]/2, b1[1]+b1[3]/2); c2 = (b2[0]+b2[2]/2, b2[1]+b2[3]/2)
    return ((c1[0]-c2[0])**2 + (c1[1]-c2[1])**2) ** 0.5 > 0.15 * w


def make_tracker(args):
    """--tracker 값에 따라 LK 또는 NanoTrack(Siamese) 추적기 생성."""
    if args.tracker == "nano":
        here = os.path.dirname(os.path.abspath(__file__))
        bb = args.nano_backbone if os.path.isabs(args.nano_backbone) else os.path.join(here, args.nano_backbone)
        nh = args.nano_head if os.path.isabs(args.nano_head) else os.path.join(here, args.nano_head)
        return NanoTracker(bb, nh, min_score=args.nano_min_score)
    return LKTracker()


def set_affinity(cores):
    """현재 프로세스를 지정 CPU 코어에 고정(Linux sched_setaffinity). cores='0,1' 형식.
    메인/detection을 다른 코어로 분리해 NN 추론의 CPU 경합이 메인 worst-case를 흔드는 것 방지.
    미지원(Windows 등)·실패 시 조용히 무시(동작엔 영향 없음)."""
    if cores and hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, {int(c) for c in str(cores).split(",") if c.strip() != ""})
        except Exception as e:
            print(f"⚠️ affinity({cores}) 설정 실패: {e}", flush=True)


def lock_memory():
    """mlockall로 메모리를 RAM에 고정 → page fault로 인한 worst-case 스파이크 제거(root 권한 필요, 실패 시 무시)."""
    try:
        if ctypes.CDLL("libc.so.6", use_errno=True).mlockall(3) != 0:  # MCL_CURRENT|MCL_FUTURE
            print(f"⚠️ mlockall 실패(권한? ulimit -l): errno={ctypes.get_errno()}", flush=True)
    except Exception as e:
        print(f"⚠️ mlockall 예외: {e}", flush=True)


def set_rt_priority(prio=10):
    """메인 스레드를 SCHED_FIFO RT로 → 스케줄 지터(worst-case) 감소(root/CAP_SYS_NICE 필요, 실패 시 무시).
    prio는 낮게 유지(시스템/드라이버 starvation 방지)."""
    if hasattr(os, "sched_setscheduler") and hasattr(os, "SCHED_FIFO"):
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
        except Exception as e:
            print(f"⚠️ RT priority 실패(권한?): {e}", flush=True)


def detection_worker(in_q, out_q, ready_ev, stop_ev, param, binf, conf, threads, nice, det_cores):
    """[multiprocessing child] NCNN detection을 별도 *프로세스*에서 실행 → GIL 분리(thread는 GIL로 실패).
    in_q에서 최신 frame을 받아 detect, 결과 (bbox|None, score, frame_id, detect_ms)를 out_q(최신만)로 전달.
    spawn 컨텍스트 전제: top-level 함수라 pickle 가능, Detector는 child에서 새로 load."""
    set_affinity(det_cores)      # spawn 상속 무시하고 detection 전용 코어로 재고정(메인과 분리)
    try:
        if nice:
            os.nice(nice)        # detection을 낮은 우선순위로 → 메인 starvation 방지
    except Exception:
        pass
    det = Detector(param, binf, threads=threads)
    det.detect(np.zeros((480, 640, 3), np.uint8), conf)   # warm-up: 첫 추론 컴파일/할당을 ready 전에
    ready_ev.set()
    while not stop_ev.is_set():
        try:
            frame, fid, t_cap = in_q.get(timeout=0.2)
        except queue.Empty:
            continue
        td = time.perf_counter()
        r = det.detect(frame, conf)
        dms = (time.perf_counter() - td) * 1000.0
        res = (r[0], r[1], fid, dms, t_cap) if r is not None else (None, 0.0, fid, dms, t_cap)
        try:
            out_q.get_nowait()          # 최신만: 묵은 결과 폐기
        except queue.Empty:
            pass
        try:
            out_q.put_nowait(res)
        except queue.Full:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--param", default="ssdlite_v3small_helipad.param")
    p.add_argument("--bin", default="ssdlite_v3small_helipad.bin")
    p.add_argument("--conf", type=float, default=0.5)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--redetect-n", type=int, default=5)
    p.add_argument("--threads", type=int, default=3)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--log", default="run.csv",
                   help="CSV log path. Default: run.csv")
    p.add_argument("--no-log", dest="log", action="store_const", const="",
                   help="Disable CSV logging.")
    p.add_argument("--headless", dest="headless", action="store_true", default=True,
                   help="Run without OpenCV display window. Default.")
    p.add_argument("--show", dest="headless", action="store_false",
                   help="Show OpenCV display window for HDMI/TV debugging.")
    p.add_argument("--record", default="",
                   help="Optional non-blocking debug video path. Default off; e.g. debug_overlay.avi")
    p.add_argument("--record-fps", type=float, default=10.0,
                   help="Debug video FPS. CSV/logging remains full-rate. Default 10.")
    p.add_argument("--record-overlay", dest="record_overlay", action="store_true", default=True,
                   help="Record bbox/state/FPS overlay video. Default.")
    p.add_argument("--record-raw", dest="record_overlay", action="store_false",
                   help="Record raw color frames without overlay.")
    p.add_argument("--record-codec", default="MJPG",
                   help="OpenCV fourcc for debug video, e.g. MJPG for .avi or mp4v for .mp4. Default MJPG.")
    p.add_argument("--record-queue", type=int, default=2,
                   help="Writer queue size. If full, recording frames are dropped; main loop never blocks.")
    p.add_argument("--tracker", choices=["lk", "nano"], default="lk",
                   help="lk=Lucas-Kanade optical flow, nano=NanoTrack(Siamese)")
    p.add_argument("--nano-backbone", default="models/nanotrack_backbone_sim.onnx")
    p.add_argument("--nano-head", default="models/nanotrack_head_sim.onnx")
    p.add_argument("--nano-min-score", type=float, default=0.0,
                   help="NanoTrack getTrackingScore() 최소값(미만이면 추적실패 처리). 0=비활성(실주행 score 관찰 후 튜닝)")
    p.add_argument("--mp-det", dest="mp_det", action="store_true", default=True,
                   help="detection을 별도 프로세스로 비동기 실행(multiprocessing, GIL 분리 → FPS dip 제거). thread 비동기는 GIL로 실패해 이 방식 사용.")
    p.add_argument("--no-mp-det", "--sync-det", dest="mp_det", action="store_false",
                   help="Disable multiprocessing detector and run detection synchronously for debugging.")
    p.add_argument("--det-threads", type=int, default=2,
                   help="[mp] detection 프로세스 NCNN 스레드 수(기본 2; A53 4코어에서 메인 starvation 방지 위해 3→2).")
    p.add_argument("--det-nice", type=int, default=5,
                   help="[mp] detection 프로세스 nice 값(+5, 메인 우선).")
    p.add_argument("--main-cores", default="0,1",
                   help="[mp] 메인 프로세스 CPU 코어(affinity). A53 4코어 기본 0,1. 빈값이면 미설정.")
    p.add_argument("--det-cores", default="2,3",
                   help="[mp] detection 프로세스 CPU 코어(affinity). 메인과 분리해 NN 경합 제거. 기본 2,3.")
    p.add_argument("--det-max-age", type=int, default=6,
                   help="[mp] detection 결과 허용 최대 age(프레임). detect 138ms≈4프레임+여유. 초과 stale은 폐기.")
    p.add_argument("--det-max-age-ms", type=float, default=350.0,
                   help="[mp] detection 결과 허용 최대 age(ms, wall-clock). detect_ms(~138ms)+큐/poll 지연 수용. 초과 stale은 폐기(GC 등 메인 멈춤도 차단) → GO 안전.")
    p.add_argument("--cam-offset-forward-m", type=float, default=DEFAULT_CAM_OFFSET_FORWARD_M,
                   help="RGB lens position from drone control origin, +forward, meters")
    p.add_argument("--cam-offset-right-m", type=float, default=DEFAULT_CAM_OFFSET_RIGHT_M,
                   help="RGB lens position from drone control origin, +right, meters")
    p.add_argument("--cam-offset-down-m", type=float, default=DEFAULT_CAM_OFFSET_DOWN_M,
                   help="RGB lens position from drone control origin, +down, meters")
    p.add_argument("--cam-tilt-forward-deg", type=float, default=DEFAULT_CAM_TILT_FORWARD_DEG,
                   help="Camera optical axis tilt from straight-down toward drone +forward. Use 0 after vertical remount.")
    p.add_argument("--center-calib-file", default=DEFAULT_CENTER_CALIB_FILE,
                   help="Drone-center pixel calibration JSON. Default: helipad_center_calib.json")
    p.add_argument("--calibrate-center", action="store_true",
                   help="One-time mode: hold drone origin over helipad center and save center pixel calibration.")
    p.add_argument("--calib-frames", type=int, default=30,
                   help="Stable CONFIRMED frames to average in --calibrate-center mode.")
    p.add_argument("--no-center-calib", action="store_true",
                   help="Ignore saved center calibration and use physical mount offsets only.")
    p.add_argument("--telemetry", choices=["off", "stdout", "jsonl", "latest", "serial"], default="off",
                   help="Optional advisory target telemetry: stdout prints 'VISION {json}', jsonl appends to file, "
                        "latest atomically overwrites a single-object JSON file (for the Uno Q bridge app), "
                        "serial writes JSON lines to UART.")
    p.add_argument("--telemetry-format", choices=["full", "stm"], default="full",
                   help="full=complete vision JSON, stm=compact STM landing payload only.")
    p.add_argument("--telemetry-path", default="",
                   help="Output path: JSONL file when --telemetry jsonl, or the single-object "
                        "latest file when --telemetry latest (default helipad_latest.json next to the script).")
    p.add_argument("--telemetry-hz", type=float, default=10.0, help="Telemetry output rate; <=0 means every processed frame")
    p.add_argument("--serial-port", default="", help="Serial device when --telemetry serial, e.g. /dev/ttyUSB0")
    p.add_argument("--serial-baud", type=int, default=115200, help="Serial baud when --telemetry serial")
    args = p.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    param = args.param if os.path.isabs(args.param) else os.path.join(here, args.param)
    binf = args.bin if os.path.isabs(args.bin) else os.path.join(here, args.bin)
    if args.calib_frames < 1:
        raise ValueError("--calib-frames must be >= 1")
    if args.record and args.record_fps <= 0:
        raise ValueError("--record-fps must be > 0 when --record is set")
    if args.record and args.record_queue < 1:
        raise ValueError("--record-queue must be >= 1 when --record is set")

    center_calib = None
    if args.calibrate_center:
        print(f"[CALIB] 드론 중심이 헬리패드 중앙 위에 오도록 들고 유지하세요. "
              f"CONFIRMED {args.calib_frames}프레임 평균을 {args.center_calib_file}에 저장합니다.", flush=True)
    elif not args.no_center_calib:
        center_calib = load_center_calib(args.center_calib_file)
        if center_calib:
            print(f"[CENTER] loaded {args.center_calib_file}: "
                  f"u={center_calib['center_u_px']:.2f}, v={center_calib['center_v_px']:.2f}, "
                  f"samples={center_calib['samples']}", flush=True)
        else:
            print(f"[CENTER] calibration file not found; using physical mount offsets. "
                  f"Run once with --calibrate-center after assembly.", flush=True)

    if args.mp_det:
        set_affinity(args.main_cores)   # 메인을 전용 코어로 고정(detection NN 경합 분리; Codex/Pro)
        gc.disable()                    # GC 스파이크로 인한 worst-case 튐 방지(측정/단기; 장기 비행은 주기적 gc.collect 설계 필요)
        lock_memory()                   # page fault 스파이크 제거(mlockall)
        set_rt_priority()               # 스케줄 지터 제거(SCHED_FIFO); 둘 다 root 권한 필요, 실패 시 무시
    cam = RealSenseCam(args.width, args.height, args.fps)
    diag = (args.width**2 + args.height**2) ** 0.5
    gate = LandingGate(diag)
    det = None; proc = in_q = out_q = stop_ev = None
    if args.mp_det:
        ctx = mp.get_context("spawn")              # fork 금지(NCNN/RealSense 핸들 상속 위험) — Codex 권고
        in_q = ctx.Queue(maxsize=1); out_q = ctx.Queue(maxsize=1)
        ready_ev = ctx.Event(); stop_ev = ctx.Event()
        proc = ctx.Process(target=detection_worker,
                           args=(in_q, out_q, ready_ev, stop_ev, param, binf,
                                 args.conf, args.det_threads, args.det_nice, args.det_cores), daemon=True)
        proc.start()
        print("detection 프로세스 warm-up 중...", flush=True)
        if not ready_ev.wait(timeout=60):
            if not proc.is_alive():
                raise RuntimeError(f"detection 프로세스 시작 실패(exitcode={proc.exitcode}) — 모델/경로/메모리 확인")
            print("⚠️ warm-up 60s timeout이나 프로세스는 생존 — 계속 진행(초기 결과 지연 가능)", flush=True)
    else:
        det = Detector(param, binf, threads=args.threads)
    mode = "MP" if args.mp_det else "SYNC"
    record_label = "off" if not args.record else f"{args.record}@{args.record_fps:g}fps"
    print(f"RealSense color+depth + SSDLite512 + {args.tracker.upper()} + angles + depth + LandingGate [{mode}]. "
          f"conf={args.conf} N={args.redetect_n} det_max_age={args.det_max_age} depth_scale={cam.scale} "
          f"cam_offset=({args.cam_offset_forward_m:.3f},{args.cam_offset_right_m:.3f},{args.cam_offset_down_m:.3f})m "
          f"tilt_forward={args.cam_tilt_forward_deg:.1f}deg center_calib={bool(center_calib)} telemetry={args.telemetry} "
          f"record={record_label}",
          flush=True)

    telemetry = TelemetrySink(args)
    recorder = VideoRecorder(args.record, fps=args.record_fps,
                             codec=args.record_codec, queue_size=args.record_queue)
    logf = open(args.log, "w", newline="") if args.log else None
    logw = csv.writer(logf) if logf else None
    if logw:
        logw.writerow(["frame", "t", "state", "GO_advisory", "src", "conf", "az", "el", "range_m",
                       "camera_depth_m", "drone_ground_distance_m", "helipad_distance_m",
                       "helipad_x_m", "helipad_y_m", "helipad_xy_distance_m",
                       "center_u_px", "center_v_px", "depth_x_px", "depth_y_px",
                       "body_forward_m_level", "body_right_m_level", "body_down_m_level", "ground_offset_m_level",
                       "center_distance_m", "center_calibrated", "measurement_model",
                       "bbox_x", "bbox_y", "bbox_w", "bbox_h", "bbox_cx", "bbox_cy", "bbox_area",
                       "bbox_jump_px", "bbox_jump_frac",
                       "event_lost", "event_center_jump", "event_det_miss", "event_state_change",
                       "record_frame_submitted", "record_drop_count", "record_submit_count", "record_written_count",
                       "nvalid", "ndet", "fps", "proc_ms", "det_age", "detect_ms", "trk_score"])

    tracker = make_tracker(args)
    trk_active = False; det_miss_latch = False; bbox = None; score = 0.0
    frame_idx = 0; fps_buf = []; proc_all = []; go_frames = 0; det_age = -1; detect_ms = -1.0
    prev_bbox_for_log = None; prev_state = None; prev_src = None
    calib_samples = []

    try:
        while True:
            t0 = time.perf_counter()
            color, depth = cam.read()
            if color is None:
                continue
            frame = color
            h, w = frame.shape[:2]
            src = "lost"
            t_cap = time.perf_counter()    # frame capture 시점(wall-clock age 기준, mp stale 판정용)

            if args.mp_det:
                # ---- multiprocessing: detection은 별도 프로세스(GIL 분리), 메인은 매 프레임 tracker(dip 제거) ----
                try:
                    in_q.get_nowait()           # 최신만: 묵은 프레임 폐기
                except queue.Empty:
                    pass
                try:
                    in_q.put_nowait((frame.copy(), frame_idx, t_cap))
                except queue.Full:
                    pass
                if trk_active:
                    ok, tb = tracker.update(frame)
                    if ok and not center_jump(bbox, tb, w) and not (bbox and tb[2]*tb[3] > 1.5*bbox[2]*bbox[3]):
                        bbox = tb; src = "trk_pred" if det_miss_latch else "trk"
                    else:
                        trk_active = False; bbox = None; src = "lost"
                try:
                    d_bbox, d_score, d_fid, d_ms, d_tcap = out_q.get_nowait()
                    det_age = frame_idx - d_fid; detect_ms = d_ms
                    age_ms = (time.perf_counter() - d_tcap) * 1000.0
                    # 프레임 카운트 + wall-clock 둘 다 fresh여야 GO 증거로 인정(메인 멈춤 시 stale 오판 방지)
                    fresh = (det_age <= args.det_max_age) and (age_ms <= args.det_max_age_ms)
                    if d_bbox is not None and fresh:
                        # fresh positive: 재init + GO 증거(det)
                        trk_active = tracker.init(frame, d_bbox)
                        bbox = d_bbox; score = d_score; src = "det"; det_miss_latch = False
                    elif d_bbox is None and fresh:
                        # fresh miss: GO 금지 + 이 프레임 즉시 강등(GO 1프레임 누수 방지)
                        det_miss_latch = True
                        if src in ("trk", "trk_pred"):
                            src = "trk_pred"
                    # stale(프레임 age>max 또는 wall-clock age_ms>max): 재init·GO 증거 모두 안 함 — 추적기 유지
                except queue.Empty:
                    pass
            else:
                # ---- 동기(기존, 검증됨): N프레임마다 blocking detection ----
                redetect = (not trk_active) or (frame_idx % args.redetect_n == 0)
                tracker_rejected = False
                if not redetect and trk_active:
                    ok, tb = tracker.update(frame)
                    if not ok:
                        redetect = True; tracker_rejected = True
                    elif center_jump(bbox, tb, w) or (bbox and tb[2]*tb[3] > 1.5*bbox[2]*bbox[3]):
                        redetect = True; tracker_rejected = True
                    else:
                        bbox = tb; src = "trk_pred" if det_miss_latch else "trk"
                if redetect:
                    r = det.detect(frame, args.conf)
                    if r is not None:
                        nbbox, score = r
                        trk_active = tracker.init(frame, nbbox)
                        bbox = nbbox; src = "det"; det_miss_latch = False
                    elif trk_active and not tracker_rejected:
                        ok, tb = tracker.update(frame)
                        if ok:
                            bbox = tb; src = "trk_pred"
                        else:
                            trk_active = False; bbox = None; src = "lost"
                        det_miss_latch = True
                    else:
                        if tracker_rejected:
                            trk_active = False
                        bbox = None; src = "lost"; det_miss_latch = True

            da = det_age if (args.mp_det and src == "det") else 0
            state, go, nv, nd = gate.update(bbox, src, score if src == "det" else 0.0, da)
            az = el = rng = None
            meas = None
            if bbox is not None:
                az, el = observation_angles(bbox)
                meas = target_depth_measurement(
                    depth, bbox, cam, args.cam_tilt_forward_deg,
                    args.cam_offset_forward_m, args.cam_offset_right_m, args.cam_offset_down_m,
                    center_calib)
                rng = meas.get("range_m")
            bbox_metrics = bbox_debug_metrics(prev_bbox_for_log, bbox, w)
            event_flags = debug_event_flags(prev_state, state, prev_src, src, det_miss_latch, bbox_metrics)
            if args.calibrate_center:
                if (meas is not None and state == "CONFIRMED" and src in ("det", "trk") and
                        meas.get("range_m") is not None and meas.get("center_u_px") is not None):
                    calib_samples.append({
                        "u": float(meas["center_u_px"]),
                        "v": float(meas["center_v_px"]),
                        "range_m": float(meas["range_m"]),
                    })
                    step = max(1, args.calib_frames // 5)
                    if len(calib_samples) == 1 or len(calib_samples) % step == 0:
                        print(f"[CALIB] samples={len(calib_samples)}/{args.calib_frames} "
                              f"u={meas['center_u_px']} v={meas['center_v_px']} range={meas['range_m']}m",
                              flush=True)
                    if len(calib_samples) >= args.calib_frames:
                        saved = save_center_calib(args.center_calib_file, calib_samples, args)
                        print(f"[CALIB] saved {args.center_calib_file}: "
                              f"u={saved['center_u_px']:.3f}, v={saved['center_v_px']:.3f}, "
                              f"avg_range={saved['avg_range_m']}m, samples={saved['samples']}", flush=True)
                        break
                elif frame_idx % max(1, args.fps) == 0 and frame_idx > 0:
                    print(f"[CALIB] waiting CONFIRMED target... state={state} src={src} "
                          f"valid={nv}/{gate.need}", flush=True)
            if go:
                go_frames += 1

            elapsed = time.perf_counter() - t0          # 메인 연산시간(sleep 전) = FPS dip / worst-case 지표
            proc_all.append(elapsed * 1000)
            sl = (1.0/args.fps) - elapsed
            if sl > 0:
                time.sleep(sl)
            act = time.perf_counter() - t0
            fps_buf.append(1.0/max(act, 1e-6))
            if len(fps_buf) > 30:
                fps_buf.pop(0)
            avg = sum(fps_buf)/len(fps_buf)

            ts = getattr(tracker, "last_score", None)
            record_frame_submitted = 0
            rec_t = time.perf_counter()
            if recorder.is_due(rec_t):
                rec_frame = frame.copy()
                if args.record_overlay:
                    draw_debug_overlay(rec_frame, bbox, state, go, src, avg, nv, gate.need, meas, az, el, rng,
                                       calibrate_center=args.calibrate_center, center_calib=center_calib,
                                       record_drops=recorder.dropped)
                record_frame_submitted = int(recorder.submit(frame_idx, rec_t, rec_frame))

            if logw:
                logw.writerow([frame_idx, f"{time.perf_counter():.3f}", state, int(go), src,
                               f"{score:.2f}", az, el, rng,
                               (meas or {}).get("camera_depth_m"),
                               (meas or {}).get("drone_ground_distance_m"),
                               (meas or {}).get("helipad_distance_m"),
                               (meas or {}).get("helipad_x_m"),
                               (meas or {}).get("helipad_y_m"),
                               (meas or {}).get("helipad_xy_distance_m"),
                               (meas or {}).get("center_u_px"), (meas or {}).get("center_v_px"),
                               (meas or {}).get("depth_x_px"), (meas or {}).get("depth_y_px"),
                               (meas or {}).get("body_forward_m_level"),
                               (meas or {}).get("body_right_m_level"),
                               (meas or {}).get("body_down_m_level"),
                               (meas or {}).get("ground_offset_m_level"),
                               (meas or {}).get("center_distance_m"),
                               int(bool((meas or {}).get("center_calibrated"))),
                               (meas or {}).get("measurement_model"),
                               bbox_metrics["bbox_x"], bbox_metrics["bbox_y"],
                               bbox_metrics["bbox_w"], bbox_metrics["bbox_h"],
                               bbox_metrics["bbox_cx"], bbox_metrics["bbox_cy"],
                               bbox_metrics["bbox_area"], bbox_metrics["bbox_jump_px"],
                               bbox_metrics["bbox_jump_frac"],
                               event_flags["event_lost"], event_flags["event_center_jump"],
                               event_flags["event_det_miss"], event_flags["event_state_change"],
                               record_frame_submitted, recorder.dropped, recorder.submitted, recorder.written,
                               nv, nd, f"{avg:.1f}", f"{elapsed*1000:.1f}", det_age, f"{detect_ms:.1f}",
                               (f"{ts:.3f}" if ts is not None else "")])

            packet = make_vision_packet(frame_idx, time.perf_counter(), state, go, src, score, bbox, az, el,
                                        meas, nv, nd, avg, elapsed * 1000.0, det_age, detect_ms, ts, args)
            telemetry.send(packet)

            if not args.headless:
                draw_debug_overlay(frame, bbox, state, go, src, avg, nv, gate.need, meas, az, el, rng,
                                   calibrate_center=args.calibrate_center, center_calib=center_calib,
                                   record_drops=recorder.dropped if recorder.enabled else None)
                cv2.imshow("Helipad landing", frame)
                if cv2.waitKey(1) == 27:
                    break
            else:
                if frame_idx % args.fps == 0 and frame_idx > 0:
                    if az is not None:
                        fr = (meas or {}).get("body_forward_m_level")
                        rr = (meas or {}).get("body_right_m_level")
                        goff = (meas or {}).get("ground_offset_m_level")
                        cd = (meas or {}).get("center_distance_m")
                        model = (meas or {}).get("measurement_model")
                        offset = (f" fwd={fr:+.2f}m right={rr:+.2f}m center={cd:.2f}m "
                                  f"off={goff:.2f}m model={model}") if fr is not None and rr is not None and cd is not None else ""
                        ang = f" az={az:+.1f} el={el:+.1f} range={rng}m{offset}"
                    else:
                        ang = ""
                    print(f"[{frame_idx}] state={state} GO={int(go)} src={src} "
                          f"valid={nv}/{gate.need} det={nd} age={det_age} det_ms={detect_ms:.0f}{ang} FPS={avg:.1f}", flush=True)
            prev_bbox_for_log = bbox
            prev_state = state
            prev_src = src
            frame_idx += 1
    except KeyboardInterrupt:
        pass
    finally:
        if proc is not None:
            stop_ev.set()
            proc.join(timeout=2)
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
            for q in (in_q, out_q):
                try:
                    q.close(); q.cancel_join_thread()
                except Exception:
                    pass
        cam.stop()
        if not args.headless:
            cv2.destroyAllWindows()
        if 'telemetry' in locals():
            telemetry.close()
        if 'recorder' in locals():
            recorder.close()
        if logf:
            logf.close()
        if proc_all:
            a = sorted(proc_all)
            p50 = a[len(a)//2]; p95 = a[min(len(a)-1, int(len(a)*0.95))]
            p99 = a[min(len(a)-1, int(len(a)*0.99))]
            print(f"\n=== Summary [{mode}] === avg FPS={sum(fps_buf)/len(fps_buf):.1f}  GO frames={go_frames}/{frame_idx}  "
                  f"proc_ms p50={p50:.1f} p95={p95:.1f} p99={p99:.1f} max={a[-1]:.1f}", flush=True)
            if 'recorder' in locals() and recorder.enabled:
                rec_msg = (f"record path={recorder.path} fps={recorder.fps:g} submitted={recorder.submitted} "
                           f"written={recorder.written} dropped={recorder.dropped}")
                if recorder.error:
                    rec_msg += f" ERROR={recorder.error}"
                print(rec_msg, flush=True)


if __name__ == "__main__":
    main()
