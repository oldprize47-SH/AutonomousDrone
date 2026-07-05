#!/usr/bin/env python3
"""
MISSION 모드 1:1 미러 시뮬레이터 (firmware Mode.cpp 기준).

목적: 업로드한 미션 .txt(lat lon alt flags)로 MODE_MISSION 전체 시퀀스
  GROUND -> SPOOLUP -> TAKEOFF -> HOLD -> GUIDANCE(PNG+orbit+avoid+alt ramp)
        -> LAND -> LAND_SLOW -> SPOOLDOWN -> DISARMED
를 펌웨어와 동일한 식으로 돌려 로직을 검증한다.

운동학 가정 (펌웨어 불변식과 일치):
  - GUIDANCE 수평: 전진속도 = PNG_VX_MS(속도제어 수렴 가정), 헤딩=yaw,
    yaw_rate = png_compute_guidance() 출력. WP 캡처 = 수평거리<PNG_WP_CAPTURE_R.
  - 수직: 시퀀서 climb-rate / alt-hold. 1차 모델(rate-limited 추종).
  - 좌표변환: 펌웨어 lla_to_ecef + png_lla_to_ned 그대로 포팅.

펌웨어 상수는 hw_config.h에서 읽어와 하드코딩과 불일치하지 않게 한다.
"""
import math, sys, os, re

# ---------------- hw_config.h에서 상수 파싱 ----------------
HW = os.path.join(os.path.dirname(__file__), "..", "hw_config.h")

def load_defines(path):
    d = {}
    rx = re.compile(r'#define\s+(\w+)\s+([^/\n]+)')
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = rx.match(line.strip())
            if not m:
                continue
            name, val = m.group(1), m.group(2).strip()
            d[name] = val
    return d

RAW = load_defines(HW)

def num(name, default=None):
    """#define 값을 float로. 매크로 참조/연산은 간단 처리."""
    if name not in RAW:
        if default is not None:
            return default
        raise KeyError(name)
    v = RAW[name].rstrip("f").rstrip("F")
    v = v.replace("f ", " ").strip()
    try:
        return float(v)
    except ValueError:
        return default

CONTROL_FREQ_HZ   = num("CONTROL_FREQ_HZ", 200.0)
DT                = 1.0 / CONTROL_FREQ_HZ
PNG_KP            = num("PNG_KP", 3.0)
PNG_VX_MS         = num("PNG_VX_MS", 1.5)
PNG_WP_CAPTURE_R  = num("PNG_WP_CAPTURE_R", 1.5)
PNG_RNG_MIN       = num("PNG_RNG_MIN", 2.0)
RC_MAX_YAW_RATE   = math.radians(num("RC_MAX_YAW_RATE_DPS", 90.0))
ORBIT_K2          = num("ORBIT_K2", 1.5)
ORBIT_R_MIN       = num("ORBIT_R_MIN", 0.5)
ORBIT_EXIT_TURN   = math.radians(num("ORBIT_EXIT_TURN_DEG", 180.0))
WP_FLAG_AVOID     = 0x01
WP_FLAG_ORBIT     = 0x02
TKO_CLIMB_RATE    = num("TKO_CLIMB_RATE", 0.5)
TKO_TARGET_ALT    = num("TKO_TARGET_ALT", 2.0)
LAND_SPEED_HIGH   = num("LAND_SPEED_HIGH", 0.4)
LAND_SPEED_SLOW   = num("LAND_SPEED_SLOW", 0.4)
LAND_SLOW_ALT     = num("LAND_SLOW_ALT", 0.3)
TD_ALT            = num("TD_ALT", 0.2)
ATL_SPOOLUP_MS    = num("ATL_SPOOLUP_MS", 3000.0)
ATL_SPOOLDOWN_MS  = num("ATL_SPOOLDOWN_MS", 3000.0)
TD_TIME_MS        = num("TD_TIME_MS", 1000.0)

# ---------------- 좌표변환 (Mode.cpp 포팅) ----------------
WGS84_A  = 6378137.0
WGS84_E2 = 6.69437999014e-3

def lla_to_ecef(lat_deg, lon_deg, h):
    lat = math.radians(lat_deg); lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    return ((N + h) * cl * co, (N + h) * cl * so, (N * (1 - WGS84_E2) + h) * sl)

class Origin:
    def __init__(self, lat, lon, h):
        self.lat, self.lon = math.radians(lat), math.radians(lon)
        self.sl, self.cl = math.sin(self.lat), math.cos(self.lat)
        self.so, self.co = math.sin(self.lon), math.cos(self.lon)
        self.X0, self.Y0, self.Z0 = lla_to_ecef(lat, lon, h)
    def to_ned(self, lat, lon, h=0.0):
        X, Y, Z = lla_to_ecef(lat, lon, h)
        dX, dY, dZ = X - self.X0, Y - self.Y0, Z - self.Z0
        n = -self.sl * self.co * dX - self.sl * self.so * dY + self.cl * dZ
        e = -self.so * dX + self.co * dY
        return n, e

def wrap_pi(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

# ---------------- 미션 파일 로드 ----------------
def load_mission(path):
    wps = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) >= 4:
                wps.append((float(parts[0]), float(parts[1]),
                            float(parts[2]), int(float(parts[3]))))
            elif len(parts) == 3:
                wps.append((float(parts[0]), float(parts[1]),
                            float(parts[2]), 0))
            elif len(parts) == 2:
                wps.append((float(parts[0]), float(parts[1]), 1.5, 0))
    return wps

# ---------------- PNG guidance (Mode.cpp png_compute_guidance 미러) ----------------
class Guidance:
    def __init__(self, wp_n, wp_e, wp_alt, wp_flags, seg_len):
        self.n, self.e, self.alt, self.flags, self.seg = wp_n, wp_e, wp_alt, wp_flags, seg_len
        self.idx = 0
        self.prev_idx = -1
        self.is_end = False
        self.alt_prev = 0.0
        self.orbit_active = False
        self.orbit_cN = self.orbit_cE = 0.0
        self.orbit_r = ORBIT_R_MIN
        self.orbit_turn = 0.0
        self.orbit_yawprev = 0.0
        self.orbit_eta_cmd = math.pi * 0.5  # 고정 좌회전(CCW)

    def step(self, px, py, yaw, alt_now, v_fwd):
        out = {"orbit_active": self.orbit_active}
        n = len(self.n)

        if self.orbit_active:
            dcN, dcE = self.orbit_cN - px, self.orbit_cE - py
            los_c = math.atan2(dcE, dcN)
            eta = wrap_pi(los_c - yaw)
            u_cmd = PNG_VX_MS
            r_den = max(self.orbit_r, ORBIT_R_MIN)
            v_law = u_cmd
            yaw_rate = (-(v_law / r_den) * math.sin(self.orbit_eta_cmd)
                        - ORBIT_K2 * (v_law / r_den) * math.sin(eta - self.orbit_eta_cmd))
            # 고도 ramp: 선회 누적 회전각 비례로 png_alt_prev -> 다음 WP 고도 보간
            # (펌웨어 Mode.cpp orbit 분기와 동일). 점프 제거.
            oprog = (self.orbit_turn / ORBIT_EXIT_TURN) if ORBIT_EXIT_TURN > 0 else 1.0
            oprog = max(0.0, min(1.0, oprog))
            alt_cmd = self.alt_prev + (self.alt[self.idx] - self.alt_prev) * oprog
            self.orbit_turn += abs(wrap_pi(yaw - self.orbit_yawprev))
            self.orbit_yawprev = yaw
            dnN, dnE = self.n[self.idx] - px, self.e[self.idx] - py
            if math.hypot(dnN, dnE) < PNG_WP_CAPTURE_R or self.orbit_turn >= ORBIT_EXIT_TURN:
                self.orbit_active = False
                # 선회 종료 시 현재 실고도로 직진 ramp 재앵커 (Mode.cpp와 동일).
                # 선회가 고도 ramp 완료 전에 끝나도(예: 3m 목표인데 2.76m),
                # 직진 ramp가 현재고도에서 같은 목표로 이어 올라가 역행(뚝 떨어짐) 제거.
                self.alt_prev = alt_now
            out.update(yaw_rate=yaw_rate, u_cmd=u_cmd, rng=math.hypot(dcN, dcE),
                       eta=eta, alt_cmd=alt_cmd, orbit_active=self.orbit_active)
            return out

        dN, dE = self.n[self.idx] - px, self.e[self.idx] - py
        rng = math.hypot(dN, dE)
        los = math.atan2(dE, dN)
        eta = wrap_pi(los - yaw)
        rng_den = max(rng, PNG_RNG_MIN)
        yaw_rate = 0.0 if self.is_end else PNG_KP * v_fwd * math.sin(eta) / rng_den
        u_cmd = 0.0 if self.is_end else PNG_VX_MS

        seg = self.seg[self.idx]
        progress = max(0.0, min(1.0, 1.0 - rng / seg))
        alt_tgt = self.alt[self.idx]
        alt_cmd = alt_tgt if self.is_end else self.alt_prev + (alt_tgt - self.alt_prev) * progress

        if (not self.is_end) and rng < PNG_WP_CAPTURE_R:
            reached = self.idx
            self.alt_prev = alt_now
            self.prev_idx = self.idx
            self.idx += 1
            if self.idx >= n:
                self.idx = n - 1
                self.is_end = True
            if (self.flags[reached] & WP_FLAG_ORBIT) and not self.is_end:
                aN, aE = self.n[reached], self.e[reached]
                bN, bE = self.n[self.idx], self.e[self.idx]
                self.orbit_cN = 0.5 * (aN + bN)
                self.orbit_cE = 0.5 * (aE + bE)
                r = 0.5 * math.hypot(bN - aN, bE - aE)
                self.orbit_r = max(r, ORBIT_R_MIN)
                self.orbit_eta_cmd = math.pi * 0.5
                self.orbit_yawprev = yaw
                self.orbit_turn = 0.0
                self.orbit_active = True
                out["orbit_active"] = True

        out.update(yaw_rate=yaw_rate, u_cmd=u_cmd, rng=rng, eta=eta, alt_cmd=alt_cmd)
        return out

# ---------------- 시뮬 본체 ----------------
def simulate(mission_path):
    wps = load_mission(mission_path)
    print(f"=== MISSION SIM: {os.path.basename(mission_path)} ===")
    print(f"WP 개수: {len(wps)}\n")

    # 원점 = WP0 근처 가정(이륙지점). 실제론 이륙 GNSS fix가 원점.
    # 펌웨어와 동일하게: 원점을 WP0보다 약간 남쪽에 두어 origin->WP0 첫구간 생성.
    o_lat, o_lon = wps[0][0] - 0.00005, wps[0][1]
    origin = Origin(o_lat, o_lon, 0.0)

    wp_n, wp_e, wp_alt, wp_flags = [], [], [], []
    for (la, lo, al, fl) in wps:
        n, e = origin.to_ned(la, lo)
        wp_n.append(n); wp_e.append(e); wp_alt.append(al); wp_flags.append(fl)

    # seg_len 사전계산 (enter_mode 미러): WP0=원점 기준
    seg_len = []
    for i in range(len(wps)):
        pn = 0.0 if i == 0 else wp_n[i - 1]
        pe = 0.0 if i == 0 else wp_e[i - 1]
        s = math.hypot(wp_n[i] - pn, wp_e[i] - pe)
        seg_len.append(max(s, PNG_RNG_MIN))

    print("WP    NED(N,E)[m]        alt[m]  flags  seg_len[m]")
    for i in range(len(wps)):
        fl = wp_flags[i]
        tag = "AVOID" if fl & WP_FLAG_AVOID else ("ORBIT" if fl & WP_FLAG_ORBIT else "-")
        print(f" {i}   ({wp_n[i]:7.2f},{wp_e[i]:7.2f})   {wp_alt[i]:.2f}   {fl}({tag:5}) {seg_len[i]:6.2f}")
    print()

    g = Guidance(wp_n, wp_e, wp_alt, wp_flags, seg_len)

    # 상태
    state = "GROUND"
    px = py = 0.0
    yaw = 0.0
    alt = 0.0           # up+
    hold_h = 0.0
    t = 0.0
    phase_start = 0.0
    td_start = None
    guidance_armed = False
    alt_cmd_prev = None

    events = []
    def log(msg): events.append(f"t={t:6.2f}s [{state:10}] {msg}")

    # 선회 진단: 진입 시 중심이 기체 왼쪽/오른쪽인지 + 선회 중 반경 이탈 추적
    orbit_diag = []      # (entry_wp, side_ok, r_cmd, r_max_dev)
    orbit_cur = None     # {"r":..., "cN":..., "cE":..., "rmin":..., "rmax":...}

    log("GROUND, 이륙 명령(t) 대기 -> 자동 발행")
    state = "SPOOLUP"; phase_start = t

    MAX_T = 600.0
    last_state = state
    seg_print = -1
    while t < MAX_T:
        # ---- 수직 + 상태머신 (atl_sequencer_step 미러) ----
        if state == "SPOOLUP":
            if (t - phase_start) * 1000.0 >= ATL_SPOOLUP_MS:
                state = "TAKEOFF"
        elif state == "TAKEOFF":
            alt += TKO_CLIMB_RATE * DT
            if alt >= TKO_TARGET_ALT:
                hold_h = alt
                state = "HOLD"
                log(f"이륙 완료 alt={alt:.2f} -> HOLD. 미션 시작(m) 자동 발행")
                state = "GUIDANCE"  # 즉시 미션 시작
                guidance_armed = False
        elif state == "GUIDANCE":
            if not guidance_armed:
                g.alt_prev = hold_h
                guidance_armed = True
            # 수평 진행: 전진속도 PNG_VX, yaw_rate 적용
            res = g.step(px, py, yaw, alt, PNG_VX_MS)
            yaw = wrap_pi(yaw + max(-RC_MAX_YAW_RATE, min(RC_MAX_YAW_RATE, res["yaw_rate"])) * DT)
            px += res["u_cmd"] * math.cos(yaw) * DT
            py += res["u_cmd"] * math.sin(yaw) * DT
            # 고도 ramp 추종 (1차, climb-rate 제한)
            # alt_cmd 순간 점프 감지 (한 스텝에 큰 계단 = 버그). 정상 ramp는
            # 스텝당 변화가 작아야 한다.
            if alt_cmd_prev is not None:
                d_altcmd = abs(res["alt_cmd"] - alt_cmd_prev)
                if d_altcmd > 0.10:
                    log(f"!! alt_cmd 점프 감지: {alt_cmd_prev:.2f} -> {res['alt_cmd']:.2f} "
                        f"(스텝당 +{d_altcmd:.2f}m, WP{g.idx} orbit={res['orbit_active']})")
            alt_cmd_prev = res["alt_cmd"]
            hold_h = res["alt_cmd"]
            dz = hold_h - alt
            alt += max(-LAND_SPEED_HIGH, min(TKO_CLIMB_RATE, dz / DT * 0.1)) * DT
            # 고도 시계열 진단 (0.5초마다): alt_cmd 궤적 추적
            if int(t * 2) != int((t - DT) * 2):
                mode = "ORBIT" if res["orbit_active"] else "PNG"
                log(f"  [alt] cmd={res['alt_cmd']:.2f} actual={alt:.2f} "
                    f"wp={g.idx} {mode} alt_prev={g.alt_prev:.2f}")
            # 세그먼트 진입 로그
            if g.idx != seg_print:
                seg_print = g.idx
                fl = wp_flags[g.prev_idx] if g.prev_idx >= 0 else 0
                avoid = (not res["orbit_active"]) and g.prev_idx >= 0 and (fl & WP_FLAG_AVOID)
                log(f"-> WP{g.idx} 추적 (prev={g.prev_idx}, "
                    f"avoid={'ON' if avoid else 'off'}, orbit={'ON' if res['orbit_active'] else 'off'}, "
                    f"alt_cmd={res['alt_cmd']:.2f})")
            if res["orbit_active"] and state == "GUIDANCE":
                pass
            if g.is_end:
                log(f"최종 WP 도달 (px={px:.1f},py={py:.1f},alt={alt:.2f}) -> LAND")
                state = "LAND"
        elif state == "LAND":
            alt -= LAND_SPEED_HIGH * DT
            if alt < LAND_SLOW_ALT:
                state = "LAND_SLOW"; td_start = None
                log(f"저고도 alt={alt:.2f} -> LAND_SLOW (착륙/비전)")
        elif state == "LAND_SLOW":
            alt -= LAND_SPEED_SLOW * DT
            if alt < 0: alt = 0.0
            td = (alt < TD_ALT)
            if td:
                if td_start is None: td_start = t
                if (t - td_start) * 1000.0 >= TD_TIME_MS:
                    state = "SPOOLDOWN"; phase_start = t
                    log("터치다운 감지 -> SPOOLDOWN")
            else:
                td_start = None
        elif state == "SPOOLDOWN":
            if (t - phase_start) * 1000.0 >= ATL_SPOOLDOWN_MS:
                state = "DISARMED"
                log("스풀다운 완료 -> DISARMED (모터 정지). 미션 종료")
                break

        # orbit 진입/종료 상태 추적 로그 + 기하 진단
        if state == "GUIDANCE":
            cur = "ORBIT" if g.orbit_active else "PNG"
            if cur != last_state:
                if g.orbit_active:
                    # 진입 순간 중심이 기체 왼쪽인지: eta0 = wrap_pi(LOS_to_center - yaw) >= 0 이면 왼쪽
                    los_c = math.atan2(g.orbit_cE - py, g.orbit_cN - px)
                    eta0 = wrap_pi(los_c - yaw)
                    side_ok = (eta0 >= 0.0)   # 고정 CCW는 중심이 왼쪽이어야 정상
                    log(f"선회(ORBIT) 진입: 중심NED=({g.orbit_cN:.2f},{g.orbit_cE:.2f}) "
                        f"r={g.orbit_r:.2f}m eta0={math.degrees(eta0):+.0f}deg "
                        f"중심={'왼쪽(정상)' if side_ok else '오른쪽(주의!)'}")
                    orbit_cur = {"wp": g.idx, "r": g.orbit_r, "cN": g.orbit_cN,
                                 "cE": g.orbit_cE, "side_ok": side_ok,
                                 "rmin": 1e9, "rmax": 0.0}
                else:
                    if orbit_cur:
                        dev = max(orbit_cur["rmax"] - orbit_cur["r"],
                                  orbit_cur["r"] - orbit_cur["rmin"])
                        orbit_diag.append((orbit_cur["wp"], orbit_cur["side_ok"],
                                           orbit_cur["r"], dev))
                        log(f"선회 종료 -> PNG 직진 복귀 "
                            f"(반경 {orbit_cur['rmin']:.2f}~{orbit_cur['rmax']:.2f}m, "
                            f"목표 {orbit_cur['r']:.2f}m)")
                        orbit_cur = None
                last_state = cur
            # 선회 중 실제 반경 추적
            if g.orbit_active and orbit_cur:
                rr = math.hypot(g.orbit_cN - px, g.orbit_cE - py)
                orbit_cur["rmin"] = min(orbit_cur["rmin"], rr)
                orbit_cur["rmax"] = max(orbit_cur["rmax"], rr)

        t += DT

    print("---- 시퀀스 이벤트 로그 ----")
    for e in events:
        print(e)
    print()

    if orbit_diag:
        print("---- 선회(ORBIT) 기하 진단 ----")
        all_ok = True
        for (wp, side_ok, r_cmd, dev) in orbit_diag:
            radius_ok = dev < 0.5 * r_cmd   # 반경 이탈이 목표의 50% 미만이면 수렴 양호
            flag = "OK" if (side_ok and radius_ok) else "확인필요"
            if not (side_ok and radius_ok):
                all_ok = False
            print(f"  WP{wp} 진입 선회: 중심={'왼쪽' if side_ok else '오른쪽'}, "
                  f"목표r={r_cmd:.2f}m, 반경이탈={dev:.2f}m -> [{flag}]")
        if not all_ok:
            print("  ! 중심이 오른쪽이거나 반경 이탈이 크면, 해당 WP에서 다음 WP가")
            print("    진행방향 기준 왼쪽에 오도록 미션 배치를 수정하세요(고정 CCW).")
        print()
    if state == "DISARMED":
        print(f"[OK] 미션 정상 종료. 총 비행시간 {t:.1f}s, 최종위치 NED=({px:.1f},{py:.1f})")
        return 0
    else:
        print(f"[FAIL] {MAX_T}s 내 미완료. 마지막 상태={state}, WP idx={g.idx}, pos=({px:.1f},{py:.1f})")
        return 1

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(__file__), "..", "..", "..",
                     "GCS", "WayPoint", "mission_subpc_Mission_2.txt")
    sys.exit(simulate(path))
