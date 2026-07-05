// ============================================================
// AVC GCS - Ground Control Station
// Windows C++ / Win32 Serial + Winsock NTRIP
//
// 빌드: cmake --build build
// 사용: GCS.exe COM13 [--ntrip]
// ============================================================

#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN

#define FLIGHT_LOG   (1)

#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <conio.h>
#include <string>
#include <stdlib.h>
#include <iomanip>

#include "serial_port.h"
#include "p8_comm.h"
#include "ntrip_client.h"
#include "RealTime.h"
#include "OptiTrack.h"
#include "dashboard.h"
#include <fstream>
#include <time.h>
#include <vector>
#include <string>

#pragma comment(lib, "ws2_32.lib")

static std::ofstream flight_log;
static std::ofstream debug_log;

// UDP forwarding socket (global for keyboard handler access)
static SOCKET udp_sock = INVALID_SOCKET;
static struct sockaddr_in udp_dst = {};

// UDP receive socket for sub-PC mission uploads (mav_bridge.py -> GCS)
static SOCKET udp_recv_sock = INVALID_SOCKET;

// Last good fused lat/lon, used to backfill DebugFrame->JSON when png_lat is 0.
// Updated by BOTH the TelemFrame and DebugFrame paths so the sub-PC always has
// a valid position even before a control mode starts producing DebugFrames.
static double g_last_lat = 0.0;
static double g_last_lon = 0.0;
static float  g_last_alt = 0.0f;

// Sub-PC telemetry policy (C): DebugFrame is the preferred source (richer), but
// it only flows in a control mode with debug ON. TelemFrame always flows (even
// in IDLE), so it is the fallback. When a DebugFrame arrived within this window
// the TelemFrame JSON path is skipped to avoid the two sources fighting over
// the same UDP keys; when DebugFrames stop, TelemFrame takes over automatically.
static uint32_t g_last_debug_json_ms = 0;
#define DEBUG_JSON_FRESH_MS  500   // TelemFrame JSON suppressed while DebugFrame fresh

static void save_telem_csv(const TelemFrame* tf, const OptiTrackData_t* ot);
static void save_debug_csv(const DebugFrame* df);
static void print_user_debug(const TelemFrame* tf);
static void print_help();
static void handle_keyboard();
static void send_debug_json(const DebugFrame* df);
static void poll_mission_udp();

// ============================================================
// Sub-PC mission receiver
// Parses P8 mission frames (0x11/0x12/0x13) arriving over UDP and saves the
// waypoints to a timestamped WayPoint/*.txt file. Does NOT forward to the MCU.
// ============================================================
struct MissionRx {
    enum { S_SYNC1, S_SYNC2, S_CMD, S_LEN_H, S_LEN_L, S_PAYLOAD, S_CRC } state = S_SYNC1;
    uint8_t  cmd = 0;
    uint16_t len = 0;
    uint16_t idx = 0;
    uint8_t  crc = 0;
    uint8_t  payload[64];

    // accumulated mission
    int      expected = 0;
    int      count = 0;
    double   lat[PNG_MAX_WAYPOINTS];
    double   lon[PNG_MAX_WAYPOINTS];
    float    alt[PNG_MAX_WAYPOINTS];   // per-WP altitude [m] from CMD_MISSION_ITEM alt_mm
};
static MissionRx g_mission;

static void save_mission_txt(const double* lat, const double* lon,
                             const float* alt, int n);

static void mission_handle_frame(MissionRx& m)
{
    switch (m.cmd) {
    case CMD_MISSION_COUNT:
        m.expected = (m.len >= 1) ? m.payload[0] : 0;
        m.count = 0;
        printf("\n[MISSION] start, expecting %d waypoints\n", m.expected);
        break;

    case CMD_MISSION_ITEM:
        // <B i i i> = idx, lat_1e7, lon_1e7, alt_mm  (alt now kept -> .txt -> MCU)
        if (m.len >= 13 && m.count < PNG_MAX_WAYPOINTS) {
            int32_t lat_1e7, lon_1e7, alt_mm;
            memcpy(&lat_1e7, m.payload + 1, 4);
            memcpy(&lon_1e7, m.payload + 5, 4);
            memcpy(&alt_mm,  m.payload + 9, 4);
            m.lat[m.count] = lat_1e7 * 1e-7;
            m.lon[m.count] = lon_1e7 * 1e-7;
            m.alt[m.count] = alt_mm * 1e-3f;   // mm -> m (takeoff-ground-relative)
            m.count++;
        }
        break;

    case CMD_MISSION_START:
        if (m.count > 0) {
            save_mission_txt(m.lat, m.lon, m.alt, m.count);
        } else {
            printf("[MISSION] START received but no items accumulated\n");
        }
        m.expected = 0;
        m.count = 0;
        break;

    case CMD_MISSION_CLEAR:
        m.expected = 0;
        m.count = 0;
        printf("[MISSION] cleared\n");
        break;

    default:
        break;  // ignore other CMD ids on the mission port
    }
}

// Feed one received byte into the lightweight uplink-CMD parser.
static void mission_feed(MissionRx& m, uint8_t b)
{
    switch (m.state) {
    case MissionRx::S_SYNC1:
        if (b == P8_SYNC1) m.state = MissionRx::S_SYNC2;
        break;
    case MissionRx::S_SYNC2:
        m.state = (b == P8_SYNC2) ? MissionRx::S_CMD : MissionRx::S_SYNC1;
        break;
    case MissionRx::S_CMD:
        m.cmd = b; m.crc = b; m.state = MissionRx::S_LEN_H;
        break;
    case MissionRx::S_LEN_H:
        m.len = (uint16_t)b << 8; m.crc ^= b; m.state = MissionRx::S_LEN_L;
        break;
    case MissionRx::S_LEN_L:
        m.len |= b; m.crc ^= b; m.idx = 0;
        if (m.len > sizeof(m.payload)) { m.state = MissionRx::S_SYNC1; break; }
        m.state = (m.len == 0) ? MissionRx::S_CRC : MissionRx::S_PAYLOAD;
        break;
    case MissionRx::S_PAYLOAD:
        m.payload[m.idx++] = b; m.crc ^= b;
        if (m.idx >= m.len) m.state = MissionRx::S_CRC;
        break;
    case MissionRx::S_CRC:
        if (b == m.crc) mission_handle_frame(m);
        m.state = MissionRx::S_SYNC1;
        break;
    }
}

// ============================================================
// mainq
// ============================================================
int main(int argc, char* argv[])
{
    flight_log.open("../data/telemetry_dummy.csv");
    flight_log << "time_s,imu_roll,imu_pitch,imu_yaw,imu_yaw_rate,opti_roll,opti_pitch,opti_yaw\n";

    debug_log.open("../data/test.csv");
    // debug_log.open("../data/test.csv");
    // Column order mirrors the DebugFrame layout: attitude -> altitude ->
    // velocity -> PNG -> misc.
    debug_log << // Group 1: attitude controller
                 "roll,pitch,yaw,p,q,r,"
                 "roll_cmd,pitch_cmd,r_cmd,"
                 "roll_rate_cmd,pitch_rate_cmd,p_f,q_f,"
                 "e_roll,e_pitch,int_e_roll,int_e_pitch,roll_ff,pitch_ff,"
                 "e_r,int_e_r,I_r,"
                 "U1,U2,U3,U4,"
                 // Group 2: altitude controller (baro/CF fields removed)
                 "throttle_N,"
                 "lidar_alt,lidar_valid,"
                 "alt_hold,alt_cmd,alt_error,hdot_cmd,DeltaT_cmd,"
                 "h_fmf,v_fmf,a_fmf,h_used,v_used,"
                 // Group 3: velocity controller (X->u, Y->v)
                 // u_acc_f/v_acc_f = 12Hz-LPF filtered accel feedback (the value
                 // actually used in the -kd*a_f damping term), not raw free accel.
                 "u_cmd,u_fb,u_acc_f,u_err,v_cmd,v_fb,v_acc_f,v_err,"
                 // Group 4: PNG guidance
                 "png_px,png_py,png_rng,png_eta,png_los,png_yaw_rate,wp_idx,"
                 "png_lat,png_lon,png_alt,"
                 // Group 4b: collision-cone obstacle avoidance
                 "obs_r_ca,obs_Ro,obs_eo,obs_gamma,obs_theta_dot,obs_avoid,"
                 // Group 5: misc (timing, battery, GNSS/RTK)
                 "time_s,batt_v,loop_count_50hz,loop_dt_max_ms,"
                 "rtk_status,gnss_fix,vel_n,vel_e,vel_u,vel_v,"
                 // Group 6: auto takeoff/landing
                 "atl_state,atl_climb_cmd,atl_hold_alt,atl_phase_ms,"
                 // Group 7: vision precision landing
                 "helipad_x,helipad_y,helipad_xy,helipad_gnd,helipad_valid,"
                 "helipad_dn,helipad_de,"
                 "vlatch_n,vlatch_e,vision_use,vlatch_set,vision_reject,"
                 // Group 8: MISSION waypoint altitude command
                 "wp_alt_cmd,"
                 // Group 9: MISSION waypoint local-NED cache
                 "wp_ned_count";
    for (int i = 0; i < PNG_MAX_WAYPOINTS; i++) debug_log << ",wp_north_" << i;
    for (int i = 0; i < PNG_MAX_WAYPOINTS; i++) debug_log << ",wp_east_"  << i;
    debug_log << "\n";

    // --- 인자 파싱 ---
    // 사용: GCS.exe [COMx] [--ip <ip>] [--tx-port <p>] [--rx-port <p>]
    std::string port_name;
    std::string sub_pc_ip = "172.20.10.3";   // sub-PC (visualization/mission planner, static on phone hotspot)
    int tx_port = 5005;                       // GCS -> sub-PC telemetry JSON
    int rx_port = 5006;                       // sub-PC -> GCS mission upload

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--ip") == 0 && i + 1 < argc) {
            sub_pc_ip = argv[++i];
        } else if (strcmp(argv[i], "--tx-port") == 0 && i + 1 < argc) {
            tx_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--rx-port") == 0 && i + 1 < argc) {
            rx_port = atoi(argv[++i]);
        } else if (argv[i][0] != '-') {
            port_name = argv[i];  // first non-flag arg is the COM port
        }
    }

    if (port_name.empty()) {
        char buf[32];
        printf("Enter COM port (e.g. COM11): ");
        if (!fgets(buf, sizeof(buf), stdin)) return 1;
        buf[strcspn(buf, "\r\n")] = '\0';
        if (strlen(buf) == 0) { printf("[ERR] No port.\n"); return 1; }
        port_name = buf;
    }

    // --- 초기화 ---
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    if (!serial_open(port_name.c_str(), 460800)) {
        printf("[ERR] Cannot open %s\n", port_name.c_str());
        WSACleanup();
        return 1;
    }
    printf("[GCS] Serial: %s @ 460800\n", port_name.c_str());

    if (ntrip_cfg.enabled && !ntrip_connect())
        printf("[NTRIP] Initial connection failed. Press 'n' to retry.\n");

    print_help();

    // OptiTrack 초기화
    if (!initOptiTrack())
        printf("[OptiTrack] Init failed. Continuing without OptiTrack.\n");

    // UDP telemetry forwarding to sub-PC (TelemFrame/DebugFrame -> JSON)
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock == INVALID_SOCKET) {
        printf("[UDP] tx socket() failed\n");
    } else {
        udp_dst.sin_family = AF_INET;
        udp_dst.sin_port = htons((u_short)tx_port);
        udp_dst.sin_addr.s_addr = inet_addr(sub_pc_ip.c_str());
        printf("[UDP] TX telemetry -> %s:%d\n", sub_pc_ip.c_str(), tx_port);
    }

    // UDP mission receive socket (sub-PC mission planner -> GCS)
    udp_recv_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_recv_sock == INVALID_SOCKET) {
        printf("[UDP] rx socket() failed\n");
    } else {
        struct sockaddr_in rx_addr = {};
        rx_addr.sin_family = AF_INET;
        rx_addr.sin_port = htons((u_short)rx_port);
        rx_addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(udp_recv_sock, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) == SOCKET_ERROR) {
            printf("[UDP] rx bind(%d) failed\n", rx_port);
            closesocket(udp_recv_sock);
            udp_recv_sock = INVALID_SOCKET;
        } else {
            u_long nb = 1;
            ioctlsocket(udp_recv_sock, FIONBIO, &nb);  // non-blocking
            printf("[UDP] RX mission <- :%d\n", rx_port);
        }
    }

    OptiTrackData_t opti = {};

    // 실시간 디버그 대시보드 (별도 UI 스레드, 가시화 전용)
    dashboard_start();

    P8Parser parser;
    p8_parser_init(&parser);

    init_timer();
    int hb_counter = 0;
    const int HB_INTERVAL = (int)(1.0 * SAMPLING_FREQUENCY);  // 1초

    // --- 리얼타임 루프 ---
    while (1) {
        update_time();

        // OptiTrack 수신
        recvOptiTrack(&opti);

        // 0. 서브PC 미션 수신 (UDP -> txt 저장)
        poll_mission_udp();

        // 1. P8 수신 -> 파싱
        // RX 버퍼를 매 루프에서 완전히 비운다(전체 드레인). serial_read는 한 번에
        // sizeof(rx_buf)까지만 반환하므로, 가득 찼으면 한 번 더 읽어 OS RX 버퍼에
        // 백로그가 쌓이지 않게 한다. 백로그가 쌓이면 화면이 실시간보다 뒤처진다.
        uint8_t rx_buf[1024];
        int n;
        do {
            n = serial_read(rx_buf, sizeof(rx_buf));
            for (int i = 0; i < n; i++) {
                P8MsgType msg = p8_parser_feed(&parser, rx_buf[i]);

            if (msg == P8_MSG_TELEM) {
                const TelemFrame* tf = p8_get_telem(&parser);
                // 화면 출력만 ~10Hz로 제한(콘솔 printf가 느려 매 프레임 출력 시
                // 처리 루프가 출력에 묶여 RX 드레인이 막히고 지연이 누적된다).
                // CSV/대시보드/UDP/NTRIP은 아래에서 전 프레임 처리한다.
                {
                    static uint64_t last_print_ms = 0;
                    uint64_t now = GetTickCount64();
                    if (now - last_print_ms >= 100) { last_print_ms = now; print_telem(tf); }
                }
                // Flight Data Log (전 프레임 저장)
                if (FLIGHT_LOG) save_telem_csv(tf, &opti);

                print_user_debug(tf);

                // 실시간 대시보드 기본(폴백) 소스: TelemFrame 전달
                dashboard_push_telem(tf);

                // Cache last good fused position (used by the DebugFrame path
                // to backfill lat/lon when png_lat is 0).
                if (tf->lat_deg != 0.0 && tf->lon_deg != 0.0) {
                    g_last_lat = tf->lat_deg;
                    g_last_lon = tf->lon_deg;
                    g_last_alt = tf->alt_m;
                }

                // Forward telemetry as JSON over UDP (policy C: fallback source).
                // Skip while a DebugFrame arrived recently so the richer
                // DebugFrame source owns the same UDP keys; resume automatically
                // when DebugFrames stop (debug OFF / left a control mode / IDLE).
                bool debug_fresh =
                    (uint32_t)(GetTickCount64() - g_last_debug_json_ms) < DEBUG_JSON_FRESH_MS;
                if (udp_sock != INVALID_SOCKET && !debug_fresh) {
                    char ubuf[512];
                    int ulen = snprintf(ubuf, sizeof(ubuf),
                        "{\"time\":%.3f,\"lat\":%.8f,\"lon\":%.8f,\"alt\":%.3f,"
                        "\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,\"yaw_rate\":%.3f,"
                        "\"vel_n\":%.3f,\"vel_e\":%.3f,\"vel_u\":%.3f,"
                        "\"batt_mv\":%d}",
                        (float)(tf->uptime_ms / 1000.0),
                        (float)tf->lat_deg, (float)tf->lon_deg, (float)tf->alt_m,
                        (float)tf->roll_deg, (float)tf->pitch_deg, (float)tf->yaw_deg,
                        (float)tf->yaw_rate_dps,
                        (float)tf->vel_n_ms, (float)tf->vel_e_ms, (float)tf->vel_u_ms,
                        (int)tf->batt1_mv);
                    sendto(udp_sock, ubuf, ulen, 0,
                           (struct sockaddr*)&udp_dst, sizeof(udp_dst));
                }

                // Update NTRIP GGA with live position when GNSS has accurate fix
                // Reject if lat/lon are rounded to integer (MTi not converged yet)
                if ((tf->status_flags & STATUS_BIT_GNSS_FIX) &&
                    tf->lat_deg != 0.0 && tf->lon_deg != 0.0 &&
                    (tf->lat_deg - (int)tf->lat_deg) != 0.0)
                    ntrip_update_position(tf->lat_deg, tf->lon_deg);
            }
            else if (msg == P8_MSG_DEBUG) {
                const DebugFrame* df = p8_get_debug(&parser);
                // 화면 출력만 ~10Hz로 제한(콘솔 병목 -> RX 드레인 지연 방지).
                // CSV/대시보드/UDP는 아래에서 전 프레임 처리한다.
                {
                    static uint64_t last_print_ms = 0;
                    uint64_t now = GetTickCount64();
                    if (now - last_print_ms >= 100) { last_print_ms = now; print_debug(df); }
                }
                // CSV 저장 (전 프레임)
                if (FLIGHT_LOG) save_debug_csv(df);

                // 서브PC로 텔레메트리 JSON 송신 (DebugFrame 소스, 단위 변환)
                send_debug_json(df);

                // 실시간 대시보드로 최신 프레임 전달
                dashboard_push(df);
            }
            else if (msg == P8_MSG_ACK) {

                if (parser.ack_payload_len >= 2) {
                    print_ack(parser.ack_payload[0], parser.ack_payload[1]);

                    // Extended diagnostic for HW_FAIL
                    if (parser.ack_payload[1] == ACK_ERR_HW_FAIL &&
                        parser.ack_payload_len >= 5) {
                        uint16_t flush = ((uint16_t)parser.ack_payload[2] << 8) |
                                          parser.ack_payload[3];
                        uint8_t step = parser.ack_payload[4];
                        const char* step_str = "?";
                        switch (step) {
                        case 1: step_str = "GoToConfig"; break;
                        case 2: step_str = "SetOutputConfig"; break;
                        case 3: step_str = "GoToMeasurement"; break;
                        }
                        printf("[IMU_DIAG] flush_bytes=%d fail_at=%s(%d)\n",
                               flush, step_str, step);
                    }
                }
            }
            } // for (i)
            // 버퍼가 가득 찼으면 RX에 더 남아있다는 뜻 -> 한 번 더 드레인.
        } while (n == (int)sizeof(rx_buf));

        // 2. 키보드
        handle_keyboard();

        // 3. NTRIP
        if (ntrip_cfg.enabled) ntrip_poll();

        // 4. 하트비트 (10초)
        if (++hb_counter >= HB_INTERVAL) {
            hb_counter = 0;
            p8_send_heartbeat();
        }

        time_idling();
    }
}

// ============================================================
// Save accumulated mission to a timestamped WayPoint/*.txt (lat lon alt).
// Format matches what the 'w' loader reads (3-column lat lon alt is parsed).
// ============================================================
static void save_mission_txt(const double* lat, const double* lon,
                             const float* alt, int n)
{
    time_t t = time(nullptr);
    struct tm* lt = localtime(&t);
    char fname[256];
    snprintf(fname, sizeof(fname),
             "../WayPoint/mission_subpc_%04d%02d%02d_%02d%02d%02d.txt",
             lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
             lt->tm_hour, lt->tm_min, lt->tm_sec);

    FILE* f = fopen(fname, "w");
    if (!f) {
        printf("[MISSION] ERROR: cannot write %s\n", fname);
        return;
    }
    char stamp[64];
    snprintf(stamp, sizeof(stamp), "%04d-%02d-%02d %02d:%02d:%02d",
             lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
             lt->tm_hour, lt->tm_min, lt->tm_sec);
    fprintf(f, "# mission from subPC %s (lat lon alt; deg deg m)\n", stamp);
    for (int i = 0; i < n; i++)
        fprintf(f, "%.7f %.7f %.2f\n", lat[i], lon[i], alt[i]);
    fclose(f);

    printf("[MISSION] saved %d wp -> %s\n", n, fname);
}

// ============================================================
// Drain the UDP mission socket and feed bytes into the mission parser.
// ============================================================
static void poll_mission_udp()
{
    if (udp_recv_sock == INVALID_SOCKET) return;
    uint8_t buf[1024];
    for (;;) {
        int n = recvfrom(udp_recv_sock, (char*)buf, sizeof(buf), 0, nullptr, nullptr);
        if (n <= 0) break;  // WSAEWOULDBLOCK or no data
        for (int i = 0; i < n; i++)
            mission_feed(g_mission, buf[i]);
    }
}

// ============================================================
// Forward DebugFrame to sub-PC as the same JSON the TelemFrame path sends.
// Keys/format match mav_bridge.py (sub-PC unchanged); values come from the
// DebugFrame with unit conversion (rad->deg). lat/lon use png_lat/png_lon;
// when those are 0 (outside PNG/fix loss) the last good fused value is reused.
// ============================================================
static void send_debug_json(const DebugFrame* df)
{
    if (udp_sock == INVALID_SOCKET) return;

    const float R2D = 57.29578f;

    double lat = df->png_lat;
    double lon = df->png_lon;
    float  alt = df->png_alt;
    if (lat != 0.0 && lon != 0.0) {
        g_last_lat = lat; g_last_lon = lon; g_last_alt = alt;
    } else {
        lat = g_last_lat; lon = g_last_lon; alt = g_last_alt;
    }

    char ubuf[512];
    int ulen = snprintf(ubuf, sizeof(ubuf),
        "{\"time\":%.3f,\"lat\":%.8f,\"lon\":%.8f,\"alt\":%.3f,"
        "\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,\"yaw_rate\":%.3f,"
        "\"vel_n\":%.3f,\"vel_e\":%.3f,\"vel_u\":%.3f,"
        "\"batt_mv\":%d}",
        (float)(df->uptime_ms / 1000.0),
        lat, lon, (float)alt,
        df->roll * R2D, df->pitch * R2D, df->yaw * R2D,
        df->r * R2D,
        // w_down (CF vertical speed) was removed from DebugFrame; use v_used
        // (vertical-speed feedback actually used by alt-hold, up+) as vel_u.
        df->vel_n, df->vel_e, df->v_used,
        (int)df->batt_mv);
    sendto(udp_sock, ubuf, ulen, 0,
           (struct sockaddr*)&udp_dst, sizeof(udp_dst));

    // Mark DebugFrame as the active source so the TelemFrame path defers.
    g_last_debug_json_ms = (uint32_t)GetTickCount64();
}

static void save_telem_csv(const TelemFrame* tf,
                           const OptiTrackData_t* ot)
{
    flight_log
        << tf->uptime_ms / 1000.0 << ","
        << tf->roll_deg << ","
        << tf->pitch_deg << ","
        << tf->yaw_deg << ","
        << tf->yaw_rate_dps << ","
        << ot->roll << ","
        << ot->pitch << ","
        << ot->yaw
        << "\n";
}

// ============================================================
// Debug control CSV logger
// ============================================================
static void save_debug_csv(const DebugFrame* df)
{
    debug_log
        // Group 1: attitude controller
        << df->roll << "," << df->pitch << "," << df->yaw << ","
        << df->p << "," << df->q << "," << df->r << ","
        << df->roll_cmd << "," << df->pitch_cmd << "," << df->r_cmd << ","
        << df->roll_rate_cmd << "," << df->pitch_rate_cmd << ","
        << df->p_f << "," << df->q_f << ","
        << df->e_roll << "," << df->e_pitch << ","
        << df->int_e_roll << "," << df->int_e_pitch << ","
        << df->roll_ff << "," << df->pitch_ff << ","
        << df->e_r << "," << df->int_e_r << "," << df->I_r << ","
        << df->U1 << "," << df->U2 << "," << df->U3 << "," << df->U4 << ","
        // Group 2: altitude controller (baro/CF fields removed)
        << df->throttle_N << ","
        << df->lidar_alt << "," << (int)df->lidar_valid << ","
        << (int)df->alt_hold << "," << df->alt_cmd << "," << df->alt_error << ","
        << df->hdot_cmd << "," << df->DeltaT_cmd << ","
        << df->h_fmf << "," << df->v_fmf << "," << df->a_fmf << ","
        << df->h_used << "," << df->v_used << ","
        // Group 3: velocity controller (X->u, Y->v)
        << df->u_cmd << "," << df->u_fb << "," << df->u_acc << "," << df->u_err << ","
        << df->v_cmd << "," << df->v_fb << "," << df->v_acc << "," << df->v_err << ","
        // Group 4: PNG guidance
        << df->png_px << "," << df->png_py << "," << df->png_rng << ","
        << df->png_eta << "," << df->png_los << "," << df->png_yaw_rate << "," << (int)df->wp_idx << ","
        // png_lat/lon need ~9 decimals (1e-7 deg); restore default precision after.
        << std::setprecision(9) << df->png_lat << "," << df->png_lon << ","
        << std::setprecision(6) << df->png_alt << ","
        // Group 4b: collision-cone obstacle avoidance
        << df->obs_r_ca << "," << df->obs_Ro << "," << df->obs_eo << ","
        << df->obs_gamma << "," << df->obs_theta_dot << "," << (int)df->obs_avoid << ","
        // Group 5: misc (timing, battery, GNSS/RTK)
        << df->uptime_ms / 1000.0 << "," << df->batt_mv / 1000.0f << ","
        << df->loop_count_50hz << "," << df->loop_dt_max_ms << ","
        << (int)df->rtk_status << "," << (int)df->gnss_fix << ","
        << df->vel_n << "," << df->vel_e << "," << df->vel_u << "," << df->vel_v << ","
        // Group 6: auto takeoff/landing
        << (int)df->atl_state << "," << df->atl_climb_cmd << ","
        << df->atl_hold_alt << "," << (int)df->atl_phase_ms << ","
        // Group 7: vision precision landing
        << df->helipad_x << "," << df->helipad_y << ","
        << df->helipad_xy << "," << df->helipad_gnd << ","
        << (int)df->helipad_valid << ","
        << df->helipad_dn << "," << df->helipad_de << ","
        << df->vlatch_n << "," << df->vlatch_e << ","
        << (int)df->vision_use << "," << (int)df->vlatch_set << ","
        << (int)df->vision_reject << ","
        // Group 8: MISSION waypoint altitude command
        << df->wp_alt_cmd << ","
        // Group 9: MISSION waypoint local-NED cache
        << (int)df->wp_ned_count;
    for (int i = 0; i < PNG_MAX_WAYPOINTS; i++) debug_log << "," << df->wp_north[i];
    for (int i = 0; i < PNG_MAX_WAYPOINTS; i++) debug_log << "," << df->wp_east[i];
    debug_log << "\n";
}

// ============================================================
// User debug output (add your custom prints here)
// ============================================================
static void print_user_debug(const TelemFrame* tf)
{
    // 사용자 디버깅용 출력부
    // 필요한 데이터를 여기에 추가하면 됩니다.
    // 예시:
    // printf("[DEBUG] vel_e=%.2f vel_n=%.2f vel_u=%.2f\n",
    //        tf->vel_e_ms, tf->vel_n_ms, tf->vel_u_ms);
    // printf("[DEBUG] RC: %d %d %d %d %d\n",
    //        tf->rc_ch[0], tf->rc_ch[1], tf->rc_ch[2], tf->rc_ch[3], tf->rc_ch[4]);
    // printf("[DEBUG] Motor: %d %d %d %d\n",
    //        tf->motor_us[0], tf->motor_us[1], tf->motor_us[2], tf->motor_us[3]);
}

// ============================================================
// Help menu
// ============================================================
static void print_help()
{
    printf("\n=== AVC GCS ===\n");
    printf("  0~9  : Set mode (0=IDLE 1=RC 2=RTK 3=MISSION 4=MTI 5=ESC_CAL 6=VEL 7=PNG 8=POS 9=AUTO_TKO_LAND)\n");
    printf("         (3 is remapped to MISSION/mode 10; PWM_TEST uses the on-board console)\n");
    printf("  w    : Pick a WayPoint/*.txt mission and upload it\n");
    printf("  q    : Abort -> IDLE\n");
    printf("  h    : Heartbeat\n");
    printf("  n    : NTRIP toggle\n");
    printf("  g    : Toggle debug/normal output (MCU)\n");
    printf("  d    : PWM disarm\n");
    printf("  e    : EMERGENCY LAND (VEL/PNG/POS/ATL: latch fixed-thrust gentle descent)\n");
    printf("  t    : AUTO TAKEOFF (mode 9/10: climb to target alt then hold)\n");
    printf("  m    : MISSION START (mode 10: HOLD -> PNG guidance)\n");
    printf("  l    : AUTO LAND (mode 9/10: descend + touchdown disarm)\n");
    printf("  p    : PWM set (prompts ch/us)\n");
    printf("  ?    : This help\n");
    printf("  ESC  : Quit\n");
    printf("================\n\n");
}

// ============================================================
// Keyboard handler
// ============================================================
static void handle_keyboard()
{
    if (!_kbhit()) return;
    int c = _getch();

    if (c == 27) {
        printf("[GCS] Quit\n");
        p8_send_abort();
        dashboard_stop();
        ntrip_close();
        cleanupOptiTrack();
        if (udp_sock != INVALID_SOCKET) closesocket(udp_sock);
        if (udp_recv_sock != INVALID_SOCKET) closesocket(udp_recv_sock);
        serial_close();
        WSACleanup();
        exit(0);
    }

    if (c >= '0' && c <= '9') {
        uint8_t mode = (uint8_t)(c - '0');
        // The '3' key is remapped from PWM_TEST (0x03) to MISSION (0x0A): MISSION
        // is mode 10, which has no single-character digit key, and PWM_TEST is not
        // driven from this console (motor PWM testing uses the on-board DebugConsole
        // / RC_CONTROL path). The MCU enum and PWM_TEST code are unchanged; only the
        // GCS entry point is re-routed so MISSION is reachable with one key.
        if (mode == 0x03) mode = 10;   // 3 -> MISSION (MCU MODE_MISSION = 0x0A)
        printf("[GCS] -> SET_MODE %s\n", MODE_NAMES[mode]);
        p8_send_set_mode(mode);
        return;
    }

    switch (c) {
    case 'w': case 'W': {
        // List WayPoint/*.txt, let the user pick one by number, then upload it
        // via CMD_SET_WAYPOINTS. Files include the original mission.txt and any
        // mission_subpc_*.txt saved from the sub-PC mission planner.
        char names[64][260];
        int file_count = 0;
        WIN32_FIND_DATAA fd;
        HANDLE hf = FindFirstFileA("../WayPoint/*.txt", &fd);
        if (hf != INVALID_HANDLE_VALUE) {
            do {
                if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) &&
                    file_count < 64) {
                    strncpy(names[file_count], fd.cFileName, 259);
                    names[file_count][259] = '\0';
                    file_count++;
                }
            } while (FindNextFileA(hf, &fd));
            FindClose(hf);
        }
        if (file_count == 0) {
            printf("[GCS] No mission files in WayPoint/\n");
            break;
        }
        printf("\n[GCS] WayPoint files:\n");
        for (int i = 0; i < file_count; i++)
            printf("   [%d] %s\n", i, names[i]);
        printf("  Select file number (-1 to cancel): ");
        int sel = -1;
        if (scanf("%d", &sel) != 1 || sel < 0 || sel >= file_count) {
            printf("[GCS] cancelled\n");
            break;
        }

        char path[320];
        snprintf(path, sizeof(path), "../WayPoint/%s", names[sel]);
        FILE* f = fopen(path, "r");
        if (!f) {
            printf("[GCS] cannot open %s\n", path);
            break;
        }
        // File format (per non-comment line), all comma- OR space-separated:
        //   lat lon              -> alt defaults to MISSION_DEFAULT_ALT_M (+warn)
        //   lat lon alt          -> per-WP altitude [m, takeoff-ground-relative]
        //   lat lon alt flags    -> + per-WP behaviour flag for the segment AFTER
        //                           this WP. One mode per WP:
        //                             0 = straight PNG (default)
        //                             1 = WP_FLAG_AVOID  (collision-cone avoidance)
        //                             2 = WP_FLAG_ORBIT  (orbit the next-WP midpoint)
        //                           Do not combine (no 3); MCU gives ORBIT priority.
        const float MISSION_DEFAULT_ALT_M = 1.5f;   // matches MCU TKO_TARGET_ALT
        double  lat[PNG_MAX_WAYPOINTS], lon[PNG_MAX_WAYPOINTS];
        float   alt[PNG_MAX_WAYPOINTS];
        uint8_t flg[PNG_MAX_WAYPOINTS];
        int  n = 0;
        bool any_alt_defaulted = false;
        char line[128];
        while (n < PNG_MAX_WAYPOINTS && fgets(line, sizeof(line), f)) {
            if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
            double a, o, al; int fl;
            if (sscanf(line, "%lf , %lf , %lf , %d", &a, &o, &al, &fl) == 4 ||
                sscanf(line, "%lf %lf %lf %d",       &a, &o, &al, &fl) == 4) {
                lat[n] = a; lon[n] = o; alt[n] = (float)al; flg[n] = (uint8_t)fl; n++;
            } else if (sscanf(line, "%lf , %lf , %lf", &a, &o, &al) == 3 ||
                       sscanf(line, "%lf %lf %lf",       &a, &o, &al) == 3) {
                lat[n] = a; lon[n] = o; alt[n] = (float)al; flg[n] = 0; n++;
            } else if (sscanf(line, "%lf , %lf", &a, &o) == 2 ||
                       sscanf(line, "%lf %lf",   &a, &o) == 2) {
                lat[n] = a; lon[n] = o;
                alt[n] = MISSION_DEFAULT_ALT_M; flg[n] = 0;
                any_alt_defaulted = true; n++;
            }
        }
        fclose(f);
        if (n == 0) {
            printf("[GCS] %s has no valid waypoints\n", names[sel]);
            break;
        }
        if (any_alt_defaulted)
            printf("[GCS] WARN: file missing altitude on some WPs -> "
                   "defaulted to %.1f m\n", MISSION_DEFAULT_ALT_M);
        printf("[GCS] -> SET_WAYPOINTS from %s (%d):\n", names[sel], n);
        for (int i = 0; i < n; i++)
            printf("   #%d  lat=%.7f  lon=%.7f  alt=%.1f  flags=%u\n",
                   i, lat[i], lon[i], alt[i], (unsigned)flg[i]);
        p8_send_waypoints(lat, lon, alt, flg, (uint8_t)n);
        break;
    }
    case 'q': case 'Q':
        printf("[GCS] -> ABORT\n");
        p8_send_abort();
        break;
    case 'h': case 'H':
        p8_send_heartbeat();
        break;
    case 'n': case 'N':
        if (ntrip_sock != INVALID_SOCKET) {
            ntrip_close();
            ntrip_cfg.enabled = false;
            printf("[NTRIP] Disconnected\n");
        } else {
            if (ntrip_connect())
                ntrip_cfg.enabled = true;
        }
        break;
    case 'g': case 'G':
        printf("[GCS] -> DEBUG_TOGGLE\n");
        p8_send_debug_toggle();
        break;
    case 'd': case 'D':
        printf("[GCS] -> PWM_DISARM\n");
        p8_send_pwm_disarm();
        break;
    case 'e': case 'E':
        printf("[GCS] -> EMERGENCY LAND (VEL/PNG/POS/ATL: fixed-thrust gentle descent)\n");
        p8_send_emergency_land();
        break;
    case 't': case 'T':
        printf("[GCS] -> AUTO TAKEOFF (AUTO_TKO_LAND: climb to target alt)\n");
        p8_send_auto_takeoff();
        break;
    case 'l': case 'L':
        printf("[GCS] -> AUTO LAND (AUTO_TKO_LAND/MISSION: descend + touchdown disarm)\n");
        p8_send_auto_land();
        break;
    case 'm': case 'M':
        printf("[GCS] -> MISSION START (MISSION: HOLD -> PNG guidance)\n");
        p8_send_mission_start();
        break;
    case 'p': case 'P': {
        int ch = 0, us = 1000;
        printf("  Motor channel (0-3): "); scanf("%d", &ch);
        printf("  Pulse width (us): ");    scanf("%d", &us);
        printf("[GCS] -> PWM_SET ch=%d us=%d\n", ch, us);
        p8_send_pwm_set((uint8_t)ch, (uint16_t)us);
        break;
    }
    case '?':
        print_help();
        break;
    }
}