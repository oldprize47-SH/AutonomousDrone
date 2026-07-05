// ============================================================
// Real-time ImGui dashboard - Win32 + OpenGL3 backend, separate UI thread.
//
// Renders the latest DebugFrame as grouped panels (attitude, altitude,
// velocity, position, torque/motors, system). Visualization only.
// ============================================================
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <GL/gl.h>

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_opengl3.h"

#include "dashboard.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <cstdio>
#include <cmath>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// ------------------------------------------------------------
// Shared state between the GCS loop and the UI thread.
// DebugFrame is the preferred (full) source; TelemFrame is the fallback (basic)
// source. The UI thread picks DebugFrame when it arrived within FRESH_MS, else
// falls back to the latest TelemFrame.
// ------------------------------------------------------------
static std::mutex        g_mtx;
static DebugFrame        g_frame = {};
static TelemFrame        g_telem = {};
static std::atomic<bool> g_have_frame{false};
static std::atomic<bool> g_have_telem{false};
static std::atomic<unsigned long long> g_last_push_ms{0};       // last DebugFrame
static std::atomic<unsigned long long> g_last_telem_ms{0};      // last TelemFrame

static std::thread       g_ui_thread;
static std::atomic<bool> g_running{false};

static HDC   g_hdc = nullptr;
static HGLRC g_hglrc = nullptr;

// DebugFrame considered "fresh" (full view active) within this window.
static const unsigned long long DEBUG_FRESH_MS = 500;

void dashboard_push(const DebugFrame* df)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_frame = *df;
    g_have_frame = true;
    g_last_push_ms = GetTickCount64();
}

void dashboard_push_telem(const TelemFrame* tf)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_telem = *tf;
    g_have_telem = true;
    g_last_telem_ms = GetTickCount64();
}

// ------------------------------------------------------------
// Win32 window + OpenGL context helpers
// ------------------------------------------------------------
static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;
    switch (msg) {
    case WM_SIZE:
        if (wParam != SIZE_MINIMIZED)
            glViewport(0, 0, LOWORD(lParam), HIWORD(lParam));
        return 0;
    case WM_CLOSE:
        // Hide instead of destroy so the GCS keeps running headless.
        ShowWindow(hWnd, SW_HIDE);
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

static bool gl_create_context(HWND hWnd)
{
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.cStencilBits = 8;
    pfd.iLayerType = PFD_MAIN_PLANE;

    g_hdc = GetDC(hWnd);
    int pf = ChoosePixelFormat(g_hdc, &pfd);
    if (pf == 0) return false;
    if (!SetPixelFormat(g_hdc, pf, &pfd)) return false;

    g_hglrc = wglCreateContext(g_hdc);
    if (!g_hglrc) return false;
    return wglMakeCurrent(g_hdc, g_hglrc) == TRUE;
}

static void gl_destroy_context(HWND hWnd)
{
    wglMakeCurrent(nullptr, nullptr);
    if (g_hglrc) { wglDeleteContext(g_hglrc); g_hglrc = nullptr; }
    if (g_hdc)   { ReleaseDC(hWnd, g_hdc); g_hdc = nullptr; }
}

// ------------------------------------------------------------
// UI widgets
// ------------------------------------------------------------
static const float R2D = 57.29578f;

// A labelled numeric value with an optional colored bar gauge.
static void metric(const char* label, float value, const char* fmt = "%.2f")
{
    ImGui::TextUnformatted(label);
    ImGui::SameLine(160);
    ImGui::Text(fmt, value);
}

static void metric_bar(const char* label, float value, float vmin, float vmax,
                       const char* fmt = "%.2f")
{
    ImGui::TextUnformatted(label);
    ImGui::SameLine(160);
    char buf[32];
    snprintf(buf, sizeof(buf), fmt, value);
    float frac = (vmax > vmin) ? (value - vmin) / (vmax - vmin) : 0.0f;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    ImGui::ProgressBar(frac, ImVec2(-1, 0), buf);
}

static void draw_dashboard(const DebugFrame& d, bool stale, unsigned long long age_ms)
{
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin("AVC Debug Dashboard", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);

    // Connection status banner
    if (stale)
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1),
                           "STALE - no DebugFrame for %llu ms (press 'g' in GCS to enable debug)",
                           age_ms);
    else
        ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1),
                           "LIVE   t=%.1fs   loop_dt_max=%.1fms",
                           d.uptime_ms / 1000.0, d.loop_dt_max_ms);
    ImGui::Separator();

    ImGui::Columns(3, "groups", true);

    // ---- Attitude ----
    ImGui::SeparatorText("Attitude");
    metric_bar("roll  (deg)", d.roll * R2D, -45, 45, "%.1f");
    metric_bar("pitch (deg)", d.pitch * R2D, -45, 45, "%.1f");
    metric("yaw   (deg)", d.yaw * R2D, "%.1f");
    metric("p (rad/s)", d.p, "%.3f");
    metric("q (rad/s)", d.q, "%.3f");
    metric("r (rad/s)", d.r, "%.3f");
    metric("roll_cmd",  d.roll_cmd, "%.3f");
    metric("pitch_cmd", d.pitch_cmd, "%.3f");
    metric("r_cmd",     d.r_cmd, "%.3f");
    metric("e_roll",    d.e_roll, "%.3f");
    metric("e_pitch",   d.e_pitch, "%.3f");

    ImGui::Spacing();
    // ---- Velocity ----
    ImGui::SeparatorText("Velocity (body)");
    metric("u_cmd", d.u_cmd, "%.2f");
    metric("u_fb",  d.u_fb, "%.2f");
    metric("u_err", d.u_err, "%.2f");
    metric("v_cmd", d.v_cmd, "%.2f");
    metric("v_fb",  d.v_fb, "%.2f");
    metric("v_err", d.v_err, "%.2f");

    ImGui::NextColumn();

    // ---- Altitude ---- (baro/CF metrics removed from DebugFrame)
    ImGui::SeparatorText("Altitude");
    metric("lidar",    d.lidar_alt, "%.2f");
    if (d.lidar_valid) { ImGui::SameLine(); ImGui::TextColored(ImVec4(0.4f,1,0.4f,1), "[ok]"); }
    metric("alt_hold", (float)d.alt_hold, "%.0f");
    metric("alt_cmd",  d.alt_cmd, "%.2f");
    metric("alt_error",d.alt_error, "%.2f");
    metric("hdot_cmd", d.hdot_cmd, "%.2f");
    metric("h_used",   d.h_used, "%.2f");
    metric("v_used",   d.v_used, "%.2f");

    ImGui::Spacing();
    // ---- Position (PNG / POS) ----
    ImGui::SeparatorText("Position (PNG/POS)");
    metric("png_px (N)", d.png_px, "%.2f");
    metric("png_py (E)", d.png_py, "%.2f");
    metric("png_rng",    d.png_rng, "%.2f");
    metric("png_eta",    d.png_eta, "%.3f");
    metric("png_los",    d.png_los, "%.3f");
    metric("wp_idx",     (float)d.wp_idx, "%.0f");
    ImGui::Text("lat %.7f", d.png_lat);
    ImGui::Text("lon %.7f", d.png_lon);
    metric("png_alt",    d.png_alt, "%.1f");

    ImGui::Spacing();
    // ---- Collision-cone avoidance (PNG) ----
    ImGui::SeparatorText("Avoidance (PNG)");
    // Latch state: ON = green, OFF = gray.
    ImGui::TextUnformatted("avoid"); ImGui::SameLine(160);
    if (d.obs_avoid) ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "ON");
    else             ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1), "off");
    metric("obs_Ro",     d.obs_Ro, "%.2f");                 // nearest obstacle range [m]
    metric("obs_eo",     d.obs_eo * 57.2958f, "%.1f");      // heading error [deg]
    metric("obs_gamma",  d.obs_gamma * 57.2958f, "%.1f");   // cone half angle [deg]
    metric("r_ca",       d.obs_r_ca * 57.2958f, "%.1f");    // avoidance yaw-rate [deg/s]
    metric("theta_dot",  d.obs_theta_dot * 57.2958f, "%.1f"); // cone boundary rate [deg/s]

    ImGui::Spacing();
    // ---- ATL / Landing (auto takeoff-land + RealSense helipad precision) ----
    ImGui::SeparatorText("ATL / Landing");
    // State machine (colored: airborne=green, ground/disarmed=gray, land=yellow).
    {
        const uint8_t s = d.atl_state;
        ImVec4 scol = ImVec4(0.7f, 0.7f, 0.7f, 1);            // GROUND/DISARMED
        if (s == 5 || s == 6) scol = ImVec4(1, 1, 0.4f, 1);   // LAND/LAND_SLOW
        else if (s >= 1 && s <= 4) scol = ImVec4(0.4f, 1, 0.4f, 1); // airborne
        ImGui::TextUnformatted("atl_state"); ImGui::SameLine(160);
        ImGui::TextColored(scol, "%s", atl_state_name(s));
    }
    metric("climb_cmd", d.atl_climb_cmd, "%.2f");
    metric("hold_alt",  d.atl_hold_alt, "%.2f");
    metric("phase_ms",  (float)d.atl_phase_ms, "%.0f");
    metric("wp_alt_cmd",d.wp_alt_cmd, "%.2f");   // MISSION ramped WP altitude

    // (a) RAW camera input (latest from the bridge, regardless of steering).
    ImGui::Spacing();
    ImGui::TextUnformatted("-- RealSense raw --");
    ImGui::TextUnformatted("cam_valid"); ImGui::SameLine(160);
    if (d.helipad_valid) ImGui::TextColored(ImVec4(0.4f,1,0.4f,1), "VALID");
    else                 ImGui::TextColored(ImVec4(1,0.5f,0.5f,1), "no/stale");
    metric("cam_x (fwd)",  d.helipad_x, "%.3f");
    metric("cam_y (right)",d.helipad_y, "%.3f");
    metric("cam_gnd",      d.helipad_gnd, "%.2f");
    // Horizontal error bar (0..1 m); flashes the reject state when a spike drops.
    metric_bar("cam_xy", d.helipad_xy, 0.0f, 1.0f, "%.3f m");
    // Attitude-corrected NED offset (raw body x/y rotated by full roll/pitch/yaw
    // DCM, before adding pos). Differs from cam_x/y once the craft tilts.
    metric("dcm_dn (N)",   d.helipad_dn, "%.3f");
    metric("dcm_de (E)",   d.helipad_de, "%.3f");

    // (b)+(c) Latch actually used + controller status.
    ImGui::Spacing();
    ImGui::TextUnformatted("-- latch / steering --");
    ImGui::TextUnformatted("lock"); ImGui::SameLine(160);
    if (d.vlatch_set) ImGui::TextColored(ImVec4(0.4f,1,0.4f,1), "LOCKED");
    else              ImGui::TextColored(ImVec4(0.7f,0.7f,0.7f,1), "no lock");
    metric("latch_n (N)", d.vlatch_n, "%.2f");
    metric("latch_e (E)", d.vlatch_e, "%.2f");
    ImGui::TextUnformatted("steering"); ImGui::SameLine(160);
    if (d.vision_use) ImGui::TextColored(ImVec4(0.4f,1,0.4f,1), "VISION");
    else              ImGui::TextColored(ImVec4(0.7f,0.7f,0.7f,1), "GNSS hold");
    ImGui::TextUnformatted("spike_reject"); ImGui::SameLine(160);
    if (d.vision_reject) ImGui::TextColored(ImVec4(1,0.5f,0.5f,1), "REJECT");
    else                 ImGui::TextColored(ImVec4(0.7f,0.7f,0.7f,1), "-");

    ImGui::NextColumn();

    // ---- Torque / Motors ----
    ImGui::SeparatorText("Torque / Motors");
    metric("U1 (thrust N)", d.U1, "%.1f");
    metric("U2", d.U2, "%.3f");
    metric("U3", d.U3, "%.3f");
    metric("U4", d.U4, "%.3f");
    // F1..F4 (per-motor thrust) removed from DebugFrame.

    ImGui::Spacing();
    // ---- System ----
    ImGui::SeparatorText("System");
    metric("batt (V)", d.batt_mv / 1000.0f, "%.2f");
    const char* rtk = (d.rtk_status == 2) ? "FIXED" :
                      (d.rtk_status == 1) ? "FLOAT" : "NONE";
    ImVec4 rtk_col = (d.rtk_status == 2) ? ImVec4(0.4f,1,0.4f,1) :
                     (d.rtk_status == 1) ? ImVec4(1,1,0.4f,1) : ImVec4(1,0.5f,0.5f,1);
    ImGui::TextUnformatted("rtk"); ImGui::SameLine(160);
    ImGui::TextColored(rtk_col, "%s", rtk);
    ImGui::TextUnformatted("gnss_fix"); ImGui::SameLine(160);
    if (d.gnss_fix) ImGui::TextColored(ImVec4(0.4f,1,0.4f,1), "FIX");
    else            ImGui::TextColored(ImVec4(1,0.5f,0.5f,1), "no fix");
    metric("loop_50hz",   (float)d.loop_count_50hz, "%.0f");
    metric("loop_dt_max", d.loop_dt_max_ms, "%.1f");
    metric("vel_n", d.vel_n, "%.2f");
    metric("vel_e", d.vel_e, "%.2f");

    ImGui::Columns(1);
    ImGui::End();
}

// Basic view rendered from a TelemFrame (fallback source). Shows the fields
// TelemFrame carries; debug-only panels are marked "-- (debug off)".
static const char* mode_str_of(uint8_t m)
{
    return (m < MODE_COUNT) ? MODE_NAMES[m] : "???";
}

static void draw_dashboard_basic(const TelemFrame& t, bool stale,
                                 unsigned long long age_ms)
{
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin("AVC Debug Dashboard", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);

    // Banner: basic (TelemFrame) source. Yellow = telem-only (no debug stream).
    if (stale)
        ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1),
                           "STALE - no telemetry for %llu ms", age_ms);
    else
        ImGui::TextColored(ImVec4(1, 0.85f, 0.3f, 1),
                           "TELEM (basic)   mode=%s   t=%.1fs   "
                           "(press 'g' in GCS for full debug view)",
                           mode_str_of(t.mode), t.uptime_ms / 1000.0);
    ImGui::Separator();

    ImGui::Columns(3, "groups", true);

    // ---- Attitude (TelemFrame: deg already) ----
    ImGui::SeparatorText("Attitude");
    metric_bar("roll  (deg)",  t.roll_deg, -45, 45, "%.1f");
    metric_bar("pitch (deg)",  t.pitch_deg, -45, 45, "%.1f");
    metric("yaw   (deg)",      t.yaw_deg, "%.1f");
    metric("yaw_rate (dps)",   t.yaw_rate_dps, "%.1f");

    ImGui::Spacing();
    ImGui::SeparatorText("Velocity (NED)");
    metric("vel_n (m/s)", t.vel_n_ms, "%.2f");
    metric("vel_e (m/s)", t.vel_e_ms, "%.2f");
    metric("vel_u (m/s)", t.vel_u_ms, "%.2f");

    ImGui::NextColumn();

    // ---- Position ----
    ImGui::SeparatorText("Position");
    ImGui::Text("lat %.7f", t.lat_deg);
    ImGui::Text("lon %.7f", t.lon_deg);
    metric("alt (m)",  t.alt_m, "%.2f");
    metric("num_sv",   (float)t.gnss_num_sv, "%.0f");
    metric("fix_type", (float)t.gnss_fix_type, "%.0f");

    ImGui::Spacing();
    ImGui::SeparatorText("Altitude / Position PID");
    ImGui::TextDisabled("-- (debug off) --");

    ImGui::NextColumn();

    // ---- RC / Motors ----
    ImGui::SeparatorText("RC / Motors (us)");
    metric("RC1", (float)t.rc_ch[0], "%.0f");
    metric("RC2", (float)t.rc_ch[1], "%.0f");
    metric("RC3", (float)t.rc_ch[2], "%.0f");
    metric("RC4", (float)t.rc_ch[3], "%.0f");
    metric("M1",  (float)t.motor_us[0], "%.0f");
    metric("M2",  (float)t.motor_us[1], "%.0f");
    metric("M3",  (float)t.motor_us[2], "%.0f");
    metric("M4",  (float)t.motor_us[3], "%.0f");

    ImGui::Spacing();
    ImGui::SeparatorText("System");
    metric("batt1 (V)", t.batt1_mv / 1000.0f, "%.2f");
    metric("batt2 (V)", t.batt2_mv / 1000.0f, "%.2f");
    const char* rtk = rtk_status_str(t.status_flags);
    uint8_t rtkv = (t.status_flags >> STATUS_RTK_SHIFT) & 0x03;
    ImVec4 rtk_col = (rtkv == 2) ? ImVec4(0.4f,1,0.4f,1) :
                     (rtkv == 1) ? ImVec4(1,1,0.4f,1) : ImVec4(1,0.5f,0.5f,1);
    ImGui::TextUnformatted("rtk"); ImGui::SameLine(160);
    ImGui::TextColored(rtk_col, "%s", rtk);
    ImGui::TextUnformatted("gnss_fix"); ImGui::SameLine(160);
    if (t.status_flags & STATUS_BIT_GNSS_FIX)
        ImGui::TextColored(ImVec4(0.4f,1,0.4f,1), "FIX");
    else
        ImGui::TextColored(ImVec4(1,0.5f,0.5f,1), "no fix");

    ImGui::Columns(1);
    ImGui::End();
}

// ------------------------------------------------------------
// UI thread entry
// ------------------------------------------------------------
static void ui_thread_main()
{
    WNDCLASSEXW wc = { sizeof(wc), CS_OWNDC, WndProc, 0, 0,
                       GetModuleHandle(nullptr), nullptr, nullptr, nullptr,
                       nullptr, L"AVC_GCS_Dashboard", nullptr };
    RegisterClassExW(&wc);
    HWND hWnd = CreateWindowW(wc.lpszClassName, L"AVC Debug Dashboard",
                              WS_OVERLAPPEDWINDOW, 100, 100, 1100, 720,
                              nullptr, nullptr, wc.hInstance, nullptr);
    if (!hWnd) { UnregisterClassW(wc.lpszClassName, wc.hInstance); return; }

    if (!gl_create_context(hWnd)) {
        gl_destroy_context(hWnd);
        DestroyWindow(hWnd);
        UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return;
    }

    ShowWindow(hWnd, SW_SHOWDEFAULT);
    UpdateWindow(hWnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_InitForOpenGL(hWnd);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (g_running) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        if (!g_running) break;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        DebugFrame dsnap;
        TelemFrame tsnap;
        bool have_d, have_t;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            dsnap = g_frame; have_d = g_have_frame;
            tsnap = g_telem; have_t = g_have_telem;
        }
        unsigned long long now = GetTickCount64();
        unsigned long long d_age = now - g_last_push_ms;
        unsigned long long t_age = now - g_last_telem_ms;

        // Source policy: DebugFrame (full) when fresh, else TelemFrame (basic).
        if (have_d && d_age < DEBUG_FRESH_MS) {
            draw_dashboard(dsnap, false, d_age);          // full debug view
        } else if (have_t) {
            draw_dashboard_basic(tsnap, t_age > 1000, t_age);  // basic telem view
        } else if (have_d) {
            draw_dashboard(dsnap, true, d_age);           // debug went stale, no telem yet
        } else {
            ImGui::SetNextWindowPos(ImGui::GetMainViewport()->WorkPos);
            ImGui::Begin("AVC Debug Dashboard");
            ImGui::Text("Waiting for telemetry... (connect the drone / open the COM port)");
            ImGui::End();
        }

        ImGui::Render();
        RECT rc; GetClientRect(hWnd, &rc);
        glViewport(0, 0, rc.right - rc.left, rc.bottom - rc.top);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SwapBuffers(g_hdc);

        std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    gl_destroy_context(hWnd);
    DestroyWindow(hWnd);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
}

void dashboard_start()
{
    if (g_running) return;
    g_running = true;
    g_ui_thread = std::thread(ui_thread_main);
}

void dashboard_stop()
{
    if (!g_running) return;
    g_running = false;
    if (g_ui_thread.joinable())
        g_ui_thread.join();
}
