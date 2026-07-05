#pragma once
#include "p8_comm.h"

// ============================================================
// Real-time ImGui dashboard (Win32 + OpenGL3, separate UI thread).
//
// The GCS real-time loop publishes frames as they arrive; the UI thread renders
// the latest snapshot at ~60 FPS. Purely a visualization view - sends nothing.
//
// Source policy (mirrors the sub-PC telemetry fallback): DebugFrame is the
// preferred source (rich: PID internals, altitude est, position PID, torques).
// TelemFrame is the fallback (basic: attitude/position/velocity/battery/rtk),
// flows even in IDLE. When a DebugFrame arrived recently the dashboard shows the
// full debug view; otherwise it shows the basic TelemFrame view, so something is
// always visible from power-on. Debug-only panels read "-- (debug off)" in the
// basic view.
// ============================================================

// Start the dashboard UI thread. Safe no-op if already running.
void dashboard_start();

// Stop the dashboard UI thread and close its window. Safe to call at exit.
void dashboard_stop();

// Publish the most recent DebugFrame (preferred/full source). Thread-safe.
void dashboard_push(const DebugFrame* df);

// Publish the most recent TelemFrame (fallback/basic source). Thread-safe.
// Used to render attitude/position/velocity/battery/rtk before any DebugFrame
// flows (e.g. IDLE, debug OFF).
void dashboard_push_telem(const TelemFrame* tf);
