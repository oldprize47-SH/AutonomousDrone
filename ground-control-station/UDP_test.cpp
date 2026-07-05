// ============================================================
// UDP Dummy Telemetry Generator
// Sends fake TELEM JSON packets with sinusoidal roll/pitch
// ============================================================

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <windows.h>

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#pragma comment(lib, "ws2_32.lib")

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main()
{
    // ------------------------------------------------------------
    // Winsock init
    // ------------------------------------------------------------
    WSADATA wsa;
    WSAStartup(MAKEWORD(2,2), &wsa);

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sock == INVALID_SOCKET)
    {
        printf("socket() failed\n");
        return -1;
    }

    // ------------------------------------------------------------
    // Destination
    // ------------------------------------------------------------
    sockaddr_in dst = {};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(14555);

    // 수신 PC 주소
    //dst.sin_addr.s_addr = inet_addr("127.0.0.1");
    // 필요하면:
    dst.sin_addr.s_addr = inet_addr("192.168.0.74");

    printf("Sending UDP telemetry...\n");

    // ------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------
    LARGE_INTEGER freq, start, now;

    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&start);

    uint32_t uptime_ms = 0;

    // ------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------
    while (1)
    {
        QueryPerformanceCounter(&now);

        double t =
            (double)(now.QuadPart - start.QuadPart) /
            (double)freq.QuadPart;

        // --------------------------------------------------------
        // Sinusoidal motion
        // --------------------------------------------------------

        // Roll: ±30 deg
        float roll_deg =
            30.0f * (float)sin(2.0 * M_PI * 0.2 * t);

        // Pitch: ±20 deg (90 deg phase shift)
        float pitch_deg =
            20.0f * (float)cos(2.0 * M_PI * 0.2 * t);

        // Optional yaw
        float yaw_deg =
            45.0f * (float)sin(2.0 * M_PI * 0.05 * t);

        // yaw rate
        float yaw_rate =
            45.0f * 2.0f * (float)M_PI * 0.05f *
            (float)cos(2.0 * M_PI * 0.05 * t);

        // --------------------------------------------------------
        // Build JSON packet
        // --------------------------------------------------------
        char buf[512];

        int len = snprintf(
            buf,
            sizeof(buf),

            "{"
            "\"time\":%.3f,"
            "\"lat\":%.8f,"
            "\"lon\":%.8f,"
            "\"alt\":%.3f,"
            "\"roll\":%.3f,"
            "\"pitch\":%.3f,"
            "\"yaw\":%.3f,"
            "\"yaw_rate\":%.3f,"
            "\"vel_n\":%.3f,"
            "\"vel_e\":%.3f,"
            "\"vel_u\":%.3f,"
            "\"batt_mv\":%d"
            "}",

            t,
            36.103166,
            129.387661,
            12.3,

            roll_deg,
            pitch_deg,
            yaw_deg,
            yaw_rate,

            0.0,
            0.0,
            0.0,

            16800
        );

        // --------------------------------------------------------
        // Send UDP
        // --------------------------------------------------------
        sendto(
            sock,
            buf,
            len,
            0,
            (sockaddr*)&dst,
            sizeof(dst)
        );

        // 콘솔 출력
        printf(
            "\rroll=%7.2f  pitch=%7.2f  yaw=%7.2f",
            roll_deg,
            pitch_deg,
            yaw_deg
        );

        fflush(stdout);

        // --------------------------------------------------------
        // 100 Hz
        // --------------------------------------------------------
        Sleep(10);

        uptime_ms += 10;
    }

    closesocket(sock);
    WSACleanup();

    return 0;
}