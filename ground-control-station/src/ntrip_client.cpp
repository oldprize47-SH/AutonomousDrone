#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include "ntrip_client.h"
#include "p8_comm.h"
#include <ws2tcpip.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// NTRIP configuration (defaults)
// ============================================================

static NtripConfig make_default_ntrip_cfg()
{
    NtripConfig c = {};
    strncpy(c.host, "RTS2.ngii.go.kr", sizeof(c.host));
    c.port = 2101;
    strncpy(c.mountpoint, "VRS-RTCM32", sizeof(c.mountpoint));
    strncpy(c.username, "joon0307", sizeof(c.username));
    strncpy(c.password, "ngii", sizeof(c.password));
    c.enabled = false;
    strncpy(c.gga_lat, "3606.1900", sizeof(c.gga_lat));
    c.gga_lat_ns = 'N';
    strncpy(c.gga_lon, "12923.2597", sizeof(c.gga_lon));
    c.gga_lon_ew = 'E';
    return c;
}
NtripConfig ntrip_cfg = make_default_ntrip_cfg();

SOCKET   ntrip_sock        = INVALID_SOCKET;
uint32_t ntrip_total_bytes  = 0;
uint32_t ntrip_total_fwd    = 0;

// ============================================================
// Base64 encoder (internal use for NTRIP auth)
// ============================================================

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void base64_encode(const char* input, char* output, int out_size)
{
    int len = (int)strlen(input);
    int i = 0, j = 0;
    while (i < len && j < out_size - 4) {
        int rem = len - i;
        uint8_t a = (uint8_t)input[i++];
        uint8_t b = (rem > 1) ? (uint8_t)input[i++] : 0;
        uint8_t c = (rem > 2) ? (uint8_t)input[i++] : 0;
        uint32_t triple = ((uint32_t)a << 16) | ((uint32_t)b << 8) | c;
        output[j++] = b64_table[(triple >> 18) & 0x3F];
        output[j++] = b64_table[(triple >> 12) & 0x3F];
        output[j++] = (rem > 1) ? b64_table[(triple >> 6) & 0x3F] : '=';
        output[j++] = (rem > 2) ? b64_table[triple & 0x3F] : '=';
    }
    output[j] = '\0';
}

// ============================================================
// GGA sender for VRS
// ============================================================

static uint32_t last_gga_tick = 0;
#define GGA_INTERVAL_MS 2000

// Live position flag: true once we get a valid fix from telemetry
static bool gga_has_live_pos = false;

void ntrip_update_position(double lat_deg, double lon_deg)
{
    if (lat_deg == 0.0 && lon_deg == 0.0) return;  // no fix yet

    char ns = (lat_deg >= 0) ? 'N' : 'S';
    char ew = (lon_deg >= 0) ? 'E' : 'W';
    double lat_abs = (lat_deg >= 0) ? lat_deg : -lat_deg;
    double lon_abs = (lon_deg >= 0) ? lon_deg : -lon_deg;

    // Convert decimal degrees -> DDMM.MMMMMM (NMEA format)
    int lat_d = (int)lat_abs;
    double lat_m = (lat_abs - lat_d) * 60.0;
    int lon_d = (int)lon_abs;
    double lon_m = (lon_abs - lon_d) * 60.0;

    snprintf(ntrip_cfg.gga_lat, sizeof(ntrip_cfg.gga_lat), "%02d%09.6f", lat_d, lat_m);
    ntrip_cfg.gga_lat_ns = ns;
    snprintf(ntrip_cfg.gga_lon, sizeof(ntrip_cfg.gga_lon), "%03d%09.6f", lon_d, lon_m);
    ntrip_cfg.gga_lon_ew = ew;

    gga_has_live_pos = true;
}

static void ntrip_send_gga()
{
    // Current UTC time from system clock
    SYSTEMTIME st;
    GetSystemTime(&st);

    // Build GGA body (between $ and *)
    char body[128];
    snprintf(body, sizeof(body),
             "GPGGA,%02d%02d%02d.%02d,%s,%c,%s,%c,1,12,1.0,10.0,M,0.0,M,,",
             st.wHour, st.wMinute, st.wSecond, st.wMilliseconds / 10,
             ntrip_cfg.gga_lat, ntrip_cfg.gga_lat_ns,
             ntrip_cfg.gga_lon, ntrip_cfg.gga_lon_ew);

    // NMEA checksum: XOR of all chars between '$' and '*'
    uint8_t cs = 0;
    for (int i = 0; body[i] != '\0'; i++) cs ^= (uint8_t)body[i];

    char frame[160];
    snprintf(frame, sizeof(frame), "$%s*%02X\r\n", body, cs);

    int sent = send(ntrip_sock, frame, (int)strlen(frame), 0);
    printf("[NTRIP] GGA sent (%d bytes): %.*s\n", sent, (int)strlen(frame) - 2, frame);
}

// ============================================================
// NTRIP client
// ============================================================

bool ntrip_connect()
{
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(ntrip_cfg.port);
    server.sin_addr.s_addr = inet_addr(ntrip_cfg.host);

    if (server.sin_addr.s_addr == INADDR_NONE) {
        struct hostent* he = gethostbyname(ntrip_cfg.host);
        if (!he) { printf("[NTRIP] DNS resolve failed: %s\n", ntrip_cfg.host); return false; }
        memcpy(&server.sin_addr, he->h_addr_list[0], he->h_length);
    }

    ntrip_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ntrip_sock == INVALID_SOCKET) { printf("[NTRIP] socket() failed\n"); return false; }

    printf("[NTRIP] Connecting to %s:%d...\n", ntrip_cfg.host, ntrip_cfg.port);

    if (connect(ntrip_sock, (struct sockaddr*)&server, sizeof(server)) != 0) {
        printf("[NTRIP] connect() failed\n");
        closesocket(ntrip_sock);
        ntrip_sock = INVALID_SOCKET;
        return false;
    }

    char auth_plain[256], auth_b64[512];
    snprintf(auth_plain, sizeof(auth_plain), "%s:%s", ntrip_cfg.username, ntrip_cfg.password);
    base64_encode(auth_plain, auth_b64, sizeof(auth_b64));

    char request[1024];
    snprintf(request, sizeof(request),
             "GET /%s HTTP/1.1\r\n"
             "Host: %s:%d\r\n"
             "Ntrip-Version: Ntrip/2.0\r\n"
             "User-Agent: AVC_GCS/1.0\r\n"
             "Authorization: Basic %s\r\n"
             "Accept: */*\r\n\r\n",
             ntrip_cfg.mountpoint, ntrip_cfg.host, ntrip_cfg.port, auth_b64);

    send(ntrip_sock, request, (int)strlen(request), 0);

    char response[1024];
    int n = recv(ntrip_sock, response, sizeof(response) - 1, 0);
    if (n <= 0) {
        printf("[NTRIP] No response\n");
        closesocket(ntrip_sock);
        ntrip_sock = INVALID_SOCKET;
        return false;
    }
    response[n] = '\0';

    if (strstr(response, "ICY 200 OK") || strstr(response, "200 OK")) {
        printf("[NTRIP] Connected to %s/%s\n", ntrip_cfg.host, ntrip_cfg.mountpoint);

        // Send initial GGA for VRS
        ntrip_send_gga();
        last_gga_tick = GetTickCount();
        printf("[NTRIP] GGA sent (%s%c, %s%c)\n",
               ntrip_cfg.gga_lat, ntrip_cfg.gga_lat_ns,
               ntrip_cfg.gga_lon, ntrip_cfg.gga_lon_ew);

        u_long mode = 1;
        ioctlsocket(ntrip_sock, FIONBIO, &mode);
        return true;
    }

    printf("[NTRIP] Server rejected: %s\n", response);
    closesocket(ntrip_sock);
    ntrip_sock = INVALID_SOCKET;
    return false;
}

void ntrip_close()
{
    if (ntrip_sock != INVALID_SOCKET) {
        closesocket(ntrip_sock);
        ntrip_sock = INVALID_SOCKET;
    }
}

void ntrip_poll()
{
    if (ntrip_sock == INVALID_SOCKET) return;

    // Periodic GGA for VRS
    uint32_t now = GetTickCount();
    if (now - last_gga_tick >= GGA_INTERVAL_MS) {
        last_gga_tick = now;
        ntrip_send_gga();
    }

    uint8_t buf[1024];
    int n = recv(ntrip_sock, (char*)buf, sizeof(buf), 0);
    if (n > 0) {
        ntrip_total_bytes += n;
        if (p8_send_rtcm(buf, (uint16_t)n)) ntrip_total_fwd++;
        printf("[NTRIP] RTCM %d bytes -> MCU (total: %u bytes, %u pkts)\n",
               n, ntrip_total_bytes, ntrip_total_fwd);
    } else if (n == 0) {
        printf("[NTRIP] Disconnected\n");
        ntrip_close();
    }
}
