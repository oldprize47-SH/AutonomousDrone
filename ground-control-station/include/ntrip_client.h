#pragma once

#include <winsock2.h>
#include <stdint.h>
#include <stdbool.h>

// NTRIP configuration
struct NtripConfig {
    char host[128];
    int  port;
    char mountpoint[64];
    char username[64];
    char password[64];
    bool enabled;
    // GGA position for VRS (DDMM.MMMM format)
    char gga_lat[16];       // e.g. "3606.1900"
    char gga_lat_ns;        // 'N' or 'S'
    char gga_lon[16];       // e.g. "12923.2597"
    char gga_lon_ew;        // 'E' or 'W'
};

extern NtripConfig ntrip_cfg;
extern SOCKET      ntrip_sock;
extern uint32_t    ntrip_total_bytes;
extern uint32_t    ntrip_total_fwd;

bool ntrip_connect();
void ntrip_close();
void ntrip_poll();

// Update GGA position from telemetry (decimal degrees)
void ntrip_update_position(double lat_deg, double lon_deg);
