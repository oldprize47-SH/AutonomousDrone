#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include "OptiTrack.h"

int main()
{
    printf("=== OptiTrack Test ===\n");
    printf("Port: %d, Sender: %s\n\n", OPTITRACK_PORT, OPTITRACK_SENDER_IP);

    if (!initOptiTrack()) {
        printf("[ERR] OptiTrack init failed\n");
        return 1;
    }

    OptiTrackData_t data = {};
    int count = 0;

    printf("Waiting for OptiTrack data... (Ctrl+C to quit)\n\n");

    while (1) {
        if (recvOptiTrack(&data)) {
            count++;
            printf("[%5d] pos=(%.3f, %.3f, %.3f)  euler=(yaw=%.1f, roll=%.1f, pitch=%.1f)\n",
                   count, data.x, data.y, data.z,
                   data.yaw, data.roll, data.pitch);
        }
        Sleep(10);  // 100Hz polling
    }

    cleanupOptiTrack();
    return 0;
}
