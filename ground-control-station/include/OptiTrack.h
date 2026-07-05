#ifndef OPTITRACK_H
#define OPTITRACK_H

#include <cstdio>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

// OptiTrack UDP 설정
#define OPTITRACK_PORT      38030
#define OPTITRACK_SENDER_IP "192.168.137.10"
#define OPTITRACK_MSG_LEN   6   // float x 6 (Position 3 + Euler 3)

typedef struct
{
    float x;        // Position X
    float y;        // Position Y
    float z;        // Position Z
    float yaw;      // Euler Yaw
    float roll;     // Euler Roll
    float pitch;    // Euler Pitch
} OptiTrackData_t;

/**
 * @brief OptiTrack UDP 수신 소켓 초기화
 * @return true: 성공, false: 실패
 */
bool initOptiTrack();

/**
 * @brief OptiTrack 데이터 수신 (논블로킹)
 * @param data 수신된 데이터를 저장할 구조체 포인터
 * @return true: 새 데이터 수신, false: 데이터 없음 또는 에러
 */
bool recvOptiTrack(OptiTrackData_t* data);

/**
 * @brief OptiTrack UDP 수신 소켓 종료
 */
void cleanupOptiTrack();

#endif // OPTITRACK_H
