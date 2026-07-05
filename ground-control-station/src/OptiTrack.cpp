#include "OptiTrack.h"

// OptiTrack 수신 소켓
static SOCKET optiSocket = INVALID_SOCKET;

bool initOptiTrack()
{
    // Winsock 초기화 (중복 호출 안전)
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    // UDP 소켓 생성
    optiSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (optiSocket == INVALID_SOCKET)
    {
        printf("[OptiTrack] Socket creation failed: %d\n", WSAGetLastError());
        return false;
    }

    // 수신 주소 바인드 (포트 38030)
    struct sockaddr_in localAddr = {};
    localAddr.sin_family      = AF_INET;
    localAddr.sin_port        = htons(OPTITRACK_PORT);
    localAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(optiSocket, (struct sockaddr*)&localAddr, sizeof(localAddr)) == SOCKET_ERROR)
    {
        printf("[OptiTrack] Bind failed (port %d): %d\n", OPTITRACK_PORT, WSAGetLastError());
        closesocket(optiSocket);
        optiSocket = INVALID_SOCKET;
        return false;
    }

    // 논블로킹 모드 설정
    unsigned long nonBlocking = 1;
    if (ioctlsocket(optiSocket, FIONBIO, &nonBlocking) == SOCKET_ERROR)
    {
        printf("[OptiTrack] Failed to set non-blocking: %d\n", WSAGetLastError());
        closesocket(optiSocket);
        optiSocket = INVALID_SOCKET;
        return false;
    }

    printf("[OptiTrack] Initialized (port %d, sender %s)\n", OPTITRACK_PORT, OPTITRACK_SENDER_IP);
    return true;
}

bool recvOptiTrack(OptiTrackData_t* data)
{
    if (optiSocket == INVALID_SOCKET || data == nullptr)
    {
        return false;
    }

    float rxBuffer[OPTITRACK_MSG_LEN];
    float latestBuffer[OPTITRACK_MSG_LEN];
    struct sockaddr_in senderAddr = {};
    int senderAddrLen = sizeof(senderAddr);
    bool received = false;

    // Drain all buffered packets, keep only the latest
    while (true)
    {
        senderAddrLen = sizeof(senderAddr);
        int recvLen = recvfrom(optiSocket, (char*)rxBuffer, sizeof(rxBuffer), 0,
                               (struct sockaddr*)&senderAddr, &senderAddrLen);

        if (recvLen == sizeof(rxBuffer))
        {
            memcpy(latestBuffer, rxBuffer, sizeof(rxBuffer));
            received = true;
        }
        else
        {
            break;
        }
    }

    if (received)
    {
        data->x     = latestBuffer[0];
        data->y     = latestBuffer[1];
        data->z     = latestBuffer[2];
        data->yaw   = latestBuffer[3];
        data->roll  = latestBuffer[4];
        data->pitch = latestBuffer[5];
    }

    return received;
}

void cleanupOptiTrack()
{
    if (optiSocket != INVALID_SOCKET)
    {
        closesocket(optiSocket);
        optiSocket = INVALID_SOCKET;
    }

    printf("[OptiTrack] Connection closed\n");
}
