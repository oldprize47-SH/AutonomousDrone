#include "serial_port.h"
#include <stdio.h>

static HANDLE hSerial = INVALID_HANDLE_VALUE;

bool serial_open(const char* port, uint32_t baud)
{
    char path[64];
    snprintf(path, sizeof(path), "\\\\.\\%s", port);

    hSerial = CreateFileA(path, GENERIC_READ | GENERIC_WRITE,
                          0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) return false;

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 1;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 1;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = 100;
    SetCommTimeouts(hSerial, &timeouts);

    return true;
}

void serial_close()
{
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
}

int serial_read(uint8_t* buf, int max_len)
{
    DWORD n = 0;
    ReadFile(hSerial, buf, max_len, &n, NULL);
    return (int)n;
}

bool serial_write(const uint8_t* buf, int len)
{
    DWORD written = 0;
    return WriteFile(hSerial, buf, len, &written, NULL) && (int)written == len;
}
