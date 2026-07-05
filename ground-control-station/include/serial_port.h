#pragma once

#include <windows.h>
#include <stdint.h>

bool serial_open(const char* port, uint32_t baud);
void serial_close();
int  serial_read(uint8_t* buf, int max_len);
bool serial_write(const uint8_t* buf, int len);
