//
// USB Serial Debug Console (mode select, PWM test commands)
//

#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "hw_config.h"

#if USE_DEBUG_SERIAL

void print_menu();
void handle_usb_input();

#endif // USE_DEBUG_SERIAL

#endif // DEBUG_CONSOLE_H
