//
// USB Serial Debug Console Implementation
//

#include "DebugConsole.h"

#if USE_DEBUG_SERIAL

#include "Mode.h"
#include "INSS.h"
#include "QuadPWM.h"
#include "Usart3Serial.h"
#include <stdlib.h>

// External references (defined in sketch.ino)
extern FlightMode current_mode;
extern QuadPWM    motors;

static char usb_buf[32];
static int  usb_idx = 0;

void print_menu()
{
    Serial.println();
    Serial.println("=== AVC UNO Q Mode Select ===");
    Serial.println("  0: IDLE");
    Serial.println("  1: RC_CONTROL");
    Serial.println("  2: RTK_MAIN");
    Serial.println("  3: PWM_TEST");
    Serial.println("  4: MTI_TEST");
    Serial.println("  q: ABORT -> IDLE");
    Serial.print("Current: "); Serial.println(mode_name(current_mode));
    Serial.println("=============================");
}

static void process_usb_line()
{
    usb_buf[usb_idx] = '\0';
    usb_idx = 0;
    if (usb_buf[0] == '\0') return;

    if (usb_buf[0] >= '0' && usb_buf[0] <= '4' && usb_buf[1] == '\0') {
        enter_mode((FlightMode)(usb_buf[0] - '0'));
        return;
    }

    if (usb_buf[0] == 'q' && usb_buf[1] == '\0') {
        exit_current_mode();
        Serial.println("[MODE] ABORT -> IDLE");
        print_menu();
        return;
    }

    if (current_mode == MODE_PWM_TEST) {
        if (usb_buf[0] == 'm' && usb_buf[1] >= '0' && usb_buf[1] <= '3' && usb_buf[2] == '\0') {
            pwm_selected_motor = usb_buf[1] - '0';
            Serial.print("> Motor "); Serial.print(pwm_selected_motor); Serial.println(" selected");
            return;
        }
        if (usb_buf[0] == '+' && usb_buf[1] == '\0') {
            pwm_motor_us[pwm_selected_motor] += 5;
            if (pwm_motor_us[pwm_selected_motor] > motors.getMaxPulse())
                pwm_motor_us[pwm_selected_motor] = motors.getMaxPulse();
            set_motor_output(pwm_motor_us[0], pwm_motor_us[1], pwm_motor_us[2], pwm_motor_us[3]);
            Serial.print("> M"); Serial.print(pwm_selected_motor);
            Serial.print(" = "); Serial.print(pwm_motor_us[pwm_selected_motor]); Serial.println(" us");
            return;
        }
        if (usb_buf[0] == '-' && usb_buf[1] == '\0') {
            if (pwm_motor_us[pwm_selected_motor] >= motors.getMinPulse() + 5)
                pwm_motor_us[pwm_selected_motor] -= 5;
            else
                pwm_motor_us[pwm_selected_motor] = motors.getMinPulse();
            set_motor_output(pwm_motor_us[0], pwm_motor_us[1], pwm_motor_us[2], pwm_motor_us[3]);
            Serial.print("> M"); Serial.print(pwm_selected_motor);
            Serial.print(" = "); Serial.print(pwm_motor_us[pwm_selected_motor]); Serial.println(" us");
            return;
        }
        if (usb_buf[0] == 'd' && usb_buf[1] == '\0') {
            set_motor_output(1000, 1000, 1000, 1000);
            for (int i = 0; i < 4; i++) pwm_motor_us[i] = motors.getMinPulse();
            Serial.println("> Disarmed");
            return;
        }
        if (usb_buf[0] == 's' && usb_buf[1] == '\0') {
            for (int i = 0; i < 4; i++) {
                Serial.print(i == pwm_selected_motor ? " >> M" : "    M");
                Serial.print(i); Serial.print(" = ");
                Serial.print(pwm_motor_us[i]); Serial.println(" us");
            }
            return;
        }
        bool is_num = true;
        for (int i = 0; usb_buf[i]; i++) {
            if (usb_buf[i] < '0' || usb_buf[i] > '9') { is_num = false; break; }
        }
        if (is_num && usb_buf[0] != '\0') {
            pwm_motor_us[pwm_selected_motor] = (uint16_t)atoi(usb_buf);
            set_motor_output(pwm_motor_us[0], pwm_motor_us[1], pwm_motor_us[2], pwm_motor_us[3]);
            Serial.print("> M"); Serial.print(pwm_selected_motor);
            Serial.print(" = "); Serial.print(pwm_motor_us[pwm_selected_motor]); Serial.println(" us");
            return;
        }
    }

    Serial.print("> Unknown: "); Serial.println(usb_buf);
}

void handle_usb_input()
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (usb_idx > 0) process_usb_line();
            continue;
        }
        if (usb_idx < (int)sizeof(usb_buf) - 1) usb_buf[usb_idx++] = c;
    }
}

#endif // USE_DEBUG_SERIAL
