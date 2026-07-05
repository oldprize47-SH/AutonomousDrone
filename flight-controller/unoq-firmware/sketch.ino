//
// AVC UNO Q - Mode-Selectable Flight Controller Firmware
// Arduino UNO Q (STM32U585 Cortex-M33)
//
// Communication: P8 Telemetry via USART3 (PA7/PA5) binary protocol
//
// Modes:
//   IDLE        - Menu / standby
//   RC_CONTROL  - RC receiver + PID + motor output
//   RTK_MAIN    - GNSS/INS data monitor + RTCM forwarding
//   PWM_TEST    - Individual motor control via commands
//   MTI_TEST    - IMU data output
//

#include <Arduino.h>
#include "hw_config.h"
#if USE_BRIDGE
#include <Arduino_RouterBridge.h>
#endif
#include "INSS.h"
#include "QuadPWM.h"
#include "RCInput.h"
#include "Mode.h"
#include "Telem.h"
#include "LidarTF.h"
#if USE_OBSTACLE_RX
#include "ObstacleRx.h"
#endif

#if USE_DEBUG_SERIAL
#include "DebugConsole.h"
#endif

// ============================================================
// Global Variables
// ============================================================

FlightMode current_mode = MODE_IDLE;

Usart3Serial P8Serial;
TelemParser  p8_parser;
Usart1Serial Usart1Ser;

INSS     ins;
QuadPWM  motors;
LidarTF  lidar;
#if USE_OBSTACLE_RX
ObstacleRx obstacle;   // H7 obstacle slave reader (QWIIC = Wire1, 0x20)
#endif
bool     ins_active    = false;
bool     motors_active = false;

// Battery voltage (mV)
uint16_t batt_main_mv = 0;
uint16_t batt_sub_mv  = 0;

// Debug output flag (true = DebugFrame in RC_CONTROL, false = TelemFrame)
bool debug_output_enabled = true;

// LED timers
unsigned long last_ins_led_time = 0;
unsigned long last_p8_led_time  = 0;
static uint32_t last_alive_led_ms = 0;

// Downlink timer
static uint32_t last_telem_ms = 0;

// Battery sample timer
static uint32_t last_batt_ms = 0;

#ifndef LEDB
#define LEDB LED3_B
#define LEDG LED3_G
#define LEDR LED4_R
#endif

// ============================================================
// setup()
// ============================================================

void setup()
{
    pinMode(LEDB, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDR, OUTPUT);

    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDR, HIGH);

    // Bridge + Serial init (on UNO Q, Serial.begin() also starts Bridge)
    Serial.begin(DEBUG_BAUD);
    while (!Serial && millis() < 3000);

    // Pre-initialize Zephyr ADC driver before USART3
    // First call is heavy (calibration etc), subsequent calls are lightweight
    analogRead(BATT_MAIN_PIN);
    analogRead(BATT_SUB_PIN);

    P8Serial.begin(TELEM_BAUD);

    // RC ISR always active (for mode switching and EMERGENCY)
    rc_init();

#if USE_BRIDGE
    // Bridge RPC for MPU debug (status string queried from Linux).
    Bridge.provide("get_status", +[]() -> String {
        char buf[512];
        MtiData d = ins.getData();
        float cal_r = (float)d.data[ROLL]  - get_roll_offset();
        float cal_p = (float)d.data[PITCH] - get_pitch_offset();
        snprintf(buf, sizeof(buf),
            "mode=%d ctrl=%d ins=%d mtr=%d "
            "rc=[%u,%u,%u,%u,%u,%u] "
            "R=%.1f P=%.1f gx=%.1f gy=%.1f "
            "cal=%.1f,%.1f off=%.1f,%.1f "
            "mtr=[%u,%u,%u,%u] batt=%umV",
            (int)current_mode,
            (int)get_control_source(),
            ins_active ? 1 : 0,
            motors_active ? 1 : 0,
            rc_pulse_us[0], rc_pulse_us[1], rc_pulse_us[2],
            rc_pulse_us[3], rc_pulse_us[4], rc_pulse_us[5],
            d.data[ROLL], d.data[PITCH],
            d.data[GYR_X], d.data[GYR_Y],
            cal_r, cal_p,
            get_roll_offset(), get_pitch_offset(),
            motors.getPulseWidth(0), motors.getPulseWidth(1),
            motors.getPulseWidth(2), motors.getPulseWidth(3),
            batt_main_mv);
        return String(buf);
    });

#if USE_VISION_LANDING
    // RealSense helipad target relay (Linux python/main.py reads track_helipad.py
    // and forwards the compact helipad_stm payload). Copy-only sink; the ATL
    // landing loop gates + rotates it. Args are int(valid)+4 doubles to match the
    // MsgPack positional decoding (same path as the old set_cam_alt double arg).
    Bridge.provide("set_helipad_target",
        +[](int valid, double x_m, double y_m, double xy_m, double gnd_m) {
            set_helipad_target(valid, x_m, y_m, xy_m, gnd_m);
        });
#endif // USE_VISION_LANDING
#endif // USE_BRIDGE

    // QWIIC bus (Wire1 / I2C4) is shared by the TF-Nova altimeter (0x10) and
    // the H7 obstacle slave (0x20). Bring the bus up exactly once here, then
    // bind each device driver to it. MUST run before imu_setup(), which does
    // the LiDAR ground-bias calibration and needs the bus/driver already up.
    #if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) || USE_OBSTACLE_RX
    Wire1.begin();
    Wire1.setClock(100000);
    #endif

    // TF-Nova LiDAR altimeter — only when configured as the aiding source.
    #if ALT_AIDING_SOURCE == ALT_AID_LIDAR
    lidar.begin(LIDAR_WIRE, LIDAR_I2C_ADDR);
    #endif

    // H7 obstacle slave reader on the same QWIIC bus.
    #if USE_OBSTACLE_RX
    obstacle.begin(OBSTACLE_WIRE, OBSTACLE_I2C_ADDR);
    #endif

    // IMU init + attitude/baro calibration (blocking ~4s)
    imu_setup();

    current_mode = MODE_IDLE;

    #if USE_DEBUG_SERIAL
    print_menu();
    #endif

    digitalWrite(LED_BUILTIN, LOW);
}

// ============================================================
// loop()
// ============================================================

// RC signal presence tracking (debounce)
static uint32_t rc_lost_since_ms = 0;
static bool     rc_was_valid     = false;
bool            rc_auto_inhibit  = false;  // set by CMD_ABORT to prevent auto re-enter
// Emergency-landing latch (GCS CMD_EMERGENCY_LAND). When set, PNG forces U1 to
// EMLAND_THRUST_FRAC*mg for a gentle descent. Cleared only by RC EMERGENCY /
// mode change (enter_mode/exit), never by a toggle.
volatile bool   emland_active    = false;
// Auto takeoff/landing request latches (GCS CMD_AUTO_TAKEOFF / CMD_AUTO_LAND).
// Set by Telem on a valid command, consumed (cleared) by loop_auto_tko_land()
// and loop_mission(). atl_mission_request (CMD_MISSION_START) starts PNG guidance
// from HOLD in MODE_MISSION only.
volatile bool   atl_takeoff_request = false;
volatile bool   atl_land_request    = false;
volatile bool   atl_mission_request = false;

void loop()
{
    // 0. RC-based mode control (highest priority, overrides P8 commands)
    bool rc_signal_valid = (rc_pulse_us[RC_CH_THROTTLE] >= 900 &&
                            rc_pulse_us[RC_CH_THROTTLE] <= 2200);

    if (rc_signal_valid) {
        rc_was_valid = true;
        rc_lost_since_ms = millis();

        // RC present: auto-enter RC_CONTROL if IDLE
        // Inhibit flag is cleared when RC signal goes away then comes back
        if (rc_auto_inhibit) {
            // do nothing until RC signal cycles off then on
        } else {
            static uint32_t last_rc_enter_ms = 0;
            if (current_mode == MODE_IDLE &&
                (millis() - last_rc_enter_ms) >= 1000) {
                last_rc_enter_ms = millis();
                enter_mode(MODE_RC_CONTROL);
            }
        }
    } else {
        // RC lost: debounce before exiting RC_CONTROL
        if (rc_was_valid && current_mode == MODE_RC_CONTROL &&
            (millis() - rc_lost_since_ms) >= RC_LOST_TIMEOUT_MS) {
            rc_was_valid = false;
            exit_current_mode();
            #if USE_DEBUG_SERIAL
            Serial.println("[RC] Signal lost -> IDLE");
            #endif
        }
        // Clear inhibit when RC goes away (so next power-on re-enables auto-enter)
        rc_auto_inhibit = false;
    }

    // 1. P8 uplink (USART3)
    while (P8Serial.available()) {
        if (p8_parser.feedByte(P8Serial.read())) {
            telem_handle_cmd(p8_parser.getCmd());

            if (millis() - last_p8_led_time > 100) {
                digitalWrite(LEDG, !digitalRead(LEDG));
                last_p8_led_time = millis();
            }
        }
    }

    // 2. USB local input (debug)
    #if USE_DEBUG_SERIAL
    handle_usb_input();
    #endif

    // 3. Mode-specific loop
    switch (current_mode) {
    case MODE_IDLE:       loop_idle();       break;
    case MODE_RC_CONTROL: loop_rc_control(); break;
    case MODE_VEL_CONTROL: loop_vel_control(); break;
    case MODE_PNG_GUIDANCE: loop_png_guidance(); break;
    case MODE_POS_CONTROL: loop_pos_control(); break;
    case MODE_AUTO_TKO_LAND: loop_auto_tko_land(); break;
    case MODE_MISSION:    loop_mission();    break;
    case MODE_RTK_MAIN:   loop_rtk_main();   break;
    case MODE_PWM_TEST:   loop_pwm_test();   break;
    case MODE_MTI_TEST:   loop_mti_test();   break;
    case MODE_ESC_CAL:    loop_esc_cal();    break;
    default: break;
    }

    // 4. Battery voltage sampling (8-sample moving average)
    if (millis() - last_batt_ms >= (1000 / BATT_SAMPLE_HZ)) {
        last_batt_ms = millis();

        static uint16_t b1_buf[8], b2_buf[8];
        static uint8_t  b_idx = 0;
        static bool     b_filled = false;

        b1_buf[b_idx] = (uint16_t)(analogRead(BATT_MAIN_PIN) * (ADC_VREF_MV / (float)ADC_MAX) * BATT_DIVIDER_RATIO);
        b2_buf[b_idx] = (uint16_t)(analogRead(BATT_SUB_PIN)  * (ADC_VREF_MV / (float)ADC_MAX) * BATT_DIVIDER_RATIO);
        b_idx = (b_idx + 1) % 8;
        if (b_idx == 0) b_filled = true;

        uint8_t cnt = b_filled ? 8 : b_idx;
        uint32_t sum1 = 0, sum2 = 0;
        for (uint8_t i = 0; i < cnt; i++) { sum1 += b1_buf[i]; sum2 += b2_buf[i]; }
        batt_main_mv = (uint16_t)(sum1 / cnt);
        batt_sub_mv  = (uint16_t)(sum2 / cnt);
    }

    // 5. Periodic downlink
    // Tight-loop modes + debug ON : DebugFrame is sent from the tight loop
    // Tight-loop modes + debug OFF: TelemFrame sent here
    // Other modes                 : TelemFrame sent here
    if (millis() - last_telem_ms >= (1000 / TELEM_FREQ_HZ)) {
        bool in_tight_loop = (current_mode == MODE_RC_CONTROL ||
                              current_mode == MODE_VEL_CONTROL ||
                              current_mode == MODE_PNG_GUIDANCE ||
                              current_mode == MODE_POS_CONTROL ||
                              current_mode == MODE_AUTO_TKO_LAND ||
                              current_mode == MODE_MISSION);
        if (!in_tight_loop || !debug_output_enabled) {
            last_telem_ms = millis();
            telem_send_frame(P8Serial);
        }
    }

    // 6. Alive LED (1Hz blink)
    if (millis() - last_alive_led_ms >= 500) {
        last_alive_led_ms = millis();
        digitalWrite(LEDR, !digitalRead(LEDR));
    }

    // 7. INS sensor LED
    if (ins_active && ins.available()) {
        if (millis() - last_ins_led_time > 100) {
            digitalWrite(LEDB, !digitalRead(LEDB));
            last_ins_led_time = millis();
        }
    }

    // 8. H7 obstacle slave polling (~50Hz). Minimal receive-verification stage:
    //    poll the 0x20 slave, validate length+CRC, and dump the decoded list +
    //    rolling stats to the debug serial. No guidance use yet.
    #if USE_OBSTACLE_RX
    {
        static uint32_t last_obs_ms   = 0;
        static uint32_t last_stat_ms  = 0;
        if (millis() - last_obs_ms >= (1000 / 50)) {   // ~50Hz
            last_obs_ms = millis();

            // TF-Nova altimeter read on the SAME QWIIC bus (0x10), printed next to
            // the H7 obstacle data so both LiDAR sources can be verified together.
            // Both share Wire1; reading them back-to-back here confirms there is no
            // bus contention between the 0x10 and 0x20 devices.
            #if (ALT_AIDING_SOURCE == ALT_AID_LIDAR) && USE_DEBUG_SERIAL
            {
                float lid_m = 0.0f;
                bool  lid_ok = lidar.read(lid_m);
                // Zephyr nano libc prints float as "ovf"; emit as integer mm with a
                // leading sign so a zeroed/negative altitude prints cleanly.
                int   mm  = (int)((lid_ok ? lid_m : lidar.lastRangeM()) * 1000.0f);
                int   amm = (mm < 0) ? -mm : mm;
                int   off_mm = (int)(lidar.groundOffset() * 1000.0f);
                Serial.print("[LID] tfnova ok="); Serial.print(lid_ok ? 1 : 0);
                Serial.print(" alt=");
                if (mm < 0) Serial.print("-");
                Serial.print(amm / 1000); Serial.print(".");
                Serial.print((amm / 100) % 10); Serial.print((amm / 10) % 10);
                Serial.print(" m off=");
                Serial.print(off_mm / 1000); Serial.print(".");
                Serial.print((off_mm / 100) % 10); Serial.print((off_mm / 10) % 10);
                Serial.print(" m age=");
                Serial.print((int)(millis() - lidar.lastMs())); Serial.println("ms");
            }
            #endif

            bool ok = obstacle.poll();

            #if USE_DEBUG_SERIAL
            if (ok) {
                uint8_t cnt = obstacle.count();
                Serial.print("[OBS] seq="); Serial.print(obstacle.lastSeq());
                Serial.print(" n=");        Serial.print(cnt);
                for (uint8_t i = 0; i < cnt; ++i) {
                    const ObstacleDec& o = obstacle.obstacle(i);
                    // Print angles as integer tenths-of-degree: the UNO Q's
                    // Zephyr nano libc prints float as "ovf". *_deg10 are deg*10
                    // (center signed: forward=0, right+, left-). Compute the
                    // magnitude in a plain int first (avoid the abs() macro's
                    // sign quirks) so a negative value prints "-70.4", not
                    // "-70.-4", with the sign emitted once up front.
                    int cd = (int)o.center_deg10;
                    int mag = (cd < 0) ? -cd : cd;
                    Serial.print(" | id=");   Serial.print(o.id);
                    Serial.print(" deg=");
                    if (cd < 0) Serial.print("-");
                    Serial.print(mag / 10); Serial.print(".");
                    Serial.print(mag % 10);
                    Serial.print(" d=");      Serial.print(o.dist_mm);
                    Serial.print("mm hs=");   Serial.print(o.half_span_deg10 / 10);
                    Serial.print(".");        Serial.print(o.half_span_deg10 % 10);
                }
                Serial.println();
            }
            // Rolling stats once a second regardless of success.
            if (millis() - last_stat_ms >= 1000) {
                last_stat_ms = millis();
                Serial.print("[OBS-STAT] ok=");  Serial.print(obstacle.okCount());
                Serial.print(" short=");          Serial.print(obstacle.shortCount());
                Serial.print(" crc=");            Serial.println(obstacle.crcCount());
            }
            #endif
        }
    }
    #endif
}
