#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"

#define Sleep(ms) usleep((ms) * 1000)
#define WHEEL_DIAMETER_MM 50
#define WHEEL_BASE_MM      104.0

static bool gyro_auto_reset = true;
static uint8_t left_motor  = DESC_LIMIT;
static uint8_t right_motor = DESC_LIMIT;

// ---------- Utility Methods ----------
static void wait_by_degrees(int speed, int degrees) {
    int wait = (speed != 0) ? ((abs(degrees) * 1000) / abs(speed)) + 200 : 1000;
    Sleep(wait);
}

static void wait_by_duration(int duration_ms) {
    Sleep(duration_ms + 200);
}

// ---------- Gyro Sensor Methods ----------
void set_gyro_auto_reset(bool enable) {
    gyro_auto_reset = enable;
}

static void reset_gyro(uint8_t sn_gyro) {
    set_sensor_mode(sn_gyro, "GYRO-RATE");
    Sleep(100);
    set_sensor_mode(sn_gyro, "GYRO-ANG");
    Sleep(100);
}

bool init_gyro(uint8_t* sn_gyro, bool reset) {
    if (ev3_search_sensor(LEGO_EV3_GYRO, sn_gyro, 0)) {
        if (reset || gyro_auto_reset) reset_gyro(*sn_gyro);
        return true;
    }
    return false;
}

bool get_gyro_angle(uint8_t sn_gyro, int* angle) {
    int raw = 0;
    if (get_sensor_value(0, sn_gyro, &raw)) {
        *angle = -raw;
        return true;
    }
    return false;
}

// ---------- Button Methods ----------
const char* get_button_name(uint8_t keys) {
    if (keys & EV3_KEY_UP)      return "UP";
    if (keys & EV3_KEY_DOWN)    return "DOWN";
    if (keys & EV3_KEY_LEFT)    return "LEFT";
    if (keys & EV3_KEY_RIGHT)   return "RIGHT";
    if (keys & EV3_KEY_CENTER)  return "CENTER";
    if (keys & EV3_KEY_BACK)    return "BACK";
    return NULL;
}

bool is_button_pressed(uint8_t button_mask) {
    uint8_t keys = 0;
    ev3_read_keys(&keys);
    return (keys & button_mask) != 0;
}

// ---------- Color Sensor Methods ----------
static const char* color_names[] = {
    "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN"
};
#define COLOR_COUNT ((int)(sizeof(color_names) / sizeof(color_names[0])))

void read_color_sensors(uint8_t sn1, uint8_t sn2, int* val1, int* val2) {
    int values[2] = {0, 0};
    uint8_t sns[2] = {sn1, sn2};
    for (int i = 0; i < 2; i++) {
        if (!get_sensor_value(0, sns[i], &values[i]) || values[i] < 0 || values[i] >= COLOR_COUNT) {
            values[i] = 0;
        }
    }
    *val1 = values[0];
    *val2 = values[1];
}

// ---------- Ultrasonic Sensor Methods ----------
bool init_ultrasonic(uint8_t* sn_us) {
    if (ev3_search_sensor(LEGO_EV3_US, sn_us, 0)) {
        set_sensor_mode(*sn_us, "US-DIST-CM");
        return true;
    }
    return false;
}

bool get_distance_mm(uint8_t sn_us, int* distance_mm) {
    return get_sensor_value(0, sn_us, distance_mm);
}

// ---------- Motor Methods ----------
bool init_motors(void) {
    ev3_tacho_init();
    int found = 0;
    for (int i = 0; i < DESC_LIMIT && found < 2; i++) {
        if (ev3_tacho[i].type_inx == LEGO_EV3_L_MOTOR) {
            if (found == 0) left_motor = i;
            else right_motor = i;
            found++;
        }
    }
    return (found == 2);
}

void set_speed(int speed) {
    set_tacho_speed_sp(left_motor,  speed);
    set_tacho_speed_sp(right_motor, speed);
}

void move_for_time(int speed, int duration_ms) {
    set_speed(speed);
    set_tacho_time_sp(left_motor,  duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    wait_by_duration(duration_ms);
}

void move_for_degrees(int speed, int degrees) {
    set_speed(speed);
    set_tacho_position_sp(left_motor,  degrees);
    set_tacho_position_sp(right_motor, degrees);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    wait_by_degrees(speed, degrees);
}

static int robot_to_wheel_deg(int robot_deg, float multiplier) {
    return (int)(robot_deg * multiplier * WHEEL_BASE_MM / WHEEL_DIAMETER_MM);
}

void tank_turn(int speed, int degrees) {
    int wheel_deg = robot_to_wheel_deg(degrees, 1.0);
    int s = abs(speed);
    set_tacho_speed_sp(left_motor,  s);
    set_tacho_speed_sp(right_motor, s);
    set_tacho_position_sp(left_motor,  wheel_deg);
    set_tacho_position_sp(right_motor, -wheel_deg);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    wait_by_degrees(s, wheel_deg);
}

void pivot_turn(int speed, int degrees, int direction) {
    int wheel_deg = robot_to_wheel_deg(degrees, 2.0);
    int s = abs(speed);
    uint8_t motor = (direction == 1) ? right_motor : left_motor;
    set_tacho_speed_sp(motor, s * ((wheel_deg >= 0) ? 1 : -1));
    set_tacho_position_sp(motor, wheel_deg);
    set_tacho_command_inx(motor, TACHO_RUN_TO_REL_POS);
    wait_by_degrees(s, wheel_deg);
}

void arc_turn(int outer_speed, float ratio, int duration_ms) {
    if (ratio < 0.0f || ratio > 1.0f) return;
    int inner_speed = (int)(outer_speed * ratio);
    set_tacho_speed_sp(left_motor,  outer_speed);
    set_tacho_speed_sp(right_motor, inner_speed);
    set_tacho_time_sp(left_motor,  duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    wait_by_duration(duration_ms);
}

void stop_motors(void) {
    set_tacho_command_inx(left_motor,  TACHO_STOP);
    set_tacho_command_inx(right_motor, TACHO_STOP);
}

void print_motor_stats(void) {
    int posL = 0, posR = 0, spdL = 0, spdR = 0;
    get_tacho_position(left_motor,  &posL);
    get_tacho_position(right_motor, &posR);
    get_tacho_speed(left_motor,     &spdL);
    get_tacho_speed(right_motor,    &spdR);
    printf("Left motor:  %d deg, %d deg/s\n", posL, spdL);
    printf("Right motor: %d deg, %d deg/s\n", posR, spdR);
}
