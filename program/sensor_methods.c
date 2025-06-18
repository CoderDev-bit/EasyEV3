#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "sensor_methods.h"

#define Sleep(ms) usleep((ms) * 1000)
#define WHEEL_DIAMETER_MM 49.5
#define WHEEL_BASE_MM      104.0

const char* color_names[] = {
    "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN"
};
const int COLOR_COUNT = sizeof(color_names) / sizeof(color_names[0]);

static bool gyro_auto_reset = true;

uint8_t left_motor  = DESC_LIMIT;
uint8_t right_motor = DESC_LIMIT;

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

bool reset_gyro(uint8_t sn_gyro) {
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

// ---------- Color Sensor Methods (Revised) ----------
int init_all_color_sensors(uint8_t* sn_array, int max_sensors) {
    int count = 0;
    uint8_t sn;
    int i = 0;
    while (ev3_search_sensor(LEGO_EV3_COLOR, &sn, i++) && count < max_sensors) {
        set_sensor_mode(sn, "COL-COLOR");
        sn_array[count++] = sn;
    }
    return count;
}

bool get_color_value(uint8_t sn_color, int* value) {
    if (get_sensor_value(0, sn_color, value)) {
        if (*value >= 0 && *value < COLOR_COUNT) {
            return true;
        }
    }
    *value = 0;
    return false;
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

// ---------- Motor Methods (Revised init_motors) ----------
bool init_motors(void) {
    if (!ev3_search_tacho_plugged_in(LEGO_EV3_L_MOTOR, EV3_PORT__OUT_B, &left_motor, 0)) {
        printf("Left motor not found on port B\n");
        return false;
    }
    if (!ev3_search_tacho_plugged_in(LEGO_EV3_L_MOTOR, EV3_PORT__OUT_C, &right_motor, 0)) {
        printf("Right motor not found on port C\n");
        return false;
    }
    return true;
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
    wait_by_degrees(s, abs(wheel_deg));
}

void pivot_turn(int speed, int degrees, int direction) {
    int wheel_deg = robot_to_wheel_deg(degrees, 2.0);
    int s = abs(speed);
    uint8_t motor_to_move = (direction == 1) ? right_motor : left_motor;
    uint8_t motor_to_stop = (direction == 1) ? left_motor : right_motor;

    set_tacho_speed_sp(motor_to_move, s);
    set_tacho_position_sp(motor_to_move, wheel_deg);
    set_tacho_command_inx(motor_to_stop, TACHO_STOP);
    set_tacho_command_inx(motor_to_move, TACHO_RUN_TO_REL_POS);
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