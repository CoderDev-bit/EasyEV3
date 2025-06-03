#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"

#define Sleep(ms) usleep((ms) * 1000)

// ---- BUTTON METHODS ----
const char* get_button_name(uint8_t keys) {
    if (keys & EV3_KEY_UP) return "UP";
    if (keys & EV3_KEY_DOWN) return "DOWN";
    if (keys & EV3_KEY_LEFT) return "LEFT";
    if (keys & EV3_KEY_RIGHT) return "RIGHT";
    if (keys & EV3_KEY_CENTER) return "CENTER";
    if (keys & EV3_KEY_BACK) return "BACK";
    return NULL;
}

bool is_button_pressed(uint8_t button_mask) {
    uint8_t keys = 0;
    ev3_read_keys(&keys);
    return keys & button_mask;
}

// ---- COLOR SENSOR METHODS ----
const char* color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT ((int)(sizeof(color) / sizeof(color[0])))

void read_color_sensors(uint8_t sn1, uint8_t sn2, int* val1, int* val2) {
    *val1 = 0; *val2 = 0;
    if (!get_sensor_value(0, sn1, val1) || *val1 < 0 || *val1 >= COLOR_COUNT) *val1 = 0;
    if (!get_sensor_value(0, sn2, val2) || *val2 < 0 || *val2 >= COLOR_COUNT) *val2 = 0;
}

// ---- GYRO SENSOR METHODS ----
bool init_gyro(uint8_t* sn_gyro, bool reset) {
    if (ev3_search_sensor(LEGO_EV3_GYRO, sn_gyro, 0)) {
        if (reset) {
            set_sensor_mode(*sn_gyro, "GYRO-RATE");
            Sleep(100);
            set_sensor_mode(*sn_gyro, "GYRO-ANG");
            Sleep(100);
        }
        return true;
    }
    return false;
}

bool get_gyro_angle(uint8_t sn_gyro, int* angle) {
    int raw;
    if (get_sensor_value(0, sn_gyro, &raw)) {
        *angle = -raw;
        return true;
    }
    return false;
}

// ---- ULTRASONIC SENSOR METHODS ----
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

// ---- MOTOR METHODS ----
uint8_t left_motor = DESC_LIMIT;
uint8_t right_motor = DESC_LIMIT;

bool init_motors() {
    ev3_tacho_init();
    int found = 0;
    for (int i = 0; i < DESC_LIMIT && found < 2; i++) {
        if (ev3_tacho[i].type_inx == LEGO_EV3_L_MOTOR) {
            if (found == 0) left_motor = i;
            else right_motor = i;
            found++;
        }
    }
    return found == 2;
}

void set_speed(int speed) {
    set_tacho_speed_sp(left_motor, speed);
    set_tacho_speed_sp(right_motor, speed);
}

void move_for_time(int speed, int duration_ms) {
    set_speed(speed);
    set_tacho_time_sp(left_motor, duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor, TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    Sleep(duration_ms + 200);
}

void move_for_degrees(int speed, int degrees) {
    set_speed(speed);
    set_tacho_position_sp(left_motor, degrees);
    set_tacho_position_sp(right_motor, degrees);
    set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    Sleep(1000);
}

void turn_in_place(int speed, int degrees) {
    set_tacho_speed_sp(left_motor, speed);
    set_tacho_speed_sp(right_motor, -speed);
    set_tacho_position_sp(left_motor, degrees);
    set_tacho_position_sp(right_motor, -degrees);
    set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    Sleep(1000);
}

void pivot_turn(int speed, int degrees, int direction) {
    if (direction == 1) {
        set_tacho_speed_sp(right_motor, speed);
        set_tacho_position_sp(right_motor, degrees);
        set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    } else if (direction == -1) {
        set_tacho_speed_sp(left_motor, speed);
        set_tacho_position_sp(left_motor, degrees);
        set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    }
    Sleep(1000);
}

void arc_turn(int outer_speed, float ratio, int duration_ms) {
    if (ratio < 0 || ratio > 1) return;
    int inner_speed = (int)(outer_speed * ratio);
    set_tacho_speed_sp(left_motor, outer_speed);
    set_tacho_speed_sp(right_motor, inner_speed);
    set_tacho_time_sp(left_motor, duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor, TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    Sleep(duration_ms + 200);
}

void stop_motors() {
    set_tacho_command_inx(left_motor, TACHO_STOP);
    set_tacho_command_inx(right_motor, TACHO_STOP);
}

void print_motor_stats() {
    int posL, posR, speedL, speedR;
    get_tacho_position(left_motor, &posL);
    get_tacho_position(right_motor, &posR);
    get_tacho_speed(left_motor, &speedL);
    get_tacho_speed(right_motor, &speedR);
    printf("Left: %d deg, %d deg/s\n", posL, speedL);
    printf("Right: %d deg, %d deg/s\n", posR, speedR);
}
