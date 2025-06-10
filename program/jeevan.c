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

// Geometry of the robot (millimeters)
// Wheel diameter measured across tire
#define WHEEL_DIAMETER_MM 50
// Distance between the two wheels (axle length)
#define WHEEL_BASE_MM     104.0

// Should gyro be automatically reset when initializing?
static bool gyro_auto_reset = true;

void set_gyro_auto_reset(bool enable) {
    gyro_auto_reset = enable;
}

static void reset_gyro(uint8_t sn_gyro) {
    set_sensor_mode(sn_gyro, "GYRO-RATE");
    Sleep(100);
    set_sensor_mode(sn_gyro, "GYRO-ANG");
    Sleep(100);
}

// Convert desired robot rotation (in degrees) to wheel rotation (in degrees) for a tank turn:
//   wheel_degrees = (robot_degrees * wheel_base) / wheel_diameter
static int robot_to_tank_wheel_deg(int robot_deg) {
    return (int)((double)robot_deg * WHEEL_BASE_MM / WHEEL_DIAMETER_MM);
}

// Convert desired robot rotation (in degrees) to wheel rotation (in degrees) for a pivot turn about one wheel:
//   wheel_degrees = (robot_degrees * 2 * wheel_base) / wheel_diameter
static int robot_to_pivot_wheel_deg(int robot_deg) {
    return (int)((double)robot_deg * 2.0 * WHEEL_BASE_MM / WHEEL_DIAMETER_MM);
}

// ---- BUTTON METHODS ----

const char* get_button_name(uint8_t keys) {
    if (keys & EV3_KEY_UP)     return "UP";
    if (keys & EV3_KEY_DOWN)   return "DOWN";
    if (keys & EV3_KEY_LEFT)   return "LEFT";
    if (keys & EV3_KEY_RIGHT)  return "RIGHT";
    if (keys & EV3_KEY_CENTER) return "CENTER";
    if (keys & EV3_KEY_BACK)   return "BACK";
    return NULL;
}

bool is_button_pressed(uint8_t button_mask) {
    uint8_t keys = 0;
    ev3_read_keys(&keys);
    return (keys & button_mask) != 0;
}

// ---- COLOR SENSOR METHODS ----

static const char* color_names[] = {
    "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN"
};
#define COLOR_COUNT ((int)(sizeof(color_names) / sizeof(color_names[0])))

void read_color_sensors(uint8_t sn1, uint8_t sn2, int* val1, int* val2) {
    int c1 = 0, c2 = 0;
    if (!get_sensor_value(0, sn1, &c1) || c1 < 0 || c1 >= COLOR_COUNT) {
        c1 = 0;
    }
    if (!get_sensor_value(0, sn2, &c2) || c2 < 0 || c2 >= COLOR_COUNT) {
        c2 = 0;
    }
    *val1 = c1;
    *val2 = c2;
}

// ---- GYRO SENSOR METHODS ----

bool init_gyro(uint8_t* sn_gyro, bool reset) {
    if (ev3_search_sensor(LEGO_EV3_GYRO, sn_gyro, 0)) {
        if (reset || gyro_auto_reset) {
            reset_gyro(*sn_gyro);
        }
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

static uint8_t left_motor  = DESC_LIMIT;
static uint8_t right_motor = DESC_LIMIT;

bool init_motors(void) {
    ev3_tacho_init();
    int found = 0;
    for (int i = 0; i < DESC_LIMIT && found < 2; i++) {
        if (ev3_tacho[i].type_inx == LEGO_EV3_L_MOTOR) {
            if (found == 0) {
                left_motor = i;
            } else {
                right_motor = i;
            }
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
    Sleep(duration_ms + 200);
}

void move_for_degrees(int speed, int degrees) {
    set_speed(speed);
    set_tacho_position_sp(left_motor,  degrees);
    set_tacho_position_sp(right_motor, degrees);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    int wait = (speed != 0) ? ((abs(degrees) * 1000) / abs(speed)) + 200 : 1000;
    Sleep(wait);
}

// Tank turn: both wheels rotate in opposite directions to pivot in place.
// 'degrees' is the robot‐centric rotation angle (degrees). Positive = clockwise.
// 'speed' is the wheel speed (degrees per second). Use nonzero speed.
void tank_turn(int speed, int degrees) {
    int wheel_deg = robot_to_tank_wheel_deg(degrees);
    int s = abs(speed);
    // Left wheel forward, right wheel backward
    set_tacho_speed_sp(left_motor,  s);
    set_tacho_speed_sp(right_motor, s);
    set_tacho_position_sp(left_motor,  wheel_deg);
    set_tacho_position_sp(right_motor, -wheel_deg);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    int wait = (s != 0) ? ((abs(wheel_deg) * 1000) / s) + 500 : 1000;
    Sleep(wait);
}

// Pivot turn: rotate about a single wheel (direction = 1 for pivot around left wheel,
// direction = -1 for pivot around right wheel). 'degrees' is robot rotation in degrees.
void pivot_turn(int speed, int degrees, int direction) {
    int wheel_deg = robot_to_pivot_wheel_deg(degrees);
    int s = abs(speed);
    if (direction == 1) {
        // Pivot around left wheel: right wheel moves
        set_tacho_speed_sp(right_motor, s * ((wheel_deg >= 0) ? 1 : -1));
        set_tacho_position_sp(right_motor, wheel_deg);
        set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    } else if (direction == -1) {
        // Pivot around right wheel: left wheel moves
        set_tacho_speed_sp(left_motor, s * ((wheel_deg >= 0) ? 1 : -1));
        set_tacho_position_sp(left_motor, wheel_deg);
        set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    }
    int wait = (s != 0) ? ((abs(wheel_deg) * 1000) / s) + 500 : 1000;
    Sleep(wait);
}

// Arc turn: outer wheel runs at 'outer_speed', inner wheel at 'outer_speed * ratio' for
// 'duration_ms' milliseconds. Ratio ∈ [0,1].
void arc_turn(int outer_speed, float ratio, int duration_ms) {
    if (ratio < 0.0f || ratio > 1.0f) return;
    int inner_speed = (int)(outer_speed * ratio);
    set_tacho_speed_sp(left_motor,  outer_speed);
    set_tacho_speed_sp(right_motor, inner_speed);
    set_tacho_time_sp(left_motor,  duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    Sleep(duration_ms + 200);
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

// Rotate the robot a full 360° in place (tank turn) at the given wheel speed.
// Internally calls tank_turn with degrees = 360.
void rotate_robot_360(int speed) {
    tank_turn(speed, 360);
}
