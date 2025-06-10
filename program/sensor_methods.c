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
#define WHEEL_BASE_MM      104.0

// Should gyro be automatically reset when initializing?
static bool gyro_auto_reset = true;

/**
 * @brief Enables or disables the automatic reset of the gyro sensor upon initialization.
 *
 * @param enable A boolean value: true to enable auto-reset, false to disable.
 */
void set_gyro_auto_reset(bool enable) {
    gyro_auto_reset = enable;
}

/**
 * @brief Resets the gyro sensor. This involves changing its mode to 'GYRO-RATE'
 * and then back to 'GYRO-ANG' to re-calibrate it.
 *
 * @param sn_gyro The sensor number (address) of the gyro sensor.
 */
static void reset_gyro(uint8_t sn_gyro) {
    set_sensor_mode(sn_gyro, "GYRO-RATE");
    Sleep(100);
    set_sensor_mode(sn_gyro, "GYRO-ANG");
    Sleep(100);
}

/**
 * @brief Converts a desired robot rotation angle for a tank turn into the
 * equivalent wheel rotation angle. In a tank turn, both wheels rotate
 * in opposite directions.
 *
 * @param robot_deg The desired rotation of the robot in degrees.
 * @return The calculated rotation for each wheel in degrees.
 */
static int robot_to_tank_wheel_deg(int robot_deg) {
    return (int)((double)robot_deg * WHEEL_BASE_MM / WHEEL_DIAMETER_MM);
}

/**
 * @brief Converts a desired robot rotation angle for a pivot turn into the
 * equivalent wheel rotation angle. In a pivot turn, one wheel stays
 * stationary while the other rotates.
 *
 * @param robot_deg The desired rotation of the robot in degrees.
 * @return The calculated rotation for the moving wheel in degrees.
 */
static int robot_to_pivot_wheel_deg(int robot_deg) {
    return (int)((double)robot_deg * 2.0 * WHEEL_BASE_MM / WHEEL_DIAMETER_MM);
}

// ---- BUTTON METHODS ----

/**
 * @brief Returns the string name of the pressed button based on the provided key mask.
 *
 * @param keys A bitmask representing the currently pressed EV3 buttons (e.g., EV3_KEY_UP, EV3_KEY_DOWN).
 * @return A constant string representing the button name, or NULL if no recognized button is pressed.
 */
const char* get_button_name(uint8_t keys) {
    if (keys & EV3_KEY_UP)      return "UP";
    if (keys & EV3_KEY_DOWN)    return "DOWN";
    if (keys & EV3_KEY_LEFT)    return "LEFT";
    if (keys & EV3_KEY_RIGHT)   return "RIGHT";
    if (keys & EV3_KEY_CENTER) return "CENTER";
    if (keys & EV3_KEY_BACK)    return "BACK";
    return NULL;
}

/**
 * @brief Checks if a specific button is currently pressed.
 *
 * @param button_mask The bitmask of the button to check (e.g., EV3_KEY_CENTER).
 * @return True if the specified button is pressed, false otherwise.
 */
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

/**
 * @brief Reads the color values from two EV3 color sensors.
 *
 * @param sn1 The sensor number (address) of the first color sensor.
 * @param sn2 The sensor number (address) of the second color sensor.
 * @param val1 A pointer to an integer where the color value of the first sensor will be stored.
 * The value corresponds to an index in the `color_names` array.
 * @param val2 A pointer to an integer where the color value of the second sensor will be stored.
 * The value corresponds to an index in the `color_names` array.
 */
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

/**
 * @brief Initializes the gyro sensor.
 *
 * @param sn_gyro A pointer to an `uint8_t` where the sensor number (address) of the found gyro sensor will be stored.
 * @param reset A boolean value: true to reset the gyro sensor after initialization, false to skip reset.
 * @return True if the gyro sensor is successfully found and initialized, false otherwise.
 */
bool init_gyro(uint8_t* sn_gyro, bool reset) {
    if (ev3_search_sensor(LEGO_EV3_GYRO, sn_gyro, 0)) {
        if (reset || gyro_auto_reset) {
            reset_gyro(*sn_gyro);
        }
        return true;
    }
    return false;
}

/**
 * @brief Retrieves the current angle from the gyro sensor.
 *
 * @param sn_gyro The sensor number (address) of the gyro sensor.
 * @param angle A pointer to an integer where the current angle in degrees will be stored.
 * @return True if the angle is successfully read, false otherwise.
 */
bool get_gyro_angle(uint8_t sn_gyro, int* angle) {
    int raw = 0;
    if (get_sensor_value(0, sn_gyro, &raw)) {
        *angle = -raw; // Gyro angle might be inverted depending on sensor orientation
        return true;
    }
    return false;
}

// ---- ULTRASONIC SENSOR METHODS ----

/**
 * @brief Initializes the ultrasonic sensor.
 *
 * @param sn_us A pointer to an `uint8_t` where the sensor number (address) of the found ultrasonic sensor will be stored.
 * @return True if the ultrasonic sensor is successfully found and initialized, false otherwise.
 */
bool init_ultrasonic(uint8_t* sn_us) {
    if (ev3_search_sensor(LEGO_EV3_US, sn_us, 0)) {
        set_sensor_mode(*sn_us, "US-DIST-CM"); // Set mode to measure distance in centimeters
        return true;
    }
    return false;
}

/**
 * @brief Retrieves the distance measured by the ultrasonic sensor in millimeters.
 *
 * @param sn_us The sensor number (address) of the ultrasonic sensor.
 * @param distance_mm A pointer to an integer where the measured distance in millimeters will be stored.
 * @return True if the distance is successfully read, false otherwise.
 */
bool get_distance_mm(uint8_t sn_us, int* distance_mm) {
    return get_sensor_value(0, sn_us, distance_mm);
}

// ---- MOTOR METHODS ----

static uint8_t left_motor  = DESC_LIMIT;
static uint8_t right_motor = DESC_LIMIT;

/**
 * @brief Initializes the EV3 motors and assigns them to the left and right motor variables.
 * It searches for two large EV3 motors and assumes the first found is the left
 * and the second found is the right.
 *
 * @return True if two large motors are successfully found and assigned, false otherwise.
 */
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

/**
 * @brief Sets the speed of both left and right motors.
 *
 * @param speed The desired motor speed in tacho degrees per second. A positive value
 * moves the motor forward, a negative value moves it backward.
 */
void set_speed(int speed) {
    set_tacho_speed_sp(left_motor,  speed);
    set_tacho_speed_sp(right_motor, speed);
}

/**
 * @brief Moves both motors forward or backward for a specified duration at a given speed.
 *
 * @param speed The desired motor speed in tacho degrees per second.
 * @param duration_ms The duration of the movement in milliseconds.
 */
void move_for_time(int speed, int duration_ms) {
    set_speed(speed);
    set_tacho_time_sp(left_motor,  duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    Sleep(duration_ms + 200); // Add a small buffer for motor ramp-down
}

/**
 * @brief Moves both motors forward or backward for a specified number of degrees at a given speed.
 *
 * @param speed The desired motor speed in tacho degrees per second.
 * @param degrees The desired rotation of the wheels in degrees. Positive for forward, negative for backward.
 */
void move_for_degrees(int speed, int degrees) {
    set_speed(speed);
    set_tacho_position_sp(left_motor,  degrees);
    set_tacho_position_sp(right_motor, degrees);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    int wait = (speed != 0) ? ((abs(degrees) * 1000) / abs(speed)) + 200 : 1000; // Calculate approximate wait time
    Sleep(wait);
}

/**
 * @brief Performs a tank turn, where both wheels rotate in opposite directions to pivot the robot in place.
 *
 * @param speed The absolute speed of the wheels in tacho degrees per second. This speed will be applied
 * to both motors, with one rotating forward and the other backward.
 * @param degrees The desired rotation angle of the robot in degrees. Positive values typically mean
 * clockwise rotation, negative for counter-clockwise.
 */
void tank_turn(int speed, int degrees) {
    int wheel_deg = robot_to_tank_wheel_deg(degrees);
    int s = abs(speed);
    // Left wheel forward, right wheel backward
    set_tacho_speed_sp(left_motor,  s);
    set_tacho_speed_sp(right_motor, s);
    set_tacho_position_sp(left_motor,  wheel_deg);
    set_tacho_position_sp(right_motor, -wheel_deg); // Right wheel rotates in opposite direction
    set_tacho_command_inx(left_motor,  TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    int wait = (s != 0) ? ((abs(wheel_deg) * 1000) / s) + 500 : 1000; // Calculate approximate wait time
    Sleep(wait);
}

/**
 * @brief Performs a pivot turn, where the robot rotates around a single stationary wheel.
 *
 * @param speed The absolute speed of the moving wheel in tacho degrees per second.
 * @param degrees The desired rotation angle of the robot in degrees.
 * @param direction An integer indicating the pivot direction: 1 for pivoting around the left wheel (right wheel moves),
 * -1 for pivoting around the right wheel (left wheel moves).
 */
void pivot_turn(int speed, int degrees, int direction) {
    int wheel_deg = robot_to_pivot_wheel_deg(degrees);
    int s = abs(speed);
    if (direction == 1) {
        // Pivot around left wheel: right wheel moves
        set_tacho_speed_sp(right_motor, s * ((wheel_deg >= 0) ? 1 : -1)); // Adjust speed direction based on angle
        set_tacho_position_sp(right_motor, wheel_deg);
        set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    } else if (direction == -1) {
        // Pivot around right wheel: left wheel moves
        set_tacho_speed_sp(left_motor, s * ((wheel_deg >= 0) ? 1 : -1)); // Adjust speed direction based on angle
        set_tacho_position_sp(left_motor, wheel_deg);
        set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    }
    int wait = (s != 0) ? ((abs(wheel_deg) * 1000) / s) + 500 : 1000; // Calculate approximate wait time
    Sleep(wait);
}

/**
 * @brief Performs an arc turn, where the robot moves in a curved path. One wheel runs at
 * 'outer_speed' and the other at 'outer_speed * ratio'.
 *
 * @param outer_speed The speed of the outer wheel in tacho degrees per second.
 * @param ratio A float between 0.0 and 1.0 (inclusive) representing the speed ratio of the inner wheel
 * to the outer wheel. A ratio of 0 means the inner wheel is stationary (pivot turn),
 * and a ratio of 1 means both wheels move at the same speed (straight line).
 * @param duration_ms The duration of the arc turn in milliseconds.
 */
void arc_turn(int outer_speed, float ratio, int duration_ms) {
    if (ratio < 0.0f || ratio > 1.0f) return; // Ensure ratio is within valid range
    int inner_speed = (int)(outer_speed * ratio);
    set_tacho_speed_sp(left_motor,  outer_speed);
    set_tacho_speed_sp(right_motor, inner_speed);
    set_tacho_time_sp(left_motor,  duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor,  TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    Sleep(duration_ms + 200); // Add a small buffer for motor ramp-down
}

/**
 * @brief Stops both the left and right motors immediately.
 */
void stop_motors(void) {
    set_tacho_command_inx(left_motor,  TACHO_STOP);
    set_tacho_command_inx(right_motor, TACHO_STOP);
}

/**
 * @brief Prints the current position (in degrees) and speed (in degrees per second)
 * of both the left and right motors to the console.
 */
void print_motor_stats(void) {
    int posL = 0, posR = 0, spdL = 0, spdR = 0;
    get_tacho_position(left_motor,  &posL);
    get_tacho_position(right_motor, &posR);
    get_tacho_speed(left_motor,      &spdL);
    get_tacho_speed(right_motor,     &spdR);
    printf("Left motor:  %d deg, %d deg/s\n", posL, spdL);
    printf("Right motor: %d deg, %d deg/s\n", posR, spdR);
}

/**
 * @brief Rotates the robot a full 360 degrees in place using a tank turn.
 *
 * @param speed The absolute speed of the wheels in tacho degrees per second during the turn.
 */
void rotate_robot_360(int speed) {
    tank_turn(speed, 360);
}