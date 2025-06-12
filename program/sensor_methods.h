#ifndef SENSOR_METHODS_H
#define SENSOR_METHODS_H

#include <stdbool.h>
#include <stdint.h>
#include "ev3.h"

// --- Shared Constants ---
extern const char* color_names[];
extern const int COLOR_COUNT;

// --- Gyro Sensor Methods ---
void set_gyro_auto_reset(bool enable);
bool init_gyro(uint8_t* sn_gyro, bool reset);
bool get_gyro_angle(uint8_t sn_gyro, int* angle);

// --- Button Methods ---
const char* get_button_name(uint8_t keys);
bool is_button_pressed(uint8_t button_mask);

// --- Color Sensor Methods ---
int init_all_color_sensors(uint8_t* sn_array, int max_sensors);
bool get_color_value(uint8_t sn_color, int* value);

// --- Ultrasonic Sensor Methods ---
bool init_ultrasonic(uint8_t* sn_us);
bool get_distance_mm(uint8_t sn_us, int* distance_mm);

// --- Motor Methods ---
bool init_motors(void);
void set_speed(int speed);
void move_for_time(int speed, int duration_ms);
void move_for_degrees(int speed, int degrees);
void tank_turn(int speed, int degrees);
void pivot_turn(int speed, int degrees, int direction);
void arc_turn(int outer_speed, float ratio, int duration_ms);
void stop_motors(void);
void print_motor_stats(void);

#endif // SENSOR_METHODS_H