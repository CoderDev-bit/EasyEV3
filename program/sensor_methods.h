#ifndef SENSOR_METHODS_H
#define SENSOR_METHODS_H

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#ifndef Sleep
#define Sleep(ms) usleep((ms) * 1000)
#endif

// Geometry of the robot (millimeters)
// Wheel diameter measured across tire: 54.5 mm
#define WHEEL_DIAMETER_MM 54.5
// Distance from axle to the centre of the ball bearing pivot: 104 mm
#define WHEEL_BASE_MM 104

void set_gyro_auto_reset(bool enable);

const char* get_button_name(uint8_t keys);
bool is_button_pressed(uint8_t button_mask);
void read_color_sensors(uint8_t sn1, uint8_t sn2, int* val1, int* val2);

bool init_gyro(uint8_t* sn_gyro, bool reset);
bool get_gyro_angle(uint8_t sn_gyro, int* angle);

bool init_ultrasonic(uint8_t* sn_us);
bool get_distance_mm(uint8_t sn_us, int* distance_mm);

bool init_motors();
void set_speed(int speed);
void move_for_time(int speed, int duration_ms);
void move_for_degrees(int speed, int degrees);
void tank_turn(int speed, int degrees);
void pivot_turn(int speed, int degrees, int direction);
void arc_turn(int outer_speed, float ratio, int duration_ms);
void stop_motors();
void print_motor_stats();

int robot_to_tank_wheel_deg(int robot_deg);
int robot_to_pivot_wheel_deg(int robot_deg);

#endif // SENSOR_METHODS_H

