#include <stdio.h>
#include <unistd.h>
#include <limits.h>         // ← for INT_MAX
#include <stdint.h>         // ← for uint8_t
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "sensor_methods.h"


#define Sleep(ms) usleep((ms) * 1000)
#define MAX_SENSORS 4

// Motor handles from sensor_methods.c (must be non-static there)
extern uint8_t left_motor;
extern uint8_t right_motor;

// --- BACK Button Check ---
static bool check_back_button_once() {
    static bool was_pressed = false;
    if (is_button_pressed(EV3_KEY_BACK)) {
        if (!was_pressed) {
            was_pressed = true;
            return true;
        }
    } else {
        was_pressed = false;
    }
    return false;
}

static void wait_until_back_released() {
    printf("...release BACK button to continue.\n");
    while (is_button_pressed(EV3_KEY_BACK)) {
        Sleep(100);
    }
}

// --- Helper Methods ---
static void wait_and_print_sensor(const char* test_name, const char* label, bool (*read_fn)(uint8_t, int*), uint8_t sensor) {
    printf("Starting test. Press BACK to skip.\n");
    for (int i = 0; i < 25; i++) {
        if (check_back_button_once()) {
            printf("\n%s test skipped.\n", test_name);
            wait_until_back_released();
            return;
        }
        int value;
        if (read_fn(sensor, &value)) {
            printf("\r%s: %-5d", label, value);
            fflush(stdout);
        }
        Sleep(300);
    }
    printf("\n");
}

static void display_color_sensor_readings(uint8_t sensors[], int count) {
    printf("Starting test. Press BACK to skip.\n");
    for (int i = 0; i < 25; i++) {
        if (check_back_button_once()) {
            printf("\nColor sensor test skipped.\n");
            wait_until_back_released();
            return;
        }
        for (int j = 0; j < count; j++) {
            int value;
            if (get_color_value(sensors[j], &value)) {
                printf("Sensor %d: %-7s | ", j + 1, color_names[value]);
            } else {
                printf("Sensor %d: ERROR    | ", j + 1);
            }
        }
        Sleep(500);
        printf("\r");
        fflush(stdout);
    }
    printf("\n");
}

// --- Device Testing Methods ---
static void test_color_sensors() {
    printf("\n--- Testing Color Sensors ---\n");
    uint8_t color_sensors[MAX_SENSORS];
    int count = init_all_color_sensors(color_sensors, MAX_SENSORS);

    if (count > 0) {
        printf("Found %d color sensor(s).\n", count);
        display_color_sensor_readings(color_sensors, count);
    } else {
        printf("No color sensors found.\n");
    }
}

static void test_buttons() {
    printf("\n--- Testing Buttons ---\nPress buttons to see their names.\nPress BACK to finish this test.\n");
    while (true) {
        uint8_t keys;
        ev3_read_keys(&keys);
        if (keys) {
            const char* name = get_button_name(keys);
            if (name) {
                printf("Pressed: %s\n", name);
                if (keys & EV3_KEY_BACK) {
                    wait_until_back_released();
                    break;
                }
                while(is_button_pressed(keys)) {
                    Sleep(50);
                }
            }
        }
        Sleep(50);
    }
}

static void test_motors() {
    printf("\n--- Testing Motors ---\n");
    if (check_back_button_once()) {
        printf("Motor test skipped.\n");
        wait_until_back_released();
        return;
    }
    if (init_motors()) {
        printf("Motors initialized. Running sequence...\n");
        printf("Forward for 1s...\n");
        move_for_time(300, 1000);
        printf("Forward 360 degrees...\n");
        move_for_degrees(300, 360);
        printf("Tank turn 180 degrees...\n");
        tank_turn(200, 180);
        printf("Pivot turn left...\n");
        pivot_turn(200, 180, -1);
        printf("Pivot turn right...\n");
        pivot_turn(200, 180, 1);
        printf("Arc turn...\n");
        arc_turn(300, 0.5, 1000);
        print_motor_stats();
        stop_motors();
        printf("Motor test complete.\n");
    } else {
        printf("Failed to initialize motors. Check connections.\n");
    }
}

static void test_gyro() {
    printf("\n--- Testing Gyro Sensor ---\n");
    if (check_back_button_once()) {
        printf("Gyro test skipped.\n");
        wait_until_back_released();
        return;
    }
    uint8_t sn_gyro;
    if (init_gyro(&sn_gyro, true)) {
        printf("Gyro initialized. Resetting angle to 0.\n");
        wait_and_print_sensor("Gyro", "Angle", get_gyro_angle, sn_gyro);
    } else {
        printf("Gyro not found.\n");
    }
}

static void test_ultrasonic() {
    printf("\n--- Testing Ultrasonic Sensor ---\n");
    if (check_back_button_once()) {
        printf("Ultrasonic test skipped.\n");
        wait_until_back_released();
        return;
    }
    uint8_t sn_us;
    if (init_ultrasonic(&sn_us)) {
        printf("Ultrasonic sensor initialized.\n");
        wait_and_print_sensor("Ultrasonic", "Distance (mm)", get_distance_mm, sn_us);
    } else {
        printf("Ultrasonic sensor not found.\n");
    }
}

static void test_everything() {
    test_buttons();
    test_color_sensors();
    test_motors();
    test_gyro();
    test_ultrasonic();
}

// --- Compoud tests ---
static void test_360_scan() {
    printf("\n--- Testing 360° Scan ---\n");
    uint8_t sn_gyro, sn_us;

    if (!init_gyro(&sn_gyro, true)) {
        printf("Gyro sensor not found.\n");
        return;
    }
    if (!init_ultrasonic(&sn_us)) {
        printf("Ultrasonic sensor not found.\n");
        return;
    }
    if (!init_motors()) {
        printf("Motors not found.\n");
        return;
    }

    // Reset gyro to 0
    reset_gyro(sn_gyro);
    Sleep(1000);  // Allow reset to settle

    printf("Starting 360° scan. Press BACK to abort.\n");

    int min_dist = INT_MAX;
    int min_angle = 0;

    // Start rotation: clockwise
    set_tacho_speed_sp(left_motor,  200);
    set_tacho_speed_sp(right_motor, -200);
    set_tacho_command_inx(left_motor,  TACHO_RUN_FOREVER);
    set_tacho_command_inx(right_motor, TACHO_RUN_FOREVER);

    while (true) {
        if (check_back_button_once()) {
            printf("360° scan aborted.\n");
            wait_until_back_released();
            stop_motors();
            return;
        }

        int angle;
        if (get_gyro_angle(sn_gyro, &angle)) {
            int dist_cm;
            if (get_distance_mm(sn_us, &dist_cm) && dist_cm < 255) {
                int dist_mm = dist_cm * 10;
                if (dist_mm < min_dist) {
                    min_dist = dist_mm;
                    min_angle = angle;
                }
            }

            if (angle >= 360) break;
        }

        Sleep(30); // Smooth scan
    }

    stop_motors();

    if (min_dist < INT_MAX) {
        printf("Nearest object at %d°, %d mm away.\n", min_angle, min_dist);

        // Determine shortest turning direction
        int current_angle;
        get_gyro_angle(sn_gyro, &current_angle);
        int turn_deg = (min_angle - current_angle + 360) % 360;
        if (turn_deg > 180) turn_deg -= 360;

        tank_turn(200, turn_deg);
        printf("Moving towards object...\n");

        while (true) {
            int dist_cm;
            if (get_distance_mm(sn_us, &dist_cm) && dist_cm * 10 > 50) {
                move_for_time(200, 200); // Move forward in small steps
            } else {
                break;
            }
        }

        stop_motors();
        printf("Reached object.\n");
    } else {
        printf("No object detected within range.\n");
    }
}

static void forward_until_black() {
    printf("--- Moving Forward Until Black Detected ---\n"); // Added newline
    uint8_t color_sensors[MAX_SENSORS];
    int count = init_all_color_sensors(color_sensors, MAX_SENSORS);
    if (count < 1) {
        printf("No color sensor found.\n"); // Added newline
        return;
    }
    // Initialize motors
    if (!init_motors()) { // Added motor initialization check
        printf("Failed to initialize motors.\n"); // Added newline
        return;
    }

    uint8_t sn_color = color_sensors[0];

    printf("Moving forward. Press BACK to abort.\n"); // Added newline
    // drive forward
    set_tacho_speed_sp(left_motor,  200);
    set_tacho_speed_sp(right_motor, 200);
    set_tacho_command_inx(left_motor,  TACHO_RUN_FOREVER);
    set_tacho_command_inx(right_motor, TACHO_RUN_FOREVER);

    // wait for black (color code 1)
    while (true) {
        if (check_back_button_once()) {
            printf("Forward-until-black aborted.\n"); // Added newline
            stop_motors();
            wait_until_back_released();
            return;
        }
        int color;
        if (get_color_value(sn_color, &color) && color == 1) {
            printf("Black detected. Stopping.\n"); // Added newline
            break;
        }
        Sleep(50);
    }
    stop_motors();
}



int main() {
    printf("============================\n");
    printf("   EV3 Hardware Test Suite  \n");
    printf("============================\n");
    printf("Press the BACK button at the start of any test to skip it.\n");

    if (ev3_init() < 1) {
        printf("Error: ev3_init failed. Is the ev3dev daemon running?\n");
        return 1;
    }
    
    ev3_sensor_init();
    ev3_tacho_init();

    forward_until_black();

    ev3_uninit();
    printf("\nTest suite finished.\n");
    return 0;
}