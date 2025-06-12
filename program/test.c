#include <stdio.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "sensor_methods.h"

#define Sleep(ms) usleep((ms) * 1000)
#define MAX_SENSORS 4

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
    printf("\n--- 360° Environmental Scan ---\n");

    if (check_back_button_once()) {
        printf("Scan test skipped.\n");
        wait_until_back_released();
        return;
    }

    uint8_t sn_gyro, sn_us;
    if (!init_gyro(&sn_gyro, true)) {
        printf("Gyro not found.\n");
        return;
    }
    if (!init_ultrasonic(&sn_us)) {
        printf("Ultrasonic sensor not found.\n");
        return;
    }
    if (!init_motors()) {
        printf("Motors not initialized.\n");
        return;
    }

    printf("Starting 360° turn and scan...\n");
    reset_gyro(sn_gyro);  // Ensure starting from angle 0

    // Begin 360° tank turn at moderate speed
    int speed = 100;
    int degrees = 360;
    int sampling_delay_ms = 100;

    // Start the turn
    tank_turn(speed, degrees);

    // While turning, collect (angle, distance) pairs
    while (true) {
        int state_l = 0, state_r = 0;
        get_tacho_state_flags(left_motor,  &state_l);
        get_tacho_state_flags(right_motor, &state_r);
        if ((state_l & TACHO_RUNNING) == 0 && (state_r & TACHO_RUNNING) == 0) break;

        int angle = 0, distance = 0;
        if (get_gyro_angle(sn_gyro, &angle) && get_distance_mm(sn_us, &distance)) {
            if (distance < 2550) {
                printf("Angle: %3d°, Distance: %4d mm\n", angle, distance);
            }
        }

        Sleep(sampling_delay_ms);
    }

    stop_motors();
    printf("--- Scan complete ---\n");
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

    test_everything();

    ev3_uninit();
    printf("\nTest suite finished.\n");
    return 0;
}