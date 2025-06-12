#include <stdio.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "sensor_methods.c"

#define Sleep(ms) usleep((ms) * 1000)

// --- BACK Button Check ---
bool check_back_button() {
    uint8_t keys = 0;
    ev3_read_keys(&keys);
    return (keys & EV3_KEY_BACK);
}

// --- Helper Methods ---
void wait_and_print_sensor(const char* label, bool (*read_fn)(uint8_t, int*), uint8_t sensor) {
    for (int i = 0; i < 10; i++) {
        if (check_back_button()) {
            printf("Test skipped by BACK button.\n");
            return;
        }
        int value;
        if (read_fn(sensor, &value)) {
            printf("%s: %d\n", label, value);
        }
        Sleep(300);
    }
}

void wait_and_print_multiple_colors(uint8_t sensors[], int count) {
    for (int i = 0; i < 10; i++) {
        if (check_back_button()) {
            printf("Color sensor test skipped by BACK button.\n");
            return;
        }
        for (int j = 0; j < count; j++) {
            int value = 0;
            get_sensor_value(0, sensors[j], &value);
            if (value < 0 || value >= 8) value = 0;
            printf("Sensor %d: %s | ", j + 1, color_names[value]);
        }
        printf("\n");
        Sleep(300);
    }
}

// --- Device Testing Methods ---
void test_color_sensors() {
    printf("\n--- Testing Color Sensors ---\n");
    uint8_t color_sensors[4];
    int count = 0;

    for (int i = 0; i < DESC_LIMIT && count < 4; i++) {
        if (ev3_sensor[i].type_inx == LEGO_EV3_COLOR) {
            set_sensor_mode(i, "COL-COLOR");
            color_sensors[count++] = i;
        }
    }

    if (count > 0) {
        wait_and_print_multiple_colors(color_sensors, count);
    } else {
        printf("No color sensors found.\n");
    }
}

void test_buttons() {
    printf("\n--- Testing Buttons ---\nPress any button (BACK to skip)\n");
    while (1) {
        uint8_t keys = 0;
        ev3_read_keys(&keys);
        const char* name = get_button_name(keys);
        if (name) printf("Detected: %s\n", name);
        if (keys & EV3_KEY_BACK) break;
        Sleep(200);
    }
}

void test_motors() {
    printf("\n--- Testing Motors ---\n");
    if (check_back_button()) {
        printf("Motor test skipped.\n");
        return;
    }
    if (init_motors()) {
        printf("Motors initialized.\n");
        move_for_time(300, 1000);
        move_for_degrees(300, 360);
        tank_turn(200, 180);
        pivot_turn(200, 180, -1);
        pivot_turn(200, 180, 1);
        arc_turn(300, 0.5, 1000);
        print_motor_stats();
        stop_motors();
    } else {
        printf("Failed to initialize motors.\n");
    }
}

void test_gyro() {
    printf("\n--- Testing Gyro Sensor ---\n");
    if (check_back_button()) {
        printf("Gyro test skipped.\n");
        return;
    }
    uint8_t sn_gyro = SENSOR__NONE_;
    if (init_gyro(&sn_gyro, true)) {
        wait_and_print_sensor("Angle", get_gyro_angle, sn_gyro);
    } else {
        printf("Gyro not found.\n");
    }
}

void test_ultrasonic() {
    printf("\n--- Testing Ultrasonic Sensor ---\n");
    if (check_back_button()) {
        printf("Ultrasonic test skipped.\n");
        return;
    }
    uint8_t sn_us = SENSOR__NONE_;
    if (init_ultrasonic(&sn_us)) {
        wait_and_print_sensor("Distance", get_distance_mm, sn_us);
    } else {
        printf("Ultrasonic sensor not found.\n");
    }
}

void test_everything() {
    test_buttons();
    test_motors();
    test_gyro();
    test_ultrasonic();
    test_color_sensors();
}

// --- Complex Action ---
void rotate_robot_360() {
    if (ev3_init() < 1) {
        printf("EV3 init failed.\n");
        return;
    }
    ev3_sensor_init();
    ev3_tacho_init();

    if (init_motors()) {
        printf("Rotating robot 360 degrees using tank turn...\n");
        tank_turn(200, 360);
        stop_motors();
    } else {
        printf("Motor initialization failed.\n");
    }

    ev3_uninit();
}

int main() {
    test_everything();  // Change from just test_gyro() for completeness
    return 0;
}
