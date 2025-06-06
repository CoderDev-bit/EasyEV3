#include <stdio.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"

// Include sensor_methods.c directly for this test
#include "sensor_methods.c"

int main() {
    printf("Initializing EV3 system...\n");
    if (ev3_init() < 1) {
        printf("EV3 init failed.\n");
        return 1;
    }
    ev3_sensor_init();
    ev3_tacho_init();

    // --- Test Buttons ---
    printf("\n--- Testing Buttons ---\nPress any button (BACK to skip)\n");
    while (1) {
        uint8_t keys = 0;
        ev3_read_keys(&keys);
        const char* name = get_button_name(keys);
        if (name) printf("Detected: %s\n", name);
        if (keys & EV3_KEY_BACK) break;
        Sleep(200);
    }

    // --- Test Motors ---
    printf("\n--- Testing Motors ---\n");
    if (init_motors()) {
        printf("Motors initialized.\n");
        move_for_time(300, 1000);
        move_for_degrees(300, 360);
        turn_in_place(200, 180);
        pivot_turn(200, 180, -1);
        pivot_turn(200, 180, 1);
        arc_turn(300, 0.5, 1000);
        print_motor_stats();
        stop_motors();
    } else {
        printf("Failed to initialize motors.\n");
    }

    // --- Test Gyro ---
    printf("\n--- Testing Gyro Sensor ---\n");
    uint8_t sn_gyro = SENSOR__NONE_;
    if (init_gyro(&sn_gyro, true)) {
        for (int i = 0; i < 10; i++) {
            int angle;
            if (get_gyro_angle(sn_gyro, &angle)) {
                printf("Angle: %d degrees\n", angle);
            }
            Sleep(300);
        }
    } else {
        printf("Gyro not found.\n");
    }

    // --- Test Ultrasonic ---
    printf("\n--- Testing Ultrasonic Sensor ---\n");
    uint8_t sn_us = SENSOR__NONE_;
    if (init_ultrasonic(&sn_us)) {
        for (int i = 0; i < 10; i++) {
            int distance;
            if (get_distance_mm(sn_us, &distance)) {
                printf("Distance: %d mm\n", distance);
            }
            Sleep(300);
        }
    } else {
        printf("Ultrasonic sensor not found.\n");
    }

    // --- Test Color Sensors ---
    printf("\n--- Testing Color Sensors ---\n");
    uint8_t sn1 = SENSOR__NONE_, sn2 = SENSOR__NONE_;
    int found = 0;
    for (int i = 0; i < DESC_LIMIT && found < 2; i++) {
        if (ev3_sensor[i].type_inx == LEGO_EV3_COLOR) {
            if (found == 0) sn1 = i;
            else sn2 = i;
            found++;
        }
    }
    if (found == 2) {
        set_sensor_mode(sn1, "COL-COLOR");
        set_sensor_mode(sn2, "COL-COLOR");
        for (int i = 0; i < 10; i++) {
            int v1, v2;
            read_color_sensors(sn1, sn2, &v1, &v2);
            printf("Sensor1: %s, Sensor2: %s\n", color[v1], color[v2]);
            Sleep(300);
        }
    } else {
        printf("Color sensors not found.\n");
    }

    ev3_uninit();
    printf("\nAll tests complete.\n");
    return 0;
}
