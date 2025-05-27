#include <string.h>
#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

// Platform compatibility
#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif

// Check for exit signal from touch sensor or UP button
static bool _check_pressed(uint8_t sn_touch) {
    int val;
    if (sn_touch == SENSOR__NONE_) {
        return (ev3_read_keys((uint8_t*)&val) && (val & EV3_KEY_UP));
    }
    return (get_sensor_value(0, sn_touch, &val) && (val != 0));
}

int main(void) {
    int val;
    uint8_t sn_touch = SENSOR__NONE_;
    uint8_t sn_us = SENSOR__NONE_; // Ultrasonic sensor

    printf("Waiting for the EV3 brick to be online...\n");
    if (ev3_init() < 1) return (1);
    printf("*** ( EV3 ) Hello! ***\n");

    ev3_sensor_init();

    // Search for sensors
    ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0);
    if (ev3_search_sensor(LEGO_EV3_US, &sn_us, 0)) {
        printf("Ultrasonic sensor is found. Reading distance...\n");
        set_sensor_mode(sn_us, "US-DIST-CM"); // Set mode to centimeters

        while (!_check_pressed(sn_touch)) {
            if (get_sensor_value(0, sn_us, &val)) {
                printf("\rDistance: %d mm    ", val);
                fflush(stdout);
            } else {
                printf("\rError reading sensor.");
                fflush(stdout);
            }
            Sleep(200);
        }
    } else {
        printf("Ultrasonic sensor NOT found.\n");
        while (!_check_pressed(sn_touch)) Sleep(100);
    }

    ev3_uninit();
    printf("\n*** ( EV3 ) Bye! ***\n");
    return (0);
}