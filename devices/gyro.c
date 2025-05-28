#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(ms) usleep((ms) * 1000)
#endif

int main(void) {
    char s[256];
    int angle_raw;
    int angle_corrected;
    uint8_t sn_gyro = SENSOR__NONE_;
    bool should_reset = true;  // CHANGE TO false if you want to skip reset
    int angle_offset = 0;

    printf("Waiting for EV3 brick...\n");
    if (ev3_init() < 1) return 1;

    printf("*** ( EV3 ) Hello! ***\n");
    ev3_sensor_init();

    // Search for the gyro sensor
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro, 0)) {
        printf("GYRO sensor found.\n");

        if (should_reset) {
            // Gyro reset: switch to rate mode, then angle mode
            set_sensor_mode(sn_gyro, "GYRO-RATE");
            Sleep(100);
            set_sensor_mode(sn_gyro, "GYRO-ANG");
            Sleep(100);
            printf("GYRO sensor reset to zero.\n");
        }

    } else {
        printf("GYRO sensor NOT found.\n");
        ev3_uninit();
        return 1;
    }

    printf("Reading gyro angle (CCW = +, CW = -), Ctrl+C to stop...\n");
    while (1) {
        if (get_sensor_value(0, sn_gyro, &angle_raw)) {
            // Invert the angle to match correct direction: CCW = positive, CW = negative
            angle_corrected = -angle_raw;
            printf("\rGyro angle: %+d degrees ", angle_corrected);
            fflush(stdout);
        } else {
            printf("\rGyro read error!         ");
            fflush(stdout);
        }
        Sleep(200);
    }

    ev3_uninit();
    return 0;
}
