#include <stdio.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"

// Include your updated sensor_methods.c
#include "sensor_methods.c"

// A small pause between tests (milliseconds)
#define PAUSE_BETWEEN_TESTS_MS 1000

int main(void) {
    // Initialize EV3 system
    if (ev3_init() < 1) {
        printf("EV3 init failed.\n");
        return 1;
    }
    ev3_sensor_init();
    ev3_tacho_init();

    // Initialize motors
    if (!init_motors()) {
        printf("Motor initialization failed.\n");
        ev3_uninit();
        return 1;
    }
    printf("Motors initialized successfully.\n");

    // 1) Test 360° tank turn (in place)
    printf("\n--- Testing tank_turn: 360° in place ---\n");
    // tank_turn(speed, robot_degrees)
    tank_turn(200, 360);
    stop_motors();
    Sleep(PAUSE_BETWEEN_TESTS_MS);

    /*// 2) Test pivot_turn: 360° around left wheel (direction = 1)
    printf("\n--- Testing pivot_turn: 360° around LEFT wheel ---\n");
    // pivot_turn(speed, robot_degrees, direction)
    pivot_turn(200, 360, 1);
    stop_motors();
    Sleep(PAUSE_BETWEEN_TESTS_MS);

    // 3) Test pivot_turn: 360° around right wheel (direction = -1)
    printf("\n--- Testing pivot_turn: 360° around RIGHT wheel ---\n");
    pivot_turn(200, 360, -1);
    stop_motors();
    Sleep(PAUSE_BETWEEN_TESTS_MS);

    // 4) Test rotate_robot_360 helper (calls tank_turn internally)
    printf("\n--- Testing rotate_robot_360 helper function ---\n");
    rotate_robot_360(200);
    stop_motors();
    Sleep(PAUSE_BETWEEN_TESTS_MS);

    // 5) Test arc_turn: gentle arc for 2 seconds
    printf("\n--- Testing arc_turn: outer_speed=200, ratio=0.5, duration=2000ms ---\n");
    // outer wheel at 200 deg/s, inner at 200 * 0.5 = 100 deg/s, run for 2000ms
    arc_turn(200, 0.5f, 2000);
    stop_motors();
    Sleep(PAUSE_BETWEEN_TESTS_MS); */

    // Optionally, print final motor stats
    printf("\n--- Final motor statistics ---\n");
    print_motor_stats();

    // Clean up
    ev3_uninit();
    printf("\nAll turn tests completed.\n");
    return 0;
}
