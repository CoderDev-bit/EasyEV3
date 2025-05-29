#include <stdio.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"

#define Sleep(ms) usleep((ms) * 1000)

uint8_t left_motor = DESC_LIMIT;
uint8_t right_motor = DESC_LIMIT;

// Initialize EV3 and auto-detect two LEGO_EV3_L_MOTOR motors
int init_motors() {
    if (ev3_init() == -1) return 0;
    ev3_tacho_init();

    int found = 0;
    for (int i = 0; i < DESC_LIMIT; i++) {
        if (ev3_tacho[i].type_inx != TACHO_TYPE__NONE_) {
            if (ev3_tacho[i].type_inx == LEGO_EV3_L_MOTOR) {
                if (found == 0) {
                    left_motor = i;
                    found++;
                } else if (found == 1) {
                    right_motor = i;
                    found++;
                    break;
                }
            }
        }
    }

    if (found < 2) {
        printf("Error: Less than 2 LEGO_EV3_L_MOTOR motors found.\n");
        return 0;
    }

    char port_name[32];
    printf("Left motor on port %s\n", ev3_tacho_port_name(left_motor, port_name));
    printf("Right motor on port %s\n", ev3_tacho_port_name(right_motor, port_name));
    return 1;
}

// Set speed (deg/sec)
void set_speed(int speed) {
    set_tacho_speed_sp(left_motor, speed);
    set_tacho_speed_sp(right_motor, speed);
}

// Move both motors for time (ms)
void move_for_time(int speed, int duration_ms) {
    set_speed(speed);
    set_tacho_time_sp(left_motor, duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);
    set_tacho_command_inx(left_motor, TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);
    Sleep(duration_ms + 200);
}

// Move for specified degrees
void move_for_degrees(int speed, int degrees) {
    set_speed(speed);
    set_tacho_position_sp(left_motor, degrees);
    set_tacho_position_sp(right_motor, degrees);
    set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    Sleep(1000);
}

// Turn in place
void turn_in_place(int speed, int degrees) {
    set_tacho_speed_sp(left_motor, speed);
    set_tacho_speed_sp(right_motor, -speed);
    set_tacho_position_sp(left_motor, degrees);
    set_tacho_position_sp(right_motor, -degrees);
    set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    Sleep(1000);
}

// Pivot turn: one wheel moves, the other stays still
// direction: 1 = pivot around left (right motor moves), -1 = pivot around right
void pivot_turn(int speed, int degrees, int direction) {
    if (direction == 1) {
        // Pivot around left wheel — right wheel moves
        set_tacho_speed_sp(right_motor, speed);
        set_tacho_position_sp(right_motor, degrees);
        set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);
    } else if (direction == -1) {
        // Pivot around right wheel — left wheel moves
        set_tacho_speed_sp(left_motor, speed);
        set_tacho_position_sp(left_motor, degrees);
        set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    } else {
        printf("Invalid pivot direction (use 1 or -1).\n");
        return;
    }

    Sleep(1000);
}

// Arc turn: both motors move forward but at different speeds
// outer_speed = speed of outer wheel
// ratio = 0 to 1 (inner wheel speed = outer_speed * ratio)
// duration_ms = how long to run
void arc_turn(int outer_speed, float ratio, int duration_ms) {
    if (ratio < 0 || ratio > 1) {
        printf("Invalid arc ratio (must be between 0 and 1).\n");
        return;
    }

    int inner_speed = (int)(outer_speed * ratio);

    set_tacho_speed_sp(left_motor, outer_speed);
    set_tacho_speed_sp(right_motor, inner_speed);

    set_tacho_time_sp(left_motor, duration_ms);
    set_tacho_time_sp(right_motor, duration_ms);

    set_tacho_command_inx(left_motor, TACHO_RUN_TIMED);
    set_tacho_command_inx(right_motor, TACHO_RUN_TIMED);

    Sleep(duration_ms + 200);
}


// Display stats
void print_motor_stats() {
    int posL, posR, speedL, speedR;
    get_tacho_position(left_motor, &posL);
    get_tacho_position(right_motor, &posR);
    get_tacho_speed(left_motor, &speedL);
    get_tacho_speed(right_motor, &speedR);
    printf("Left: %d deg, %d deg/s\n", posL, speedL);
    printf("Right: %d deg, %d deg/s\n", posR, speedR);
}

// Stop both motors
void stop_motors() {
    set_tacho_command_inx(left_motor, TACHO_STOP);
    set_tacho_command_inx(right_motor, TACHO_STOP);
}

int main() {
    if (!init_motors()) {
        printf("Motor initialization failed.\n");
        return 1;
    }

    printf("Motors initialized.\n");

    move_for_time(300, 2000);
    print_motor_stats();

    move_for_degrees(300, -360);
    print_motor_stats();

    turn_in_place(200, 180);
    print_motor_stats();

    // 🔁 Pivot turn: pivot around right wheel
    pivot_turn(200, 180, -1);
    print_motor_stats();

    // 🔁 Pivot turn: pivot around left wheel
    pivot_turn(200, 180, 1);
    print_motor_stats();

    // 🔃 Arc turn: smooth left curve
    arc_turn(300, 0.5, 2000);
    print_motor_stats();

    stop_motors();
    ev3_uninit();
    printf("Done.\n");

    return 0;
}

