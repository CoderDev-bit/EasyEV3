#include <stdio.h>
#include "ev3.h"
#include "ev3_tacho.h"

#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(ms) usleep((ms) * 1000)
#endif

#define LEFT_PORT  OUTPUT_B
#define RIGHT_PORT OUTPUT_C
uint8_t left_motor, right_motor;
int max_speed;

// Helper to wait for motor to stop
void wait_for_motor(uint8_t sn) {
    FLAGS_T state;
    do {
        get_tacho_state_flags(sn, &state);
    } while (state);
}

// Move forward or backward a certain distance in degrees
void move(int degrees, int speed) {
    set_tacho_stop_action_inx(left_motor, TACHO_BRAKE);
    set_tacho_stop_action_inx(right_motor, TACHO_BRAKE);

    set_tacho_speed_sp(left_motor, speed);
    set_tacho_speed_sp(right_motor, speed);
    set_tacho_position_sp(left_motor, degrees);
    set_tacho_position_sp(right_motor, degrees);

    set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);

    wait_for_motor(left_motor);
    wait_for_motor(right_motor);
}

// Turn by rotating only one wheel (or in opposite directions)
void turn_left(int degrees, int speed) {
    set_tacho_speed_sp(left_motor, -speed);
    set_tacho_speed_sp(right_motor, speed);
    set_tacho_position_sp(left_motor, -degrees);
    set_tacho_position_sp(right_motor, degrees);

    set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);

    wait_for_motor(left_motor);
    wait_for_motor(right_motor);
}

void turn_right(int degrees, int speed) {
    set_tacho_speed_sp(left_motor, speed);
    set_tacho_speed_sp(right_motor, -speed);
    set_tacho_position_sp(left_motor, degrees);
    set_tacho_position_sp(right_motor, -degrees);

    set_tacho_command_inx(left_motor, TACHO_RUN_TO_REL_POS);
    set_tacho_command_inx(right_motor, TACHO_RUN_TO_REL_POS);

    wait_for_motor(left_motor);
    wait_for_motor(right_motor);
}

int main(void) {
    printf("Waiting for EV3 brick...\n");
    if (ev3_init() < 1) return (1);
    ev3_tacho_init();

    if (!ev3_search_tacho_plugged_in(LEFT_PORT, &left_motor, 0) ||
        !ev3_search_tacho_plugged_in(RIGHT_PORT, &right_motor, 0)) {
        printf("Motors not found. Check ports B and C.\n");
        return 1;
    }

    get_tacho_max_speed(left_motor, &max_speed);

    printf("*** Ready to Move! ***\n");

    // Example movement sequence
    move(720, max_speed / 2);        // Move forward
    turn_left(180, max_speed / 4);   // Turn left
    move(360, max_speed / 2);        // Move forward
    turn_right(180, max_speed / 4);  // Turn right
    move(-720, max_speed / 2);       // Move backward

    ev3_uninit();
    printf("*** Done! ***\n");
    return 0;
}