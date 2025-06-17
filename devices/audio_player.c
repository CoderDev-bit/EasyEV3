#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ev3.h"
#include "ev3_sound.h"

#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(msec) usleep((msec) * 1000)
#endif

#define NUM_FILES 8

const char* audio_files[NUM_FILES] = {
    "/home/robot/sounds/line_1_turning_left.wav",
    "/home/robot/sounds/line_2_turning_right.wav",
    "/home/robot/sounds/line_3_reversing.wav",
    "/home/robot/sounds/line_4_black_detected.wav",
    "/home/robot/sounds/line_5_forward.wav",
    "/home/robot/sounds/line_6_white_detected.wav",
    "/home/robot/sounds/line_7_arrived_at_destination.wav",
    "/home/robot/sounds/line_8_starting_trip.wav"
};

// Your existing button-name function
const char* get_button_name(uint8_t keys) {
    if (keys & EV3_KEY_UP) return "UP";
    if (keys & EV3_KEY_DOWN) return "DOWN";
    if (keys & EV3_KEY_LEFT) return "LEFT";
    if (keys & EV3_KEY_RIGHT) return "RIGHT";
    if (keys & EV3_KEY_CENTER) return "CENTER";
    if (keys & EV3_KEY_BACK) return "BACK";
    return NULL;
}

int main(void) {
    printf("Initializing EV3...\n");
    if (ev3_init() < 1) {
        printf("Failed to initialize EV3 system.\n");
        return 1;
    }

    uint8_t current_keys = 0;
    uint8_t previous_keys = 0;
    int index = 0;

    printf("*** Audio Player Started ***\n");
    printf("Press CENTER to play next sound, BACK to exit.\n");

    while (1) {
        ev3_read_keys(&current_keys);
        uint8_t pressed_now = current_keys & ~previous_keys;

        // Play sound on CENTER press
        if (pressed_now & EV3_KEY_CENTER) {
            printf("Playing: %s\n", audio_files[index]);
            ev3_sound_file(audio_files[index], 100); // 100 = max volume

            index = (index + 1) % NUM_FILES;
        }

        // Exit on BACK press
        if (current_keys & EV3_KEY_BACK) {
            printf("BACK button pressed. Exiting.\n");
            break;
        }

        previous_keys = current_keys;
        Sleep(50); // debounce
    }

    ev3_uninit();
    printf("*** Audio Player Ended ***\n");
    return 0;
}
