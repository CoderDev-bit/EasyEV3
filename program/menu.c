#include <stdio.h>
#include <string.h>
#include "ev3.h"

#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(msec) usleep((msec) * 1000)
#endif

// Define available modes
const char* modes[] = {"Mode A", "Mode B"};
int mode_count = 2;

// Function prototypes
void display_menu(int selected_index);
void on_up_pressed(int* selected_index);
void on_down_pressed(int* selected_index);
void on_center_pressed(int selected_index);
void on_back_pressed();

// Mode actions (placeholders for actual behavior)
void start_mode_a();
void start_mode_b();

// Array of function pointers for mode behavior
void (*mode_functions[])(void) = {start_mode_a, start_mode_b};

void display_menu(int selected_index) {
    printf("\n=== Select Robot Mode ===\n");
    for (int i = 0; i < mode_count; i++) {
        if (i == selected_index)
            printf("> %s <\n", modes[i]);
        else
            printf("  %s\n", modes[i]);
    }
}

void on_up_pressed(int* selected_index) {
    *selected_index = (*selected_index - 1 + mode_count) % mode_count;
    display_menu(*selected_index);
}

void on_down_pressed(int* selected_index) {
    *selected_index = (*selected_index + 1) % mode_count;
    display_menu(*selected_index);
}

void on_center_pressed(int selected_index) {
    printf("\nYou selected: %s\n", modes[selected_index]);
    printf("Starting %s...\n", modes[selected_index]);
    mode_functions[selected_index]();  // Call the corresponding mode function
}

void on_back_pressed() {
    printf("BACK button pressed. Exiting...\n");
}

// Sample robot actions for each mode
void start_mode_a() {
    printf("[Robot is running in Mode A...]\n");
    // Add robot behavior here
}

void start_mode_b() {
    printf("[Robot is running in Mode B...]\n");
    // Add robot behavior here
}

int main(void) {
    printf("Waiting for the EV3 brick to come online...\n");

    if (ev3_init() < 1) {
        printf("ERROR: Failed to initialize EV3 system.\n");
        return 1;
    }

    int selected_index = 0;
    uint8_t current_keys = 0;
    uint8_t previous_keys = 0;

    display_menu(selected_index);

    while (1) {
        ev3_read_keys(&current_keys);
        uint8_t pressed_now = current_keys & ~previous_keys;

        if (pressed_now != 0) {
            if (pressed_now & EV3_KEY_UP)        on_up_pressed(&selected_index);
            else if (pressed_now & EV3_KEY_DOWN) on_down_pressed(&selected_index);
            else if (pressed_now & EV3_KEY_CENTER) {
                on_center_pressed(selected_index);
                break;  // Exit after starting mode
            }
            else if (pressed_now & EV3_KEY_BACK) {
                on_back_pressed();
                break;
            }
        }

        previous_keys = current_keys;
        Sleep(100);
    }

    ev3_uninit();
    printf("*** ( EV3 ) Mode Selector Ended ***\n");
    return 0;
}
