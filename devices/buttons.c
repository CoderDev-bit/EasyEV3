#include <stdio.h>   // For printf for console output
#include <string.h>  // Included for string operations, though not heavily used here
#include "ev3.h"     // Core EV3 system functions, including button definitions

// For Sleep/usleep compatibility across different operating systems
#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 ) // Define Sleep for Unix-like systems (EV3DEV)
#endif

/**
 * @brief Returns the name of the first identified button from a given key bitmask.
 * If multiple buttons are pressed, it returns the name of one of them.
 * @param keys A bitmask representing the state of pressed buttons.
 * @return A string literal representing the button's name, or NULL if no recognized button.
 */
const char* get_button_name(uint8_t keys) {
    if (keys & EV3_KEY_UP) return "UP";
    if (keys & EV3_KEY_DOWN) return "DOWN";
    if (keys & EV3_KEY_LEFT) return "LEFT";
    if (keys & EV3_KEY_RIGHT) return "RIGHT";
    if (keys & EV3_KEY_CENTER) return "CENTER";
    if (keys & EV3_KEY_BACK) return "BACK";
    return NULL; // No single recognized button in the bitmask
}

int main(void) {
    printf("Waiting the EV3 brick online...\n");
    // Initialize the EV3 system. Returns 1 on success, < 1 on failure.
    if (ev3_init() < 1) {
        printf("ERROR: Failed to initialize EV3 system.\n");
        return 1;
    }
    printf("*** ( EV3 ) Button Monitor Started! ***\n");
    printf("Press any button to see its name. Press the BACK button to exit.\n");

    uint8_t current_keys = 0;  // Stores the button state in the current iteration
    uint8_t previous_keys = 0; // Stores the button state from the previous iteration
    const char* button_name;   // To store the name of the pressed button

    // Main loop: Continues indefinitely until the BACK button is pressed
    while (1) {
        // Read the current state of all EV3 buttons
        // ev3_read_keys populates 'current_keys' with a bitmask where each bit
        // corresponds to a button's pressed state.
        ev3_read_keys(&current_keys);

        // Detect which buttons were just pressed (edge detection).
        // This calculates buttons that are currently down (current_keys)
        // AND were not down in the previous iteration (~previous_keys).
        uint8_t pressed_now = current_keys & ~previous_keys;

        // If 'pressed_now' is not zero, at least one button was just pressed.
        if (pressed_now != 0) {
            button_name = get_button_name(pressed_now); // Get the name of the pressed button

            if (button_name) {
                printf("Button Pressed: %s\n", button_name);
            } else {
                // This case handles situations where multiple buttons might be pressed
                // simultaneously in a way that doesn't match a single EV3_KEY_X constant.
                printf("Button Pressed: Unknown combination (Raw bitmask: 0x%02X)\n", pressed_now);
            }
        }

        // Check for the exit condition: If the BACK button is currently held down.
        if (current_keys & EV3_KEY_BACK) {
            printf("BACK button pressed. Exiting...\n");
            break; // Exit the while loop
        }

        // Update previous_keys for the next iteration.
        previous_keys = current_keys;

        // Introduce a small delay to prevent busy-waiting and to debounce button presses.
        // A 50ms delay is usually sufficient for button monitoring.
        Sleep(50);
    }

    // Uninitialize the EV3 system and release resources.
    ev3_uninit();
    printf("*** ( EV3 ) Button Monitor Ended! ***\n");
    return 0; // Indicate successful program execution
}