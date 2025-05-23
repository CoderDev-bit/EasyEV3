#include <string.h>  // For string manipulation (e.g., sprintf, not strictly needed for this version but kept from original)
#include <stdio.h>   // For printf (console output)
#include "ev3.h"         // Core EV3 system functions
#include "ev3_port.h"    // Port definitions
#include "ev3_sensor.h"  // Sensor-specific functions

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 ) // Define Sleep for Unix-like systems
#endif

/**
 * @brief Checks if an exit condition is met.
 * If a touch sensor is found, it checks if the touch sensor is pressed.
 * If no touch sensor is found, it checks if the EV3's UP button is pressed.
 * @param sn The system index of the touch sensor (or SENSOR__NONE_ if not found).
 * @return True if the exit condition is met, false otherwise.
 */
static bool _check_pressed( uint8_t sn )
{
    int val;
    // If no touch sensor is found (sn == SENSOR__NONE_), check the EV3's UP button.
    if ( sn == SENSOR__NONE_ ) {
        return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
    }
    // If a touch sensor is found, check if its value is non-zero (i.e., pressed).
    return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}

int main( void )
{
    char s[ 256 ]; // Buffer for sensor mode strings
    int distance_val; // Variable to store the ultrasonic distance value
    uint8_t sn_touch; // System index for the touch sensor (for exit condition)
    uint8_t sn_ultrasonic; // System index for the ultrasonic sensor

    printf( "Waiting the EV3 brick online...\n" );
    // Initialize the EV3 system. Returns 1 on success, < 1 on failure.
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );

    // Initialize sensor subsystem.
    ev3_sensor_init();

    // --- Search for Touch Sensor (for exit condition) ---
    // Search for a LEGO EV3 Touch Sensor on any port (port 0).
    if ( ev3_search_sensor( LEGO_EV3_TOUCH, &sn_touch, 0)) {
        printf( "TOUCH sensor is found, press BUTTON for EXIT...\n" );
    } else {
        printf( "TOUCH sensor is NOT found, press UP on the EV3 brick for EXIT...\n" );
        sn_touch = SENSOR__NONE_; // Set to NONE if not found, so _check_pressed uses EV3_KEY_UP
    }

    // --- Search for Ultrasonic Sensor ---
    // Search for a LEGO EV3 Ultrasonic Sensor on any port (port 0).
    // If you want to specify a port, replace '0' with EV3_PORT_S1, EV3_PORT_S2, etc.
    if ( ev3_search_sensor( LEGO_EV3_US, &sn_ultrasonic, 0 )) {
        printf( "ULTRASONIC sensor is found, reading distance...\n" );
        // Set the ultrasonic sensor mode to read distance in centimeters.
        set_sensor_mode( sn_ultrasonic, LEGO_EV3_US_US_DIST_CM );

        // --- Main Loop: Read and Display Ultrasonic Distance ---
        for ( ; ; ) { // Infinite loop
            // Get the current distance value from the ultrasonic sensor.
            // get_sensor_value(0, sn, &val) reads the first value (index 0) from the sensor.
            if ( !get_sensor_value( 0, sn_ultrasonic, &distance_val )) {
                // If reading fails, print an error.
                printf( "\rERROR: Failed to read ultrasonic sensor value.    " );
            } else {
                // Print the distance to the console.
                // Using \r to return cursor to start of line for continuous update.
                // Added spaces to clear previous text if the number of digits changes.
                printf( "\rDistance: %d cm          ", distance_val );
            }
            fflush( stdout ); // Flush stdout to ensure immediate printing

            // Check if the exit button/sensor is pressed.
            if ( _check_pressed( sn_touch )) break; // Exit the loop if pressed

            Sleep( 200 ); // Wait for 200 milliseconds before the next reading
        }
    } else {
        printf( "ULTRASONIC sensor is NOT found\n" );
        // If no ultrasonic sensor, wait for exit condition
        while ( !_check_pressed( sn_touch )) Sleep( 100 );
    }

    // Uninitialize the EV3 system and release resources.
    ev3_uninit();
    printf( "\n*** ( EV3 ) Bye! ***\n" ); // Print exit message
    return ( 0 ); // Indicate successful execution
}