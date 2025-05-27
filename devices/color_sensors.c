/*
     ____ __     ____   ___    ____ __         ((((()))
    | |_  \ \  /   ) ) | |  ) | |_  \ \  /  \(@)- /
    ||__  \\/  __)) ||/  ||__  \_\/   /(@)- \
                                               ((())))
*/
#include <string.h>
#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

#ifdef __WIN32__
#include <windows.h>
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif

const char *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

static bool _check_pressed( uint8_t sn ) {
    int val;
    if ( sn == SENSOR__NONE_ ) {
        return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
    }
    return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}

int main( void ) {
    char s[256];
    int val1, val2;
    uint8_t sn_touch = SENSOR__NONE_;
    uint8_t sn_color1 = SENSOR__NONE_;
    uint8_t sn_color2 = SENSOR__NONE_;

    printf( "Waiting for the EV3 brick to be online...\n" );
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );

    ev3_sensor_init();

    if ( ev3_search_sensor( LEGO_EV3_TOUCH, &sn_touch, 0 )) {
        printf( "TOUCH sensor is found.\n" );
    } else {
        printf( "TOUCH sensor is NOT found. Use UP on the EV3 brick to exit.\n" );
    }

    // Find two color sensors
    int found = 0;
    for ( int i = 0; i < DESC_LIMIT && found < 2; i++ ) {
        if ( ev3_sensor[i].type_inx == LEGO_EV3_COLOR ) {
            if (found == 0) {
                sn_color1 = i;
                found++;
            } else if (found == 1) {
                sn_color2 = i;
                found++;
            }
        }
    }

    if ( sn_color1 != SENSOR__NONE_ && sn_color2 != SENSOR__NONE_ ) {
        printf( "Both COLOR sensors found. Reading COLORS...\n" );
        set_sensor_mode( sn_color1, "COL-COLOR" );
        set_sensor_mode( sn_color2, "COL-COLOR" );

        while ( !_check_pressed( sn_touch )) {
            if ( !get_sensor_value( 0, sn_color1, &val1 ) || val1 < 0 || val1 >= COLOR_COUNT ) val1 = 0;
            if ( !get_sensor_value( 0, sn_color2, &val2 ) || val2 < 0 || val2 >= COLOR_COUNT ) val2 = 0;

            printf( "\rSensor 1: (%s) | Sensor 2: (%s)        ", color[val1], color[val2] );
            fflush( stdout );
            Sleep( 200 );
        }
    } else {
        printf( "ERROR: Two COLOR sensors not found.\n" );
    }

    ev3_uninit();
    printf( "\n*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}
