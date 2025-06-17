
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "sensor_methods.h"

#define TILE_LENGTH 253
#define RETURN_LENGTH 70
#define SPEED 30
#define N 4
#define R 4
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

int map[N][R];
int x_pos = 0;
int y_pos = 0;
int direction = NORTH;

uint8_t sn_gyro;
uint8_t sn_color[2];

void print_tile(int y, int x) {
    if (map[y][x] == 2) printf("■ ");
    else if (map[y][x] == 1) printf("□ ");
    else printf("⍰ ");
}

void print_map() {
    for (int y = N - 1; y >= 0; y--) {
        for (int x = 0; x < R; x++) {
            print_tile(y, x);
        }
        printf("\n");
    }
}

void update_position() {
    switch (direction) {
        case NORTH: y_pos++; break;
        case SOUTH: y_pos--; break;
        case EAST:  x_pos++; break;
        case WEST:  x_pos--; break;
    }
}

void move_forward_tile() {
    move_for_degrees(SPEED, (360 * TILE_LENGTH) / (3.14 * 49.5)); // Convert mm to degrees
}

void move_back_return() {
    move_for_degrees(SPEED, -(360 * RETURN_LENGTH) / (3.14 * 49.5));
}

void turn_left() {
    tank_turn(SPEED, 90);
    direction = (direction + 90) % 360;
}

void turn_right() {
    tank_turn(SPEED, -90);
    direction = (direction + 270) % 360;
}

void turn_around() {
    tank_turn(SPEED, 180);
    direction = (direction + 180) % 360;
}

bool is_valid(int y, int x) {
    return y >= 0 && y < N && x >= 0 && x < R;
}

bool explore_tile() {
    int color_value;
    get_color_value(sn_color[0], &color_value);
    if (color_value == 1 || color_value == 5) { // BLACK or RED
        map[y_pos][x_pos] = 2;
        move_back_return();
        return false;
    }
    map[y_pos][x_pos] = 1;
    return true;
}

bool is_goal() {
    return (x_pos == 3 && y_pos == 3);
}

void robot_loop() {
    while (!is_goal()) {
        explore_tile();

        // Example direction logic - always turn right if possible
        // In full implementation, you would check map[][] values
        turn_right();
        update_position();
        move_forward_tile();
    }
}

int main() {
    ev3_init();
    init_motors();
    init_gyro(&sn_gyro, true);
    init_all_color_sensors(sn_color, 2);

    for (int i = 0; i < N; i++)
        for (int j = 0; j < R; j++)
            map[i][j] = 0;

    robot_loop();
    print_map();

    ev3_uninit();
    return 0;
}
