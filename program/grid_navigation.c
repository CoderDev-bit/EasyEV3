#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "sensor_methods.h"

#define TILE_LENGTH 253
#define RETURN_LENGTH 70
#define SPEED 30
#define WHEEL_DIAMETER 49.5
#define WHEEL_BASE 104
#define N 4
#define R 4

int map[N][R]; // 0 = unvisited, 1 = white, 2 = obstacle
char CURRENT_DIR = 'N';
int x_pos = 0;
int y_pos = 0;

uint8_t sn_gyro;
uint8_t color_sensors[2];

void gyro_correction() {
    int angle;
    Sleep(500);
    get_gyro_angle(sn_gyro, &angle);
    while (abs(angle) > 3) {
        int correction = angle > 0 ? -5 : 5;
        tank_turn(10, correction);
        get_gyro_angle(sn_gyro, &angle);
    }
    reset_gyro(sn_gyro);
}

void update_direction(char turn) {
    switch (CURRENT_DIR) {
        case 'N': CURRENT_DIR = (turn == 'L') ? 'W' : 'E'; break;
        case 'S': CURRENT_DIR = (turn == 'L') ? 'E' : 'W'; break;
        case 'E': CURRENT_DIR = (turn == 'L') ? 'N' : 'S'; break;
        case 'W': CURRENT_DIR = (turn == 'L') ? 'S' : 'N'; break;
    }
}

void update_position() {
    if (CURRENT_DIR == 'N') y_pos++;
    else if (CURRENT_DIR == 'S') y_pos--;
    else if (CURRENT_DIR == 'E') x_pos++;
    else if (CURRENT_DIR == 'W') x_pos--;
}

void move_forward_tile() {
    move_for_degrees(SPEED, (int)(TILE_LENGTH / (WHEEL_DIAMETER * 3.14159) * 360));
}

void move_back_return() {
    move_for_degrees(-SPEED, (int)(RETURN_LENGTH / (WHEEL_DIAMETER * 3.14159) * 360));
}

bool detect_obstacle(int color) {
    return color == 1 || color == 5; // black or red
}

void mark_tile(int value) {
    if (x_pos >= 0 && x_pos < R && y_pos >= 0 && y_pos < N) {
        map[y_pos][x_pos] = value;
    }
}

bool is_valid(int x, int y) {
    return x >= 0 && x < R && y >= 0 && y < N && map[y][x] != 2;
}

void turn_robot(char turn) {
    tank_turn(SPEED, (turn == 'L') ? 90 : -90);
    update_direction(turn);
    gyro_correction();
}

void explore_grid() {
    while (x_pos != 3 || y_pos != 3) {
        int color;
        get_color_value(color_sensors[0], &color);

        if (detect_obstacle(color)) {
            mark_tile(2);
            move_back_return();
            continue;
        } else {
            mark_tile(1);
        }

        int next_x = x_pos;
        int next_y = y_pos;
        bool moved = false;

        if (CURRENT_DIR == 'N') next_y++;
        else if (CURRENT_DIR == 'S') next_y--;
        else if (CURRENT_DIR == 'E') next_x++;
        else if (CURRENT_DIR == 'W') next_x--;

        if (is_valid(next_x, next_y)) {
            update_position();
            move_forward_tile();
            continue;
        }

        turn_robot('L');
        if (CURRENT_DIR == 'N') next_y++;
        else if (CURRENT_DIR == 'S') next_y--;
        else if (CURRENT_DIR == 'E') next_x++;
        else if (CURRENT_DIR == 'W') next_x--;

        if (is_valid(next_x, next_y)) {
            update_position();
            move_forward_tile();
            continue;
        }

        turn_robot('R'); turn_robot('R'); // 180°
        if (CURRENT_DIR == 'N') next_y++;
        else if (CURRENT_DIR == 'S') next_y--;
        else if (CURRENT_DIR == 'E') next_x++;
        else if (CURRENT_DIR == 'W') next_x--;

        if (is_valid(next_x, next_y)) {
            update_position();
            move_forward_tile();
        }
    }
}

void print_map() {
    printf("Final Grid:\n");
    for (int y = N - 1; y >= 0; y--) {
        for (int x = 0; x < R; x++) {
            switch (map[y][x]) {
                case 0: printf("⍰ "); break;
                case 1: printf("□ "); break;
                case 2: printf("■ "); break;
            }
        }
        printf("\n");
    }
}

int main() {
    ev3_init();
    init_motors();
    init_gyro(&sn_gyro, true);
    init_all_color_sensors(color_sensors, 2);

    for (int i = 0; i < N; i++) for (int j = 0; j < R; j++) map[i][j] = 0;

    explore_grid();
    print_map();
    ev3_uninit();
    return 0;
}
