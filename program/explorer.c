#include <stdio.h>
#include "grid_data.h"
#include "sensor_utils.h"
#include "sensor_methods.h"

extern int x, y;
extern int direction;
extern Tile grid[N][M];
extern int dx[4], dy[4];
extern uint8_t sn_color;

void bump_reflex();  // Forward declaration

void move_to(int nx, int ny) {
    int move_degrees = 360; // Adjust based on tile size
    move_for_degrees(200, move_degrees); // Move forward one tile

    int color_val = 0;
    if (get_color_value(sn_color, &color_val)) {
        printf("Color sensor reads: %d\n", color_val);
        if (color_val == 1) { // 1 = BLACK
            printf("Black tile detected at (%d, %d)! Reversing.\n", nx, ny);
            bump_reflex();
            return;
        }
    }

    x = nx;
    y = ny;
    grid[y][x].visited = true;
    printf("Moved to (%d, %d)\n", x, y);
}

void turn_left() {
    tank_turn(100, -90);  // Left turn
    direction = (direction + 3) % 4;
}

void turn_right() {
    tank_turn(100, 90);   // Right turn
    direction = (direction + 1) % 4;
}

void turn_around() {
    tank_turn(100, 180);
    direction = (direction + 2) % 4;
}

void bump_reflex() {
    move_for_degrees(-150, 180); // reverse
    arc_turn(150, 0.5f, 800);    // arc away
    stop_motors();
}

void handle_dead_end() {
    int back_x = x - dx[direction];
    int back_y = y - dy[direction];
    if (!can_move(back_x, back_y)) return;
    move_for_degrees(-200, 360); // reverse one tile
    x = back_x;
    y = back_y;

    for (int i = 0; i < 4; ++i) {
        turn_left();
        int nx = x + dx[direction];
        int ny = y + dy[direction];
        if (can_move(nx, ny) && !grid[ny][nx].visited) {
            return;
        }
    }
    handle_dead_end();
}

void explore() {
    grid[y][x].visited = true;

    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[direction];
        int ny = y + dy[direction];

        if (can_move(nx, ny) && !grid[ny][nx].visited) {
            move_to(nx, ny);
            explore();
            int back_x = x - dx[direction];
            int back_y = y - dy[direction];
            move_for_degrees(-200, 360); // backtrack
            x = back_x;
            y = back_y;
            continue;
        }
        turn_right();
    }

    handle_dead_end();
}
