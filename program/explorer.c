#include "grid_data.h"
#include "sensor_utils.h"

extern int x, y;
extern int direction;
extern Tile grid[N][M];
extern int dx[4], dy[4];

void move_to(int nx, int ny) {
    x = nx;
    y = ny;
    grid[y][x].visited = true;
}

void turn_left() {
    direction = (direction + 3) % 4;
}

void turn_right() {
    direction = (direction + 1) % 4;
}

void turn_around() {
    direction = (direction + 2) % 4;
}

void handle_dead_end() {
    int back_x = x - dx[direction];
    int back_y = y - dy[direction];
    if (!can_move(back_x, back_y)) return;
    move_to(back_x, back_y);

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
            move_to(back_x, back_y);
            continue;
        }
        turn_right();
    }

    handle_dead_end();
}
