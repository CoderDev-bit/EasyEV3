#include "sensor_utils.h"
#include "grid_data.h"

bool detect_black_tile(int nx, int ny) {
    return grid[ny][nx].color == BLACK;
}

bool can_move(int nx, int ny) {
    return nx >= 0 && nx < M && ny >= 0 && ny < N && grid[ny][nx].color == WHITE;
}
