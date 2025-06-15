#include "grid_data.h"

extern void initialize_grid(Tile grid[N][M]);
extern void explore();

int x = 0, y = 0;
int direction = NORTH;
Tile grid[N][M];
int dx[4] = {0, 1, 0, -1};
int dy[4] = {-1, 0, 1, 0};

int main() {
    initialize_grid(grid);
    explore();
    return 0;
}
