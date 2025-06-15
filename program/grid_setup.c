#include "grid_data.h"
#include <string.h>

void initialize_grid(Tile grid[N][M]) {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            grid[i][j].color = WHITE;
            grid[i][j].visited = false;
            memset(grid[i][j].canMove, true, sizeof(bool) * 4);
        }
    }
    // Sample black tiles
    grid[1][0].color = BLACK;
    grid[1][1].color = BLACK;
    grid[1][2].color = BLACK;
}
