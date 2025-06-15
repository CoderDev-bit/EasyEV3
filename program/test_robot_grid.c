#include <stdio.h>
#include "grid_data.h"

// Declare external functions from other modules
extern void initialize_grid(Tile grid[N][M]);
extern void explore();

// Global variable definitions
int x = 0, y = 0;
int direction = NORTH;
Tile grid[N][M];
int dx[4] = {0, 1, 0, -1};
int dy[4] = {-1, 0, 1, 0};

void print_grid() {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            if (grid[i][j].color == BLACK) {
                printf(" # ");
            } else if (x == j && y == i) {
                printf(" R ");
            } else if (grid[i][j].visited) {
                printf(" . ");
            } else {
                printf(" _ ");
            }
        }
        printf("\n");
    }
    printf("\n");
}

int main() {
    initialize_grid(grid);
    printf("Initial Grid State:\n");
    print_grid();

    explore();

    printf("Final Grid State (Visited Path):\n");
    print_grid();
    return 0;
}
