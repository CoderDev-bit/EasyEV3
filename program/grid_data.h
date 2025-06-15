#ifndef GRID_DATA_H
#define GRID_DATA_H

#include <stdbool.h>

#define N 10
#define M 10
#define BLACK 1
#define WHITE 0

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

typedef struct {
    int color;
    bool visited;
    bool canMove[4];
} Tile;

extern Tile grid[N][M];
extern int x, y;
extern int direction;
extern int dx[4], dy[4];

#endif
