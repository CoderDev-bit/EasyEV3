// grid_navigation.c
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "sensor_methods.h"

// ======= CONSTANTS AND GLOBAL VARIABLES =======
#define START_X 0
#define START_Y 0
#define END_X 3
#define END_Y 3
#define N 4
#define R 4

// Color Constants for Traversable and Non-Traversable Tiles
// Define Color Constants for easier swapping
#define TRAVERSABLE_COLOR_1 6   // Default traversable color 1 (e.g., White)
#define TRAVERSABLE_COLOR_2 7   // Default traversable color 2 (e.g., Brown)
#define NON_TRAVERSABLE_COLOR_1 1  // Default non-traversable color 1 (e.g., Black)
#define NON_TRAVERSABLE_COLOR_2 5    // Default non-traversable color 2 (e.g., Red)



#define SPEED 200              // mm per second
#define TILE_LENGTH 253       // mm
#define RETURN_LENGTH 70      // mm

// Directions: 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

// Map legend: 0 = unvisited, 1 = white (visited), 2 = black/red (obstacle)
int map[N][R];

// Robot's current position and direction (0=N, 1=E, 2=S, 3=W)
int x_pos = START_X;
int y_pos = START_Y;
int current_dir = NORTH;

// Color sensor(s)
#define MAX_SENSORS 4
uint8_t color_sensors[MAX_SENSORS];
int color_sensor_count = 0;

// ====== HELPER FUNCTIONS ======

// Sleep helper
#define Sleep(ms) usleep((ms) * 1000)

// Convert direction index to string for debug
const char* dir_to_str(int d) {
    switch (d % 4) {
        case 0: return "NORTH";
        case 1: return "EAST";
        case 2: return "SOUTH";
        case 3: return "WEST";
    }
    return "?";
}

// Delta X/Y based on current direction
int dx[4] = {0, 1, 0, -1}; // N,E,S,W
int dy[4] = {1, 0, -1, 0}; // N,E,S,W

// Checks if (x, y) is in bounds
bool in_bounds(int x, int y) {
    return (x >= 0 && x < R && y >= 0 && y < N);
}

// Print the current map grid
void print_map() {
    printf("\nMaze Map (Y-down):\n");
    for (int y = N-1; y >= 0; y--) {
        for (int x = 0; x < R; x++) {
            if (x_pos == x && y_pos == y)
                printf("R ");
            else if (map[y][x] == 2)
                printf("N ");
            else if (map[y][x] == 1)
                printf("T ");
            else
                printf("⋅ ");
        }
        printf("\n");
    }
    printf("\n");
}


// Get tile color using the first color sensor (0=none, 1=black, 5=red, 6=white, 7=brown)
int get_current_tile_color() {
    int color = 0;
    if (color_sensor_count > 0) {
        get_color_value(color_sensors[0], &color);
    }
    return color;
}

// Turn robot to left (CCW 90°) or right (CW 90°)
void turn_left_90() {
    tank_turn(70, 90); // 90 degrees CCW
    current_dir = (current_dir + 3) % 4;
}
void turn_right_90() {
    tank_turn(70, -90); // 90 degrees CW
    current_dir = (current_dir + 1) % 4;
}
void turn_around_180() {
    tank_turn(70, 180); // 180 degrees
    current_dir = (current_dir + 2) % 4;
}
// Move robot forward one tile and update position
// Move robot forward one tile and update position
// Move robot forward one tile and update position
void move_forward_one_tile() {
    move_for_time(SPEED, (TILE_LENGTH * 1000) / SPEED);
    x_pos += dx[current_dir];
    y_pos += dy[current_dir];

    // Get the current tile's color
    int color = get_current_tile_color();

    // Mark tile as visited (traversable) unless it's an obstacle (black/red/white)
    if (color != NON_TRAVERSABLE_COLOR_1 && color != NON_TRAVERSABLE_COLOR_2) {
        map[y_pos][x_pos] = 1; // Mark the tile as traversable (visited)
    }
}




// Move robot backward return length (when hitting obstacle, don't update position)
void move_backward_return() {
    move_for_time(-SPEED, (RETURN_LENGTH * 1000) / SPEED);
}

// Set up all sensors and motors, initialize map to zero
// Set up all sensors and motors, initialize map to zero
bool initialize_robot() {
    printf("Initializing...\n");
    ev3_sensor_init();
    ev3_tacho_init();
    if (!init_motors()) {
        printf("Failed to initialize motors.\n");
        return false;
    }
    color_sensor_count = init_all_color_sensors(color_sensors, MAX_SENSORS);
    if (color_sensor_count < 1) {
        printf("No color sensor found.\n");
        return false;
    }
    for (int y = 0; y < N; y++)
        for (int x = 0; x < R; x++)
            map[y][x] = 0;
    srand(time(NULL));
    printf("Init done. Starting at (%d,%d) facing %s\n", x_pos, y_pos, dir_to_str(current_dir));
    map[START_Y][START_X] = 1;  // Mark start position as traversable
    return true;
}


// Returns if a given tile is open (unvisited or white)
bool is_tile_open(int x, int y) {
    return in_bounds(x, y) && (map[y][x] == 0 || map[y][x] == 1);
}

// Pick next direction: random left/right if both are open, otherwise deterministic
// Returns -1 if must backtrack, else 0 for left, 1 for right
int pick_next_direction() {
    int left_dir = (current_dir + 3) % 4;
    int right_dir = (current_dir + 1) % 4;
    int lx = x_pos + dx[left_dir], ly = y_pos + dy[left_dir];
    int rx = x_pos + dx[right_dir], ry = y_pos + dy[right_dir];

    bool left_in_bounds = in_bounds(lx, ly);
    bool right_in_bounds = in_bounds(rx, ry);

    bool left_open = left_in_bounds && is_tile_open(lx, ly);
    bool right_open = right_in_bounds && is_tile_open(rx, ry);

    if (!left_in_bounds)
        printf("DEBUG: Left move blocked by edge at (%d,%d)\n", lx, ly);
    if (!right_in_bounds)
        printf("DEBUG: Right move blocked by edge at (%d,%d)\n", rx, ry);

    if (left_open && right_open) {
        printf("DEBUG: Both left (%d,%d) and right (%d,%d) are open. Choosing randomly.\n", lx, ly, rx, ry);
        return rand() % 2; // random: 0=left, 1=right
    } else if (left_open) {
        printf("DEBUG: Only left (%d,%d) is open.\n", lx, ly);
        return 0; // Turn left
    } else if (right_open) {
        printf("DEBUG: Only right (%d,%d) is open.\n", rx, ry);
        return 1; // Turn right
    } else {
        printf("DEBUG: No open left/right tiles. Must backtrack.\n");
        return -1; // Need to backtrack
    }
}


void navigation_loop() {
    bool first_move = true;

    while (!(x_pos == END_X && y_pos == END_Y)) {
        print_map();

        // Step 1: Color detection and logic...

        // When an obstacle is detected, the robot turns 180 and checks left/right
        int color = get_current_tile_color();
        if (color == NON_TRAVERSABLE_COLOR_1 || color == NON_TRAVERSABLE_COLOR_2) {  // Black or Red = obstacle
            printf("Obstacle detected at (%d,%d).\n", x_pos, y_pos);
            map[y_pos][x_pos] = 2; // Mark as non-traversable
            move_backward_return();
            turn_around_180();
            // After 180° turn, choose to go left, right, or backtrack
            int next_turn = pick_next_direction();

            if (next_turn == -1) {
                printf("No open left/right. Backtracking...\n");
                move_forward_one_tile(); // Backtrack
            } else if (next_turn == 0) {
                printf("Turning +90° (CCW).\n");
                turn_left_90();
            } else if (next_turn == 1) {
                printf("Turning -90° (CW).\n");
                turn_right_90();
            }
            move_forward_one_tile(); // After turning, move forward one tile
            continue;
        }

        // Step 2: Mark tile as visited (white or brown)
        if (color == TRAVERSABLE_COLOR_1 || color == TRAVERSABLE_COLOR_2) {
            map[y_pos][x_pos] = 1; // Mark tile as visited
        }

        // ----- FIXED first_move block -----
        if (first_move) {
            int fx = x_pos + dx[current_dir], fy = y_pos + dy[current_dir];
            if (in_bounds(fx, fy) && is_tile_open(fx, fy)) {
                printf("Moving forward to (%d,%d)...\n", fx, fy);
                move_forward_one_tile();
            } else {
                printf("At map edge on first move, not moving forward.\n");
            }
            first_move = false;
            continue;
        }
        // ----- END FIX -----

        // ---- NEW FORWARD-CHECK LOGIC ----
        int fx = x_pos + dx[current_dir], fy = y_pos + dy[current_dir];
        if (in_bounds(fx, fy) && is_tile_open(fx, fy)) {
            printf("Moving forward to (%d,%d)...\n", fx, fy);
            move_forward_one_tile();
            continue;
        } else {
            printf("DEBUG: Forward move blocked by edge at (%d,%d)\n", fx, fy);
        }

        // Otherwise, check left/right for alternative routes
        int next_turn = pick_next_direction();
        if (next_turn == -1) {
            printf("No open left/right. Backtracking...\n");
            turn_around_180();
            move_forward_one_tile();
            continue;
        } else if (next_turn == 0) {
            printf("Turning +90° (CCW).\n");
            turn_left_90();
        } else if (next_turn == 1) {
            printf("Turning -90° (CW).\n");
            turn_right_90();
        }

        // After turning, move forward one tile
        int nx = x_pos + dx[current_dir], ny = y_pos + dy[current_dir];
        if (in_bounds(nx, ny) && is_tile_open(nx, ny)) {
            printf("Moving forward to (%d,%d)...\n", nx, ny);
            move_forward_one_tile();
        } else {
            printf("DEBUG: Forward move blocked by edge at (%d,%d)\n", nx, ny);
        }

        // Safety: Check bounds
        if (!in_bounds(x_pos, y_pos)) {
            printf("Moved out of bounds! Ending navigation.\n");
            break;
        }
    }
    printf("Reached end position (%d,%d).\n", x_pos, y_pos);
}



// After navigation, print map with legend
void print_final_grid() {
    printf("\nFinal Map:\n");
    for (int y = N-1; y >= 0; y--) {
        for (int x = 0; x < R; x++) {
            // Print robot's position
            if (x == x_pos && y == y_pos) {
                printf("R "); // Robot position
            }
            else if (map[y][x] == 2) {
                printf("N "); // Non-traversable (obstacle)
            }
            else if (map[y][x] == 1) {
                printf("T "); // Traversable (visited)
            }
            else {
                printf("⋅ "); // Unvisited
            }
        }
        printf("\n");
    }
}

// Print the value of a tile at given (x, y) coordinates
void print_tile_value(int x, int y) {
    // Check if the coordinates are in bounds
    if (in_bounds(x, y)) {
        printf("Tile at (%d, %d) has value: %d\n", x, y, map[y][x]);
    } else {
        printf("Invalid coordinates (%d, %d)\n", x, y);
    }
}



// ========== MAIN ===========
int main() {
    printf("==== EV3 Grid Navigation ====\n");

    if (ev3_init() < 1) {
        printf("Error: ev3_init failed.\n");
        return 1;
    }
    if (!initialize_robot()) {
        printf("Robot setup failed. Exiting.\n");
        return 1;
    }

    navigation_loop();

    print_final_grid();

    ev3_uninit();
    printf("Program complete.\n");
    print_tile_value(3, 3);  
    return 0;
}
