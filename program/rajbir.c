#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>    // For M_PI, cos, sin, fabs
#include <unistd.h>  // For sleep_ms (usleep)

// EasyEV3 includes
#include <ev3.h>
#include <ev3_port.h>
#include <ev3_tacho.h>
#include <ev3_sensor.h>

// --- Configuration Constants ---
#define LEFT_MOTOR_PORT      EV3_PORT__B // Adjust if your left motor is on a different port
#define RIGHT_MOTOR_PORT     EV3_PORT__C // Adjust if your right motor is on a different port
#define ULTRASONIC_SENSOR_PORT EV3_PORT__1 // Adjust if your ultrasonic sensor is on a different port
#define GYRO_SENSOR_PORT     EV3_PORT__2 // Adjust if your gyro sensor is on a different port

#define ROTATION_SPEED_PERCENT 10 // Motor power for slow rotation (0-100)
#define SCAN_DURATION_MS       10000 // Total time for 360-degree scan (adjust based on ROTATION_SPEED)
#define SAMPLE_INTERVAL_MS     50   // How often to read sensors during scan
#define ULTRASONIC_DETECTION_THRESHOLD_CM 100 // Max distance to consider a pylon detected
#define ANGULAR_GROUPING_THRESHOLD_DEG 10 // Max angle difference to group raw detections into one pylon
#define MIN_PYLON_DISTANCE_CM 5 // Minimum distance to consider a valid pylon (e.g., ignore self-detection)

#define MAX_RAW_DETECTIONS     (SCAN_DURATION_MS / SAMPLE_INTERVAL_MS + 10) // Max raw readings to store
#define MAX_PYLONS             20 // Max distinct pylons to store

// --- Data Structures ---

// Enum for pylon colors (can be expanded later)
typedef enum {
    COLOR_UNKNOWN,
    COLOR_RED,
    COLOR_BLUE,
    COLOR_GREEN,
    // Add more colors as needed
} PylonColor;

// Struct to store data for a single detected pylon
typedef struct {
    int id;           // Unique identifier for the pylon
    float x;          // X-coordinate in the grid system (cm)
    float y;          // Y-coordinate in the grid system (cm)
    PylonColor color; // Detected color (initialized to UNKNOWN)
    bool visited;     // True if the robot has visited this pylon
    bool pushed;      // True if the pylon has been pushed
} Pylon;

// Struct to store raw sensor readings during the scan
typedef struct {
    float angle_deg;  // Gyro angle at time of reading
    float distance_cm; // Ultrasonic distance at time of reading
} RawDetection;

// --- Global Variables for Sensor/Motor Handles ---
static uint8_t left_motor_id = 0;
static uint8_t right_motor_id = 0;
static uint8_t ultrasonic_sensor_id = 0;
static uint8_t gyro_sensor_id = 0;

// --- Function Prototypes ---
void initialize_robot_components();
void stop_motors();
void perform_360_scan(RawDetection raw_detections[], int *num_raw_detections);
void process_raw_detections(RawDetection raw_detections[], int num_raw_detections, Pylon pylons[], int *num_pylons);
void convert_polar_to_cartesian(float angle_deg, float distance_cm, float *x, float *y);

// --- Main Program ---
int main() {
    printf("Initializing EV3 robot...\n");
    if (!ev3_init()) {
        printf("EV3 initialization failed!\n");
        return 1;
    }

    initialize_robot_components();

    // Arrays to store data
    RawDetection raw_detections[MAX_RAW_DETECTIONS];
    int num_raw_detections = 0;

    Pylon detected_pylons[MAX_PYLONS];
    int num_detected_pylons = 0;

    printf("Starting 360-degree scan...\n");
    perform_360_scan(raw_detections, &num_raw_detections);
    printf("Scan complete. Stopping motors.\n");
    stop_motors();

    printf("Processing raw detections (%d readings)...\n", num_raw_detections);
    process_raw_detections(raw_detections, num_raw_detections, detected_pylons, &num_detected_pylons);

    printf("\n--- Detected Pylons (%d total) ---\n", num_detected_pylons);
    for (int i = 0; i < num_detected_pylons; i++) {
        printf("Pylon %d: (X=%.2f cm, Y=%.2f cm)\n",
               detected_pylons[i].id, detected_pylons[i].x, detected_pylons[i].y);
    }

    printf("\nShutting down EV3 system.\n");
    ev3_uninit();
    return 0;
}

// --- Function Implementations ---

/**
 * @brief Initializes motors and sensors, and resets the gyro.
 */
void initialize_robot_components() {
    // Find and initialize left motor
    if (!ev3_search_tacho_plugged_in(LEFT_MOTOR_PORT, 0, &left_motor_id)) {
        printf("ERROR: Left motor not found on port %s\n", ev3_port_name(LEFT_MOTOR_PORT));
        exit(EXIT_FAILURE);
    }
    printf("Left motor found on port %s (ID: %d)\n", ev3_port_name(LEFT_MOTOR_PORT), left_motor_id);
    set_tacho_stop_action_inx(left_motor_id, TACHO_BRAKE); // Brake when stopped

    // Find and initialize right motor
    if (!ev3_search_tacho_plugged_in(RIGHT_MOTOR_PORT, 0, &right_motor_id)) {
        printf("ERROR: Right motor not found on port %s\n", ev3_port_name(RIGHT_MOTOR_PORT));
        exit(EXIT_FAILURE);
    }
    printf("Right motor found on port %s (ID: %d)\n", ev3_port_name(RIGHT_MOTOR_PORT), right_motor_id);
    set_tacho_stop_action_inx(right_motor_id, TACHO_BRAKE); // Brake when stopped

    // Find and initialize ultrasonic sensor
    if (!ev3_search_sensor_plugged_in(ULTRASONIC_SENSOR_PORT, EV3_ULTRASONIC, &ultrasonic_sensor_id)) {
        printf("ERROR: Ultrasonic sensor not found on port %s\n", ev3_port_name(ULTRASONIC_SENSOR_PORT));
        exit(EXIT_FAILURE);
    }
    printf("Ultrasonic sensor found on port %s (ID: %d)\n", ev3_port_name(ULTRASONIC_SENSOR_PORT), ultrasonic_sensor_id);
    set_sensor_mode(ultrasonic_sensor_id, "US-DIST-CM"); // Set mode to distance in CM

    // Find and initialize gyro sensor
    if (!ev3_search_sensor_plugged_in(GYRO_SENSOR_PORT, EV3_GYRO, &gyro_sensor_id)) {
        printf("ERROR: Gyro sensor not found on port %s\n", ev3_port_name(GYRO_SENSOR_PORT));
        exit(EXIT_FAILURE);
    }
    printf("Gyro sensor found on port %s (ID: %d)\n", ev3_port_name(GYRO_SENSOR_PORT), gyro_sensor_id);
    set_sensor_mode(gyro_sensor_id, "GYRO-ANG"); // Set mode to angle

    // Reset gyro sensor to 0
    printf("Resetting gyro sensor...\n");
    set_sensor_mode(gyro_sensor_id, "GYRO-RESET"); // Reset mode
    sleep_ms(100); // Give it a moment to reset
    set_sensor_mode(gyro_sensor_id, "GYRO-ANG"); // Set back to angle mode
    sleep_ms(100); // Give it a moment to stabilize
    int current_gyro_angle;
    get_sensor_value0(gyro_sensor_id, &current_gyro_angle);
    printf("Gyro angle after reset: %d\n", current_gyro_angle);
}

/**
 * @brief Stops both left and right motors.
 */
void stop_motors() {
    set_tacho_command_inx(left_motor_id, TACHO_STOP);
    set_tacho_command_inx(right_motor_id, TACHO_STOP);
}

/**
 * @brief Performs a 360-degree tank turn, collecting raw sensor data.
 * @param raw_detections Array to store raw (angle, distance) readings.
 * @param num_raw_detections Pointer to an integer tracking the number of stored readings.
 */
void perform_360_scan(RawDetection raw_detections[], int *num_raw_detections) {
    // Start tank turn: one motor forward, one backward
    set_tacho_speed_sp(left_motor_id, ROTATION_SPEED_PERCENT * 10);  // Speed in milli-percent
    set_tacho_speed_sp(right_motor_id, -ROTATION_SPEED_PERCENT * 10); // Opposite direction

    set_tacho_command_inx(left_motor_id, TACHO_RUN_FOREVER);
    set_tacho_command_inx(right_motor_id, TACHO_RUN_FOREVER);

    int start_time_ms = ev3_time_ms();
    int current_time_ms = start_time_ms;
    int elapsed_time_ms = 0;

    int gyro_angle = 0;
    int ultrasonic_distance = 0;

    while (elapsed_time_ms < SCAN_DURATION_MS && *num_raw_detections < MAX_RAW_DETECTIONS) {
        // Read sensor values
        get_sensor_value0(gyro_sensor_id, &gyro_angle);
        get_sensor_value0(ultrasonic_sensor_id, &ultrasonic_distance);

        // Convert ultrasonic distance to float cm
        float dist_cm = (float)ultrasonic_distance / 10.0f; // Ultrasonic sensor usually returns mm, convert to cm

        // If a potential pylon is detected, store the raw reading
        if (dist_cm > MIN_PYLON_DISTANCE_CM && dist_cm < ULTRASONIC_DETECTION_THRESHOLD_CM) {
            raw_detections[*num_raw_detections].angle_deg = (float)gyro_angle;
            raw_detections[*num_raw_detections].distance_cm = dist_cm;
            (*num_raw_detections)++;
        }

        // Print status (optional, for debugging)
        // printf("Angle: %d deg, Distance: %.1f cm\n", gyro_angle, dist_cm);

        sleep_ms(SAMPLE_INTERVAL_MS); // Wait for the next sample
        current_time_ms = ev3_time_ms();
        elapsed_time_ms = current_time_ms - start_time_ms;
    }
    stop_motors(); // Ensure motors are stopped after scan
}

/**
 * @brief Compares two RawDetection structs by angle for sorting.
 */
int compare_raw_detections(const void *a, const void *b) {
    RawDetection *det_a = (RawDetection *)a;
    RawDetection *det_b = (RawDetection *)b;
    if (det_a->angle_deg < det_b->angle_deg) return -1;
    if (det_a->angle_deg > det_b->angle_deg) return 1;
    return 0;
}

/**
 * @brief Processes raw sensor detections to identify distinct pylons and calculate their coordinates.
 * @param raw_detections Array of raw (angle, distance) readings.
 * @param num_raw_detections Number of raw readings.
 * @param pylons Array to store identified Pylon structs.
 * @param num_pylons Pointer to an integer tracking the number of identified pylons.
 */
void process_raw_detections(RawDetection raw_detections[], int num_raw_detections, Pylon pylons[], int *num_pylons) {
    if (num_raw_detections == 0) {
        printf("No raw detections to process.\n");
        return;
    }

    // Sort raw detections by angle to easily group them
    qsort(raw_detections, num_raw_detections, sizeof(RawDetection), compare_raw_detections);

    *num_pylons = 0; // Reset pylon count

    // Iterate through sorted raw detections to group them into distinct pylons
    int i = 0;
    while (i < num_raw_detections && *num_pylons < MAX_PYLONS) {
        float current_group_sum_angle = 0;
        float current_group_min_distance = raw_detections[i].distance_cm;
        int group_count = 0;

        // Start a new group
        int j = i;
        while (j < num_raw_detections) {
            // Check if this detection belongs to the current group
            // Angular difference (handling wrap-around for 0/360 degrees)
            float angle_diff = fabs(raw_detections[j].angle_deg - raw_detections[i].angle_deg);
            if (angle_diff > 180.0f) { // Handle wrap-around for angles like 350 vs 10
                angle_diff = 360.0f - angle_diff;
            }

            if (angle_diff <= ANGULAR_GROUPING_THRESHOLD_DEG) {
                current_group_sum_angle += raw_detections[j].angle_deg;
                if (raw_detections[j].distance_cm < current_group_min_distance) {
                    current_group_min_distance = raw_detections[j].distance_cm;
                }
                group_count++;
                j++;
            } else {
                break; // This detection is too far in angle, start a new group
            }
        }

        // If the group is valid (has enough readings, min distance is reasonable)
        if (group_count > 0) {
            float average_angle = current_group_sum_angle / group_count;

            // Create a new Pylon struct
            pylons[*num_pylons].id = (*num_pylons) + 1;
            pylons[*num_pylons].color = COLOR_UNKNOWN;
            pylons[*num_pylons].visited = false;
            pylons[*num_pylons].pushed = false;

            // Convert polar to Cartesian coordinates
            convert_polar_to_cartesian(average_angle, current_group_min_distance,
                                       &pylons[*num_pylons].x, &pylons[*num_pylons].y);

            (*num_pylons)++;
        }
        i = j; // Move to the start of the next potential group
    }
}

/**
 * @brief Converts polar coordinates (angle, distance) to Cartesian (x, y) coordinates.
 * Assumes robot starts at (0,0) facing positive Y-axis (0 degrees gyro).
 * Gyro angle increases clockwise.
 *
 * @param angle_deg Angle in degrees from the gyro (0 is positive Y, increases clockwise).
 * @param distance_cm Distance in centimeters.
 * @param x Pointer to store the calculated X-coordinate.
 * @param y Pointer to store the calculated Y-coordinate.
 */
void convert_polar_to_cartesian(float angle_deg, float distance_cm, float *x, float *y) {
    // Adjust angle for standard mathematical conventions:
    // 0 degrees is positive X-axis, increases counter-clockwise.
    // If EV3 gyro is 0 deg when facing positive Y, and increases clockwise:
    // Math angle = (90 - gyro_angle) degrees
    // Handle negative angles from gyro by normalizing to 0-360 range if needed
    float normalized_angle_deg = fmod(angle_deg, 360.0f);
    if (normalized_angle_deg < 0) {
        normalized_angle_deg += 360.0f;
    }

    float math_angle_deg = 90.0f - normalized_angle_deg;

    // Convert to radians
    float angle_rad = math_angle_deg * M_PI / 180.0f;

    *x = distance_cm * cosf(angle_rad);
    *y = distance_cm * sinf(angle_rad);
}

// Helper function for sleep in milliseconds
void sleep_ms(int ms) {
    usleep(ms * 1000);
}
