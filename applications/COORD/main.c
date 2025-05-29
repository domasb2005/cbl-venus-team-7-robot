#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libpynq.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <stepper.h>
#include <pthread.h>
#include <mosquitto.h>
#include <math.h>
#include <ctype.h>



// Position tracking globals
volatile double current_x = 0.0;
volatile double current_y = 0.0;
volatile double heading_deg = 0.0;  // in degrees

pthread_mutex_t position_mutex = PTHREAD_MUTEX_INITIALIZER;

// Simple linear scaling based on empirical measurement
#define STEPS_PER_90_DEGREES 625
#define M_PI 3.14159265358979323846
// For 90° turn: left=625, right=-625, difference = 625-(-625) = 1250
#define STEP_DIFFERENCE_PER_90_DEGREES (STEPS_PER_90_DEGREES * 2)  // = 1250
#define DEGREES_PER_STEP_DIFFERENCE (90.0 / STEP_DIFFERENCE_PER_90_DEGREES)  // = 0.072 degrees per step difference

// IR sensor pin definitions (ADC channels)
#define IR_2L ADC0  // Middle - left (a0)
#define IR_1L ADC3  // Closest to center - left (a1)
#define IR_3L ADC2  // Furthest from center - left (a2)
#define IR_3R ADC1  // Furthest from center - right (a3) -line follower
#define IR_1R ADC4  // Closest to center - right (a4) -line follower
#define IR_2R ADC5  // Middle - right (a5) - line follower

// Add these calibration constants at the top of your file with other defines
#define RIGHT_TURN_ADJUSTMENT 4   // Empirically determined offset for right turns
#define LEFT_TURN_ADJUSTMENT 2     // Empirically determined offset for left turns

// RIGHT 90° turn is 607 steps each wheel = 1214 step difference
#define RIGHT_STEP_DIFFERENCE_PER_90 ((STEPS_PER_90_DEGREES - RIGHT_TURN_ADJUSTMENT) * 2)  // = 1214
// LEFT 90° turn is 622 steps each wheel = 1244 step difference
#define LEFT_STEP_DIFFERENCE_PER_90 ((STEPS_PER_90_DEGREES - LEFT_TURN_ADJUSTMENT) * 2)  // = 1244

// Calculate degrees per step differently for left and right turns
#define RIGHT_DEGREES_PER_STEP_DIFF (90.0 / RIGHT_STEP_DIFFERENCE_PER_90)  // = 90/1214 = 0.0741
#define LEFT_DEGREES_PER_STEP_DIFF (90.0 / LEFT_STEP_DIFFERENCE_PER_90)    // = 90/1244 = 0.0723

// Updated heading calculation that accounts fora different left/right calibration
double compute_heading_change(int left_steps, int right_steps) {
    // Calculate step difference
    int step_difference = right_steps - left_steps;
    
    // Apply appropriate conversion factor based on turn direction
    if (step_difference > 0) {
        // Right turn - use the right turn calibration
        return step_difference * RIGHT_DEGREES_PER_STEP_DIFF;
    } else if (step_difference < 0) {
        // Left turn - use the left turn calibration
        return step_difference * LEFT_DEGREES_PER_STEP_DIFF;
    } else {
        // No turn (straight line)
        return 0.0;
    }
}

// This function calculates heading change with motor imperfection adjustments
double compute_heading_change_calibrated(int16_t left_steps, int16_t right_steps, char direction) {
    // Apply calibration based on turn direction
    if (direction == 'd' || direction == 'D') {  // Right turn
        // Adjust steps to match calibrated right turn values
        double adjustment_factor = (double)RIGHT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES;
        double adjusted_steps = abs(left_steps) * (1.0 - adjustment_factor);
        return compute_heading_change(-adjusted_steps, adjusted_steps);
    } 
    else if (direction == 'a' || direction == 'A') {  // Left turn
        // Adjust steps to match calibrated left turn values
        double adjustment_factor = (double)LEFT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES;
        double adjusted_steps = abs(left_steps) * (1.0 - adjustment_factor);
        return compute_heading_change(adjusted_steps, -adjusted_steps);
    }
    else {
        // Forward/backward - no calibration needed
        return compute_heading_change(left_steps, right_steps);
    }
}

void normalize_heading() {
    while (heading_deg >= 360.0) heading_deg -= 360.0;
    while (heading_deg < 0.0) heading_deg += 360.0;
}


void* uart_position_publisher(void* arg) {
    (void)arg;  // Explicitly mark arg as unused to silence warning
    // Setup UART
    switchbox_init();
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    uart_init(UART0);
    uart_reset_fifos(UART0);

    // Setup MQTT
    printf("Connecting to MQTT broker...\n");
    struct mosquitto *mosq = NULL;
    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Error: Out of memory.\n");
        return NULL;
    }

    if (mosquitto_connect(mosq, "192.168.101.6", 1883, 60)) {
        fprintf(stderr, "Unable to connect to MQTT broker.\n");
        return NULL;
    }
    printf("[✓] Successfully connected to MQTT broker at 192.168.101.4:1883\n");


    char msg[128];

    while (1) {
        pthread_mutex_lock(&position_mutex);
        double x = current_x;
        double y = current_y;
        double h = heading_deg;
        pthread_mutex_unlock(&position_mutex);

        snprintf(msg, sizeof(msg), "{\"x\":%.2f,\"y\":%.2f,\"heading\":%.2f}", x, y, h);

        // Send UART packet
        uint32_t length = strlen(msg);
        uint8_t* len_bytes = (uint8_t*)&length;
        for (uint32_t i = 0; i < 4; i++) {
            uart_send(UART0, len_bytes[i]);
        }
        for (uint32_t i = 0; i < length; i++) {
            uart_send(UART0, msg[i]);
        }

        // Publish to MQTT
        mosquitto_publish(mosq, NULL, "robot/position", strlen(msg), msg, 0, false);

        sleep(1);  // Send every 1 second
    }

    // Cleanup MQTT
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    // Cleanup UART
    uart_reset_fifos(UART0);
    uart_destroy(UART0);
    pthread_exit(NULL);
}


// Helper function to update position based on completed steps
void update_position_from_steps(int16_t left_steps, int16_t right_steps, char movement_type) {
    pthread_mutex_lock(&position_mutex);
    double rad = heading_deg * (M_PI / 180.0);
    
    if (movement_type == 'W' || movement_type == 'w' || movement_type == 'S' || movement_type == 's') {
        // Forward/backward - we use left_steps for distance as both wheels should move the same
        // The sign of left_steps already indicates direction, so we just add (+ for forward, - for backward)
        current_x += cos(rad) * left_steps;
        current_y += sin(rad) * left_steps;
    } else if (movement_type == 'A' || movement_type == 'a' || movement_type == 'D' || movement_type == 'd') {
        // Turning - use calibrated heading calculation
        double heading_change = compute_heading_change(left_steps, right_steps);
        heading_deg += heading_change;
        normalize_heading();
    }
    
    pthread_mutex_unlock(&position_mutex);
}

// Helper function to print current position and heading
void print_position_status(const char* prefix) {
    pthread_mutex_lock(&position_mutex);
    printf("%s (%.2f, %.2f), Heading: %.2f°\n", 
           prefix ? prefix : "Position:", current_x, current_y, heading_deg);
    pthread_mutex_unlock(&position_mutex);
}


void calibration_mode() {
    printf("Calibration Mode\n");
    printf("Commands:\n");
    printf("  t: Test 360° turn using small right steps\n");
    printf("  f: Full motor revolution (1600 steps both wheels)\n");
    printf("  s: Full revolution using small_step_forward\n");
    printf("  r: Interrupted forward movement test\n");
    printf("  p: Interrupted left turn test\n");
    printf("  9: Test 90° turns (r=right, l=left, q=exit)\n");
    printf("  q: Exit calibration mode\n");
    
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    stepper_enable();

    int running = 1;
    while (running) {
        printf("\nCalibration command: ");
        fflush(stdout);
        
        char c;
        if (scanf(" %c", &c)) {
            switch(c) {
                // Add this case to your calibration_mode() function
case 'r': {  // 'r' for interrupted Revolution
    printf("\n[⏹] Testing interrupted movement...\n");
    
    stepper_enable();
    stepper_set_speed(15000, 15000);
    
    printf("Starting 5000 steps forward (press 'q' to stop)...\n");
    stepper_steps(5000, 5000);
    
    // Wait for user to press 'q' to stop
    struct termios old_term, new_term;
    tcgetattr(STDIN_FILENO, &old_term);
    new_term = old_term;
    new_term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
    
    printf("Robot is moving... Press 'q' to stop: ");
    fflush(stdout);
    
    char input;
    while (1) {
        if (read(STDIN_FILENO, &input, 1) > 0) {
            if (input == 'q') {
                break;
            }
        }
        
        // Check if movement completed naturally
        if (stepper_steps_done()) {
            printf("\nMovement completed naturally!\n");
            break;
        }
        
        sleep_msec(10);  // Small delay to prevent CPU hogging
    }
    
    printf("\nStopping movement...\n");
    
    // Get completed steps BEFORE resetting
    int16_t completed_left, completed_right;
    stepper_get_completed_steps(&completed_left, &completed_right);
    
    stepper_reset();
    
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    
    printf("\n[✓] Interrupted movement results:\n");
    printf("Requested: Left=5000, Right=5000\n");
    printf("Completed: Left=%d, Right=%d\n", completed_left, completed_right);
    printf("Completion rate: %.1f%%\n", 
           (completed_left / 5000.0) * 100.0);
    
    // Update position based on actual completed steps
    pthread_mutex_lock(&position_mutex);
    double rad = heading_deg * (M_PI / 180.0);
    current_x += cos(rad) * completed_left;
    current_y += sin(rad) * completed_left;
    pthread_mutex_unlock(&position_mutex);
    
    break;
}
                
                case 'f': {
                    printf("\n[→] Performing full motor revolution (1600 steps)...\n");
                    
                    // Save initial position for comparison
                    pthread_mutex_lock(&position_mutex);
                    double start_x = current_x;
                    double start_y = current_y;

                    pthread_mutex_unlock(&position_mutex);
                    
                    // Execute one full motor revolution
                    stepper_reset();
                    stepper_enable();
                    stepper_set_speed(15000, 15000);
                    stepper_steps(1600, 1600);
                    
                    printf("Waiting for full revolution to complete...\n");
                    while (stepper_steps_done() != true) {
                        sleep_msec(1);
                    }
                    
                    // Update position based on full revolution
                    pthread_mutex_lock(&position_mutex);
                    double rad = heading_deg * (M_PI / 180.0);
                    current_x += cos(rad) * 1600;
                    current_y += sin(rad) * 1600;
                    pthread_mutex_unlock(&position_mutex);
                    
                    // Show results
                    pthread_mutex_lock(&position_mutex);
                    printf("\n[✓] Full revolution complete!\n");
                    printf("Distance traveled: X=%.2f, Y=%.2f\n", 
                           current_x - start_x, current_y - start_y);
                    printf("Heading remained at: %.2f°\n", heading_deg);
                    pthread_mutex_unlock(&position_mutex);
                    break;
                }

case 'p': {  // 'p' for interrupted left turn on spot
    printf("\n[⏹] Testing interrupted left turn...\n");
    
    stepper_enable();
    stepper_set_speed(30000, 30000);
    
    printf("Starting crazy left turn (10000 steps, press 'q' to stop)...\n");
    stepper_steps(10000, -10000);  // Left wheel forward, right wheel backward = left turn
    
    // Wait for user to press 'q' to stop
    struct termios old_term, new_term;
    tcgetattr(STDIN_FILENO, &old_term);
    new_term = old_term;
    new_term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
    
    printf("Robot is turning left... Press 'q' to stop: ");
    fflush(stdout);
    
    char input;
    while (1) {
        if (read(STDIN_FILENO, &input, 1) > 0) {
            if (input == 'q') {
                break;
            }
        }
        
        sleep_msec(10);  // Small delay to prevent CPU hogging
    }
    
    printf("\nStopping turn...\n");
    
    // Get completed steps BEFORE resetting
    int16_t completed_left, completed_right;
    stepper_get_completed_steps(&completed_left, &completed_right);
    
    stepper_reset();
    
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    
    printf("\n[✓] Interrupted left turn results:\n");
    printf("Requested: Left=10000, Right=-10000\n");
    printf("Completed: Left=%d, Right=%d\n", completed_left, completed_right);
    printf("Left completion rate: %.1f%%\n", (completed_left / 10000.0) * 100.0);
    printf("Right completion rate: %.1f%%\n", (completed_right / 10000.0) * 100.0);
    
    // Update heading based on actual completed steps
    pthread_mutex_lock(&position_mutex);
    heading_deg += compute_heading_change(completed_left, -completed_right);
    normalize_heading();
    printf("New heading: %.2f°\n", heading_deg);
    pthread_mutex_unlock(&position_mutex);
    
    break;
}

                case '9': {
                    // 90 degree turn test mode
                    printf("\n[⟳] 90° Turn Testing Mode\n");
                    printf("Commands:\n");
                    printf("  r: Right 90° turn (-%d+%d, +%d-%d steps)\n", 
                           STEPS_PER_90_DEGREES, RIGHT_TURN_ADJUSTMENT, 
                           STEPS_PER_90_DEGREES, RIGHT_TURN_ADJUSTMENT);
                    printf("  l: Left 90° turn (+%d-%d, -%d+%d steps)\n", 
                           STEPS_PER_90_DEGREES, LEFT_TURN_ADJUSTMENT, 
                           STEPS_PER_90_DEGREES, LEFT_TURN_ADJUSTMENT);
                    printf("  q: Exit to main calibration menu\n");
                    
                    // Reset heading to 0 for clean test
                    printf("Resetting heading to 0.0° for test. Continue? (y/n): ");
                    char confirm;
                    scanf(" %c", &confirm);
                    if (confirm == 'y' || confirm == 'Y') {
                        pthread_mutex_lock(&position_mutex);
                        heading_deg = 0.0;
                        pthread_mutex_unlock(&position_mutex);
                        printf("Heading reset to 0.0°\n");
                    } else {
                        printf("Heading reset canceled\n");
                    }
                    
                    while (1) {
                        printf("Enter command (r/l/q): ");
                        char turn_dir;
                        scanf(" %c", &turn_dir);
                        
                        if (turn_dir == 'q') {
                            printf("Exiting 90° Turn Testing Mode\n");
                            break;
                        }
                        
                        int16_t steps = STEPS_PER_90_DEGREES;
                        int16_t left_steps, right_steps;
                        
                        if (turn_dir == 'r') {
                            left_steps = -(steps - RIGHT_TURN_ADJUSTMENT);
                            right_steps = steps - RIGHT_TURN_ADJUSTMENT;
                            
                            printf("[→] Turning right 90° (%d, %d steps)\n", left_steps, right_steps);
                            stepper_reset();
                            stepper_enable();
                            stepper_set_speed(30000, 30000);
                            stepper_steps(left_steps, right_steps);
                        } else if (turn_dir == 'l') {
                            left_steps = steps - LEFT_TURN_ADJUSTMENT;
                            right_steps = -(steps - LEFT_TURN_ADJUSTMENT);
                            
                            printf("[←] Turning left 90° (%d, %d steps)\n", left_steps, right_steps);
                            stepper_reset();
                            stepper_enable();
                            stepper_set_speed(30000, 30000);
                            stepper_steps(left_steps, right_steps);
                        } else {
                            printf("Invalid command '%c'. Use r, l, or q.\n", turn_dir);
                            continue;
                        }
                        
                        // Wait for turn to complete
                        while (stepper_steps_done() != true) {
                            sleep_msec(1);
                        }
                        
                        // Update heading using actual step values that were used 
                        pthread_mutex_lock(&position_mutex);
                        heading_deg += compute_heading_change(left_steps, right_steps);
                        normalize_heading();
                        printf("Computed heading change: %.2f°\n", 
                               compute_heading_change(left_steps, right_steps));
                        printf("New heading: %.2f°\n", heading_deg);
                        pthread_mutex_unlock(&position_mutex);
                    }
                    
                    break;
                }

                case 'm': {
                    // Snake game pattern mode with hardcoded sequence
                    printf("\n[🐍] Snake Game Pattern Simulator\n");
                    printf("Controls (same as manual mode):\n");
                    printf("  W: Forward 20K steps\n");
                    printf("  S: Backward 20K steps\n");
                    printf("  A: Left turn 20K steps\n");
                    printf("  D: Right turn 20K steps\n");
                    
                    // Speed setting for all movements
                    const uint16_t SNAKE_SPEED = 30000;
                    // Sleep time between movements (ms)
                    const int SNAKE_DELAY_MS = 1500;
                    
                    // Reset position tracking
                    printf("Reset position and heading to 0? (y/n): ");
                    char confirm;
                    scanf(" %c", &confirm);
                    if (confirm == 'y' || confirm == 'Y') {
                        pthread_mutex_lock(&position_mutex);
                        current_x = 0.0;
                        current_y = 0.0;
                        heading_deg = 0.0;
                        pthread_mutex_unlock(&position_mutex);
                        printf("Position and heading reset to 0\n");
                    }
                    
                    // Define a hardcoded sequence instead of getting user input WLSRSRWLSRSLSRSRSRWRWRW
                    const char sequence[] = "WASDSDWASDSASDSDSDWDWDW";  // Example: Forward, Left, Forward, Right, Forward, Left, Forward, Left
                    const int seq_len = strlen(sequence);
                    
                    printf("\n[▶] Starting Snake Pattern with sequence: %s\n", sequence);
                    printf("Press Enter to begin: ");
                    getchar(); // Consume previous newline
                    getchar(); // Wait for Enter
                    
                    // Track completed steps for position/heading updates
                    int16_t completed_left = 0;
                    int16_t completed_right = 0;
                    
                    // Execute the sequence
                    for (int i = 0; i < seq_len; i++) {
                        char move = sequence[i];
                        printf("\n[%d/%d] ", i + 1, seq_len);
                        
                        stepper_reset();
                        stepper_enable();
                        stepper_set_speed(SNAKE_SPEED, SNAKE_SPEED);
                        
                        int16_t left_steps = 0, right_steps = 0;
                        
                        switch (move) {
                            case 'W':
                                printf("Moving FORWARD 20K steps...\n");
                                left_steps = 20000;
                                right_steps = 20000;
                                break;
                                
                            case 'S':
                                printf("Moving BACKWARD 20K steps...\n");
                                left_steps = -20000;
                                right_steps = -20000;
                                break;
                                
                            case 'A':
                                printf("Turning LEFT 20K steps (calibrated)...\n");
                                left_steps = 20000 - (20000 * LEFT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES);
                                right_steps = -(20000 - (20000 * LEFT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES));
                                break;
                                
                            case 'D':
                                printf("Turning RIGHT 20K steps (calibrated)...\n");
                                left_steps = -(20000 - (20000 * RIGHT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES));
                                right_steps = 20000 - (20000 * RIGHT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES);
                                break;
                        }
                        
                        stepper_steps(left_steps, right_steps);
                        
                        // Let it run for the delay time
                        printf("Running for %d ms... ", SNAKE_DELAY_MS);
                        fflush(stdout);
                        sleep_msec(SNAKE_DELAY_MS);
                        
                        // Get completed steps before stopping
                        stepper_get_completed_steps(&completed_left, &completed_right);
                        
                        // Stop movement
                        printf("Stopping.\n");
                        stepper_reset();
                        
                        // Update position/heading based on completed steps
                        update_position_from_steps(completed_left, completed_right, move);
                        
                        // Print movement details
                        printf("Completed %s: %d left, %d right\n", 
                               (move == 'W') ? "forward" : 
                               (move == 'S') ? "backward" : 
                               (move == 'A') ? "left turn" : "right turn", 
                               completed_left, completed_right);
                        
                        // For turns, print the heading change for debugging
                        if (move == 'A' || move == 'D') {
                            printf("Heading change: %.2f°\n", 
                                   compute_heading_change(completed_left, completed_right));
                        }
                        
                        // Print current position
                        print_position_status("Position:");
                    }
                    
                    printf("\n[✓] Snake Pattern Complete!\n");
                    printf("Final position: (%.2f, %.2f), Heading: %.2f°\n", 
                           current_x, current_y, heading_deg);
                    break;
                }


                case 'q':
                    printf("\nExiting calibration mode\n");
                    running = 0;
                    break;
                    
                default:
                    printf("Unknown command '%c'\n", c);
                    break;
            }
        }
    }

    stepper_disable();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}



void manual_mode() {
    printf("Starting manual mode (Snake game style)...\n");
    printf("Controls:\n");
    printf("  W: Start 20K steps forward\n");
    printf("  S: Start 20K steps backward\n");
    printf("  A: Start 20K steps left turn\n");
    printf("  D: Start 20K steps right turn\n");
    printf("  Q: Exit manual mode\n");
    printf("Press any direction key to change direction immediately\n");
    
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    stepper_reset();
    stepper_enable();
    
    // Track last completed steps for position/heading update
    int16_t completed_left = 0;
    int16_t completed_right = 0;
    char last_direction = '\0';

    int running = 1;
    while (running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval timeout = {0, 50000}; // 50ms timeout
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                // Get completed steps before changing direction
                if (last_direction != '\0') {
                    stepper_get_completed_steps(&completed_left, &completed_right);
                    update_position_from_steps(completed_left, completed_right, last_direction);
                }
                
                // Reset steppers for new direction
                stepper_reset();
                stepper_enable();
                
                // Set new direction with calibrated steps for turns
                switch(c) {
                    case 'w':
                        printf("\r[↑] Moving forward 20K steps...                     \n");
                        stepper_set_speed(30000, 30000);
                        stepper_steps(20000, 20000);
                        last_direction = 'w';
                        break;
                    case 's':
                        printf("\r[↓] Moving backward 20K steps...                    \n");
                        stepper_set_speed(30000, 30000);
                        stepper_steps(-20000, -20000);
                        last_direction = 's';
                        break;
                    case 'a':
                        // Left turn with adjustment
                        printf("\r[←] Turning left 20K steps (calibrated)...          \n");
                        stepper_set_speed(30000, 30000);
                        // Apply same adjustment ratio as in 90° turn
                        int16_t left_steps = 20000 - (20000 * LEFT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES);
                        int16_t right_steps = -(20000 - (20000 * LEFT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES));
                        stepper_steps(left_steps, right_steps);
                        last_direction = 'a';
                        break;
                    case 'd':
                        // Right turn with adjustment
                        printf("\r[→] Turning right 20K steps (calibrated)...         \n");
                        stepper_set_speed(30000, 30000);
                        // Apply same adjustment ratio as in 90° turn
                        int16_t r_left_steps = -(20000 - (20000 * RIGHT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES));
                        int16_t r_right_steps = 20000 - (20000 * RIGHT_TURN_ADJUSTMENT / STEPS_PER_90_DEGREES);
                        stepper_steps(r_left_steps, r_right_steps);
                        last_direction = 'd';
                        break;
                    case 'q':
                        printf("\nExiting manual mode\n");
                        running = 0;
                        break;
                }
                
                // Print current position and heading
                if (running) {
                    pthread_mutex_lock(&position_mutex);
                    printf("Position: (%.2f, %.2f), Heading: %.2f°\n", 
                           current_x, current_y, heading_deg);
                    pthread_mutex_unlock(&position_mutex);
                }
            }
        }
    }

    // Final stop
    stepper_reset();
    stepper_disable();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}

int main() {
    // Initialize the PYNQ library and stepper
    pynq_init();
    stepper_init();  // Single stepper initialization
    pthread_t uart_thread;
    pthread_create(&uart_thread, NULL, uart_position_publisher, NULL);


    printf("IR Sensor Reading Program\n");
    printf("Commands:\n");
    printf("  i: Start reading IR sensors\n");
    printf("  f: Follow line mode\n");
    printf("  d: Detect line mode\n");
    printf("  m: Manual control mode\n");
    printf("  q: Quit program\n");

    char cmd;
    while (1) {
        printf("\nEnter command: ");
        scanf(" %c", &cmd);

        switch (cmd) {                                    
            case 'm':
                manual_mode();
                break;
            
            case 'q':
                printf("Exiting program\n");
                stepper_destroy();  // Single stepper cleanup
                pynq_destroy();
                return 0;
            case 'c':
                calibration_mode();
                break;
            
            default:
                printf("Unknown command '%c'\n", cmd);
                break;
        }
    }

    // Cleanup in case we break out of the loop
    stepper_destroy();
    pynq_destroy();
    return 0;
}