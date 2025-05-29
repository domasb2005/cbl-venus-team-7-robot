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


#define THRESHOLD 0.18
#define LEFT_THRESHOLD 0.3


#define IR_2L ADC0  // Middle - left (a0)
#define IR_1L ADC3  // Closest to center - left (a1)
#define IR_3L ADC2  // Furthest from center - left (a2)
#define IR_3R ADC1  // Furthest from center - right (a3) -line follower
#define IR_1R ADC4  // Closest to center - right (a4) -line follower
#define IR_2R ADC5  // Middle - right (a5) - line follower



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


void get_steps_and_restart(int16_t* left_steps, int16_t* right_steps) {
    stepper_get_completed_steps(left_steps, right_steps);
    stepper_reset();
    stepper_enable();
}


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
                    // Replace these individual calls:
                    // stepper_get_completed_steps(&completed_left, &completed_right);
                    // stepper_reset();
                    // stepper_enable();
                    
                    // With the helper function:
                    get_steps_and_restart(&completed_left, &completed_right);
                    
                    update_position_from_steps(completed_left, completed_right, last_direction);
                }
                
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

void read_ir_sensors() {
    double ir_values[6];
    uint32_t ir_raw_values[6];
    char *sensor_names[] = {"1L", "1R", "2L", "2R", "3L", "3R"};
    int sensor_pins[] = {IR_1L, IR_1R, IR_2L, IR_2R, IR_3L, IR_3R};
    
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    
    
    printf("\033[2J\033[H");
    printf("IR Sensor Readings (press 'q' to exit):\n\n");
    for (int i = 0; i < 6; i++) {
        printf("IR %s: 0.00V (    0)\n", sensor_names[i]);
    }
    
    int running = 1;
    while (running) {
        printf("\033[6A");
        
        for (int i = 0; i < 6; i++) {
            ir_values[i] = adc_read_channel(sensor_pins[i]);
            ir_raw_values[i] = adc_read_channel_raw(sensor_pins[i]);
            printf("\033[K");
            printf("IR %s: %.2fV (%5u)\n", sensor_names[i], ir_values[i], ir_raw_values[i]);
        }
        
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0 && c == 'q') {
                printf("\nExiting IR reading mode\n");
                running = 0;
            }
        }
        
        sleep_msec(100);
    }
    
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}
void align_line() {
    stepper_enable();

    printf("\n[↺] Aligning with line: turning left until IR_2R goes OFF...\n");
    // Turn left until IR_2R becomes OFF (below threshold)
    stepper_set_speed(30000, 30000);
    stepper_steps(5000, 5000); // Start a left turn (arbitrary large step count)
    while (1) {
        double ir_r2 = adc_read_channel(IR_2R);
        double ir_r1 = adc_read_channel(IR_1R);
        double ir_r3 = adc_read_channel(IR_3R);
        if (ir_r2 < THRESHOLD && ir_r1 < THRESHOLD && ir_r3 < THRESHOLD) {
            printf("[✓] IR_2R is OFF (%.2fV < %.2f). Stopping turn.\n", ir_r2, THRESHOLD);
            break;
        }
        sleep_msec(1);
    }
    
    // Use the helper function instead of raw reset/enable
    int16_t completed_left, completed_right;
    get_steps_and_restart(&completed_left, &completed_right);
    
    // Update position based on this first part of alignment
    update_position_from_steps(completed_left, completed_right, 'w');
    sleep_msec(1000); // Small delay to ensure position is updated

    // stepper_set_speed(60000, 60000); // Set speed for next part of alignment
    // stepper_steps(100, 100);
    // while(!stepper_steps_done()){
    //     sleep_msec(1);
    // }
    // update_position_from_steps(100, -100, 'w');
    // sleep_msec(1000); // Small delay to ensure position is updated

    printf("[↺] Continuing left turn until IR_2R goes ON again...\n");
    stepper_set_speed(65500, 65500);
    stepper_steps(5000, -5000); // Continue left turn
    while (1) {
        double ir_r2 = adc_read_channel(IR_2R);
        double ir_r1 = adc_read_channel(IR_1R);
        double ir_r3 = adc_read_channel(IR_3R);
        if (ir_r1 > THRESHOLD) {
            printf("[✓] IR_1R is ON again (%.2fV > %.2f). Stopping turn. IRR1 WORKS\n", ir_r1, THRESHOLD);
            break;
        }
        if (ir_r3 > THRESHOLD) {
            printf("[✓] IR_3 is ON again (%.2fV > %.2f). R2 MISSED THE TAPE Stopping turn.\n", ir_r3, THRESHOLD);
            break;
        }
        printf("\r[↺] IR_2R: %.2fV", ir_r2);
        if (ir_r2 > THRESHOLD) {
            printf("[✓] IR_2R is ON again (%.2fV > %.2f). Alignment complete.\n", ir_r2, THRESHOLD);
            break;
        }
        sleep_msec(1);
    }
    
    // Use the helper function again for the second part of alignment
    get_steps_and_restart(&completed_left, &completed_right);
    
    // Update position based on this second part of alignment
    update_position_from_steps(completed_left, completed_right, 'a');
    
    // Print final position after alignment
    print_position_status("[🧭] After alignment");

    printf("[✔] Line alignment completed.\n");
}

void detect_line() {
    printf("\n Starting detect_line mode (coarse movement)...\n");

    stepper_reset();
    stepper_enable();

    const int FORWARD_STEPS = 20000;
    const int TURN_STEPS = 625;  // ~90° calibrated

    while (1) {
        printf("\n[↑] Driving forward %d steps...\n", FORWARD_STEPS);
        stepper_set_speed(30000, 30000);
        stepper_steps(FORWARD_STEPS, FORWARD_STEPS);

        char condition = 'x';  // Unknown until we check
        while (!stepper_steps_done()) {
            double ir_r1 = adc_read_channel(IR_1R);
            double ir_r2 = adc_read_channel(IR_2R);
            double ir_r3 = adc_read_channel(IR_3R);
            double ir_l1 = adc_read_channel(IR_1L);
            double ir_l2 = adc_read_channel(IR_2L);

            if (ir_r2 > THRESHOLD) {
                printf("[✓] IR_2R saw black — ready to align directly.\n");
                condition = '2';  // Use single character
                break;
            } else if (ir_r3 > THRESHOLD) {
                printf("[✓] IR_3R saw black — continue forward until R2 sees black.\n");
                condition = '3';  // Use single character
                break;
            } else if (ir_l1 > LEFT_THRESHOLD || ir_l2 > LEFT_THRESHOLD || ir_r1 > THRESHOLD) {
                printf("[✓] Left IR saw black — rotate left until R2 sees black.\n");
                condition = 'l';
                break;
            }

            sleep_msec(50);
        }

        int16_t completed_left, completed_right;
        get_steps_and_restart(&completed_left, &completed_right);
        
        update_position_from_steps(completed_left, completed_right, 'w');
        print_position_status("[🧭] After forward");

        // === Handle logic branch ===
        if (condition == '2') {  // R2 sensor detected black
            align_line();
            break;

        } else if (condition == '3') {  // R3 sensor detected black
            printf("[→] Driving forward until R2 sees black...\n");
            while (1) {
                stepper_set_speed(30000, 30000);
                stepper_steps(FORWARD_STEPS, FORWARD_STEPS);

                while (!stepper_steps_done()) {
                    double ir_r2 = adc_read_channel(IR_2R);
                    if (ir_r2 > THRESHOLD) {
                        printf("[✓] IR_2R now on black. Proceeding to alignment.\n");
                        goto forward_done;
                    }
                    sleep_msec(50);
                }

                get_steps_and_restart(&completed_left, &completed_right);
                
                update_position_from_steps(completed_left, completed_right, 'w');
                print_position_status("[🧭] After extra forward");
            }
        forward_done:
            get_steps_and_restart(&completed_left, &completed_right);
            
            align_line();
            break;

        } else if (condition == 'l') {
            printf("[↺] Turning left until R2 sees black...\n");
            stepper_set_speed(30000, 30000);
            stepper_steps(TURN_STEPS * 4, -TURN_STEPS * 4);

            while (!stepper_steps_done()) {
                double ir_r2 = adc_read_channel(IR_2R);
                if (ir_r2 > THRESHOLD) {
                    printf("[✓] IR_2R now on black. Proceeding to alignment.\n");
                    break;
                }
                sleep_msec(50);
            }

            get_steps_and_restart(&completed_left, &completed_right);
            
            update_position_from_steps(completed_left, completed_right, 'a');
            print_position_status("[🧭] After left turn");

            align_line();
            break;
        } else {
            printf("[✗] No valid sensor condition met. Retrying...\n");
        }
    }

    stepper_disable();
}




int main() {
    // Initialize the PYNQ library and stepper
    pynq_init();
    stepper_init();  // Single stepper initialization
    adc_init();  // Initialize ADC for IR sensors
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
            case 'a':
            printf("Starting alignment mode...\n");
                align_line();
                break;                        
            case 'm':
                manual_mode();
                break;
            case 'i':
                read_ir_sensors();
                break;

            case 'q':
                printf("Exiting program\n");
                stepper_destroy();  // Single stepper cleanup
                pynq_destroy();
                return 0;
            case 'c':
                break;
            case 'd':
                printf("Starting detect line mode...\n");
                detect_line();
                break;
            
            default:
                printf("Unknown command '%c'\n", cmd);
                break;
        }
    }

    // Cleanup in case we break out of the loop
    stepper_destroy();
    pynq_destroy();
    adc_destroy();

    return 0;
}