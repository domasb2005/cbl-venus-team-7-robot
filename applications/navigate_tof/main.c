#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stepper.h>
#include <termios.h>
#include <unistd.h>
#include "vl53l0x.h"
#include <iic.h>
#include <time.h>
#include <sys/select.h>  // Add this for fd_set, FD_ZERO, FD_SET and select
#include <sys/time.h>    // Add this for struct timeval
#define TOF_ADDR 0x29
// Global sensor structure
static vl53x sensor;
static int sensor_initialized = 0;

// Structure to hold infrared sensor readings
typedef struct {
    gpio_level_t left;    // AR13
    gpio_level_t center;  // AR12
    gpio_level_t right;   // AR11
} infrared_readings_t;

// Function to configure terminal for immediate key reading
void configure_terminal(struct termios *old) {
    struct termios new;
    tcgetattr(STDIN_FILENO, old);
    new = *old;
    new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new);
}

// Function to restore terminal settings
void restore_terminal(struct termios *old) {
    tcsetattr(STDIN_FILENO, TCSANOW, old);
}

uint32_t read_distance_low(void) {
    if (!sensor_initialized) {
        // Initialize I2C if not done yet
        switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
        switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
        iic_init(IIC0);

        // Verify sensor connectivity
        if (tofPing(IIC0, TOF_ADDR) != 0) {
            printf("Error: TOF sensor not responding\n");
            return 0;
        }

        // Initialize sensor
        if (tofInit(&sensor, IIC0, TOF_ADDR, 0) != 0) {
            printf("Failed to initialize sensor\n");
            return 0;
        }
        
        sensor_initialized = 1;
    }

    return tofReadDistance(&sensor);
}

infrared_readings_t read_infrared(void) {
    infrared_readings_t readings;
    
    // Initialize GPIO
    gpio_init();
    
    // Set pins as input
    gpio_set_direction(IO_AR13, GPIO_DIR_INPUT);
    gpio_set_direction(IO_AR12, GPIO_DIR_INPUT);
    gpio_set_direction(IO_AR11, GPIO_DIR_INPUT);
    
    // Read levels
    readings.left = gpio_get_level(IO_AR13);
    readings.center = gpio_get_level(IO_AR11);
    readings.right = gpio_get_level(IO_AR12);
    
    return readings;
}

void read_and_print_sensors(void) {
    // Read TOF sensor
    uint32_t distance = read_distance_low();
    printf("TOF Distance = %dmm\n", distance);
    
    // Read IR sensors
    infrared_readings_t ir_readings = read_infrared();
    printf("IR Sensors - Left: %d, Center: %d, Right: %d\n", 
           ir_readings.left, ir_readings.center, ir_readings.right);
}

// Function to make a 90-degree right turn
void right_90(void) {
    printf("Performing 90-degree right turn\n");
    stepper_enable();

    stepper_steps(-615, 615);
    sleep_msec(500);  // Small delay after turn
}

// Function to make a 90-degree left turn
void left_90(void) {
    printf("Performing 90-degree left turn\n");
    stepper_enable();
    stepper_steps(615, -615);  // 7 * 100 steps, inverse for left turn
    sleep_msec(500);  // Small delay after turn
}


void auto_mode(int custom_speed) {
    // Threshold for black/white detection in volts
    const double THRESHOLD = 1.15;      // 1V threshold for black detection
    
    printf("Starting edge following auto mode with ADC IR sensors (threshold: %.2fV)\n", THRESHOLD);
    
    int running = 1;
    int iterations = 0;
    const int MAX_ITERATIONS = 15000; // Safety limit
    
    // Speed settings for different scenarios
    const int FORWARD_SPEED = custom_speed;
    const int RIGHT_TURN_SPEED = custom_speed / 3;    // Faster/stronger right turns
    const int LEFT_TURN_SPEED = custom_speed / 1.5;   // More gentle left turns
    
    // Step sizes - bigger for right turns
    const int SMALL_STEP = 2;           // Basic step size
    const int RIGHT_TURN_STEP = 4;      // Bigger steps for right turns
    
    // Initialize ADC once
    adc_init();
    
    stepper_enable();
    stepper_set_speed(FORWARD_SPEED, FORWARD_SPEED);
    
    // Track current state to avoid redundant prints
    int current_state = -1;
    int prev_state = -1;
    
    while(running) {
        iterations++;
        
        // Read IR sensors using ADC
        double ir_left = adc_read_channel(ADC0);     // A0
        double ir_center = adc_read_channel(ADC2);   // A2
        double ir_right = adc_read_channel(ADC1);    // A1
        
        // Print current readings
        printf("\rIR Sensors: Left=%.2fV, Center=%.2fV, Right=%.2fV    ", 
               ir_left, ir_center, ir_right);
        fflush(stdout);
        
        // Check for 'q' key press
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms timeout
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char key;
            if (read(STDIN_FILENO, &key, 1) > 0) {
                if (key == 'q') {
                    stepper_reset();
                    printf("\nExiting turn180 mode\n");
                    running = 0;
                    break;
                }
            }
        }
        
        // Check if any sensor detects black (above threshold)
        int left_value = (ir_left < THRESHOLD) ? 1 : 0;
        int center_value = (ir_center < THRESHOLD) ? 1 : 0;
        int right_value = (ir_right < THRESHOLD) ? 1 : 0;
        
        // Print sensor readings only occasionally
        if (iterations % 50 == 0) {
            printf("\nIteration %d - IR Sensors: Left=%.2fV (%d), Center=%.2fV (%d), Right=%.2fV (%d)\n", 
                  iterations, ir_left, left_value, ir_center, center_value, ir_right, right_value);
            printf("TOF Distance: %dmm\n", read_distance_low());
        }
        
        // Determine current state using the binary values from ADC readings
        if (left_value == 0 && center_value == 0 && right_value == 0) {
            current_state = 0; // Final state - all on black
        } else if (left_value == 1 && center_value == 1 && right_value == 1) {
            current_state = 1; // All white
        } else if (left_value == 0 && center_value == 1 && right_value == 1) {
            current_state = 2; // Left on black
        } else if (left_value == 0 && center_value == 0 && right_value == 1) {
            current_state = 3; // Left+center on black
        } else if (left_value == 1 && center_value == 1 && right_value == 0) {
            current_state = 4; // Right on black
        } else if (left_value == 1 && center_value == 0 && right_value == 0) {
            current_state = 5; // Right+center on black
        } else if (left_value == 1 && center_value == 0 && right_value == 1) {
            current_state = 6; // Only center on black
        } else if (left_value == 0 && center_value == 1 && right_value == 0) {
            current_state = 7; // Inconsistent
        }
        
        // Only print when state changes
        if (current_state != prev_state) {
            switch (current_state) {
                case 0:
                    printf("Goal reached: All sensors on black edge - STOPPING\n");
                    break;
                case 1:
                    printf("All sensors on white - Moving forward\n");
                    break;
                case 2:
                    printf("Left sensor on black edge - Right turn (light)\n");
                    break;
                case 3:
                    printf("Left+Center sensors on black edge - Right turn (medium)\n");
                    break;
                case 4:
                    printf("Right sensor on black edge - Left turn using right wheel\n");
                    break;
                case 5:
                    printf("Right+Center sensors on black edge - Left turn (strong) using right wheel\n");
                    break;
                case 6:
                    printf("Only center on black - Small forward adjustment\n");
                    break;
                case 7:
                    printf("Inconsistent reading (both sides) - Correcting\n");
                    break;
            }
            prev_state = current_state;
        }
        
        // FINAL STATE: All sensors on black (0) - stop
        if (current_state == 0) {
            running = 0; // Exit the loop
            break;
        }
        // All sensors on white (1) - move forward
        else if (current_state == 1) {
            stepper_set_speed(FORWARD_SPEED, FORWARD_SPEED);
            stepper_steps(SMALL_STEP, SMALL_STEP);
        }
        // Left sensor detects black (0) - right turn (reduce right wheel speed)
        else if (current_state == 2) {
            stepper_set_speed(RIGHT_TURN_SPEED, RIGHT_TURN_SPEED);
            stepper_steps(-RIGHT_TURN_STEP/3, -RIGHT_TURN_STEP); // Stronger right turn with left wheel only
        }
        // Left and center sensors detect black (0) - stronger right turn
        else if (current_state == 3) {
            stepper_set_speed(RIGHT_TURN_SPEED, RIGHT_TURN_SPEED / 2); // Even stronger turn
            stepper_steps(-RIGHT_TURN_STEP/3, -RIGHT_TURN_STEP); // Only left wheel moves, more steps
        }
        // Right sensor detects black (0) - left turn using right wheel
        else if (current_state == 4) {
            stepper_set_speed(LEFT_TURN_SPEED, LEFT_TURN_SPEED);
            stepper_steps(-SMALL_STEP*2, -SMALL_STEP*2); // Only right wheel moves
            stepper_steps(-SMALL_STEP*2, -SMALL_STEP/10*9*2); // Only right wheel moves
        }
        // Right and center sensors detect black (0) - stronger left turn using right wheel
        else if (current_state == 5) {
            stepper_set_speed(LEFT_TURN_SPEED, LEFT_TURN_SPEED);
            stepper_steps(-SMALL_STEP*2, -SMALL_STEP*2); // Only right wheel moves
            stepper_steps(-SMALL_STEP*2, -SMALL_STEP/10*9*2); // Only right wheel moves
        }
        // Only center sensor on black (0) - small adjustments
        else if (current_state == 6) {
            stepper_set_speed(LEFT_TURN_SPEED, LEFT_TURN_SPEED);
            stepper_steps(SMALL_STEP, SMALL_STEP); // Both wheels move but slower
        }
        // Inconsistent readings (both sides but no center)
        else if (current_state == 7) {
            stepper_set_speed(LEFT_TURN_SPEED, LEFT_TURN_SPEED);
            stepper_steps(SMALL_STEP, SMALL_STEP); // Move forward cautiously
        }
    }
    
    if (iterations >= MAX_ITERATIONS) {
        printf("\nReached maximum iterations (%d) - stopping auto mode\n", MAX_ITERATIONS);
    }
    
    stepper_disable();
    adc_destroy(); // Clean up ADC resources
    printf("\nEdge following complete\n");
}

void ir_analog_mode(void) {
    stepper_reset();
    stepper_disable();
    printf("Starting analog IR sensor mode\n");
    printf("Reading values from A0-A5 pins\n");
    printf("Press 'q' to quit this mode\n\n");
    
    struct termios old_settings, new_settings;
    char key;
    
    // Configure terminal for non-blocking input
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    
    // Initialize ADC
    adc_init();
    
    int running = 1;
    int iterations = 0;
    
    while(running) {
        // Read ADC values from all 6 channels
        double adc_values[6];
        uint32_t adc_raw_values[6];
        
        // Read all channels
        adc_values[0] = adc_read_channel(ADC0);     // A0
        adc_values[1] = adc_read_channel(ADC1);     // A1
        adc_values[2] = adc_read_channel(ADC2);     // A2
        adc_values[3] = adc_read_channel(ADC3);     // A3
        adc_values[4] = adc_read_channel(ADC4);     // A4
        adc_values[5] = adc_read_channel(ADC5);     // A5
        
        // Get raw values
        adc_raw_values[0] = adc_read_channel_raw(ADC0);
        adc_raw_values[1] = adc_read_channel_raw(ADC1);
        adc_raw_values[2] = adc_read_channel_raw(ADC2);
        adc_raw_values[3] = adc_read_channel_raw(ADC3);
        adc_raw_values[4] = adc_read_channel_raw(ADC4);
        adc_raw_values[5] = adc_read_channel_raw(ADC5);
        
        // Display values every 10 iterations to avoid flooding the console
        if (iterations % 2 == 0) {
            printf("\rADC Values - ");
            for (int i = 0; i < 6; i++) {
                printf("A%d: %.2fV (%5u) | ", 
                       i, adc_values[i], adc_raw_values[i]);
            }
            printf("    \r");
            fflush(stdout);
        }
        
        // Check for quit command
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms timeout
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            if (read(STDIN_FILENO, &key, 1) > 0) {
                if (key == 'q') {
                    running = 0;
                    printf("\nExiting IR analog mode\n");
                }
            }
        }
        
        iterations++;
        sleep_msec(100); // Delay between readings
    }
    
    // Clean up
    adc_destroy();
    
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}

// Function to perform 180-degree turn sequence
void turn180_mode(void) {
    printf("Starting turn180 mode - moving forward until black tape detection\n");
    
    // Initialize ADC
    adc_init();
    stepper_reset();
    stepper_enable();
    stepper_set_speed(20000, 20000); // Set speed for forward movement
    stepper_steps(20000,20000);
    
    int running = 1;
    const double THRESHOLD = 0.20;  // Same threshold as auto mode
    const int FORWARD_STEP = 20;    // Small steps like in auto mode 
    
    // Configure terminal for non-blocking input
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    
    while(running) {
        // Read all 6 IR sensors using ADC
        double ir_left = adc_read_channel(ADC0);     // A0
        double ir_center = adc_read_channel(ADC2);   // A2
        double ir_right = adc_read_channel(ADC1);    // A1
        double ir_a3 = adc_read_channel(ADC3);       // A3
        double ir_a4 = adc_read_channel(ADC4);       // A4
        double ir_a5 = adc_read_channel(ADC5);       // A5
        
        // Print current readings for all 6 sensors
        printf("\rIR Sensors: Left=%.2fV, Center=%.2fV, Right=%.2fV, A3=%.2fV, A4=%.2fV, A5=%.2fV    ", 
               ir_left, ir_center, ir_right, ir_a3, ir_a4, ir_a5);
        fflush(stdout);
        
        // Check for 'q' key press
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms timeout
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char key;
            if (read(STDIN_FILENO, &key, 1) > 0) {
                if (key == 'q') {
                    stepper_reset();
                    printf("\nExiting turn180 mode\n");
                    running = 0;
                    break;
                }
            }
        }
        
        // Check if any of the 6 sensors detects black (above threshold)
        if (ir_right >= THRESHOLD || ir_left >= THRESHOLD || ir_center >= THRESHOLD ||
            ir_a3 >= THRESHOLD || ir_a4 >= THRESHOLD || ir_a5 >= THRESHOLD) {
            stepper_reset();
            stepper_enable();
            printf("\nBlack tape detected! Executing 180-degree turn sequence\n");
            sleep_msec(1000);  // Small delay

            // Step 1: Reverse 300 steps
            printf("1. Moving backwards 300 steps...\n");
            stepper_steps(-300, -300);
            sleep_msec(1000);  // Small delay
            
            // Step 2: First 90-degree right turn
            printf("2. Performing first 90-degree right turn...\n");
            right_90();
            sleep_msec(1000);  // Small delay
            
            // Step 3: Forward 1000 steps
            printf("3. Moving forward 1000 steps...\n");
            stepper_steps(1000, 1000);
            sleep_msec(2000);  // Small delay
            
            // Step 4: Second 90-degree right turn
            printf("4. Performing second 90-degree right turn...\n");
            right_90();
            sleep_msec(2000);  // Small delay

            
            printf("180-degree turn sequence complete!\n");
            running = 0;  // Exit the mode
        }
        
        sleep_msec(1);  // Small delay between readings
    }
    
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    
    stepper_disable();
    adc_destroy();  // Clean up ADC resources
}

int main(void) {
    struct termios old_settings;
    char mode;
    char key;
    int left_speed, right_speed;
    const int STEP_COUNT = 10;
    const int SMALL_MANUAL_STEP = 100;  // New constant for manual mode
    const int CUSTOM_SPEED = 20000;
    
    // Calculate timing information
    const float US_PER_STEP = (30.0f * 3072) / CUSTOM_SPEED;
    int STEP_TIME_MS = (int)((STEP_COUNT * US_PER_STEP) / 1000);
    
    
    // Initialize all systems
    pynq_init();
    stepper_init();
    stepper_enable();
    
    while(1) {  // Main loop to keep asking for mode
        // Mode selection
        printf("Select mode:\n");
        printf("'m' - Manual control (arrow keys)\n");
        printf("'a' - Auto mode (forward until black edge)\n");
        printf("'i' - IR analog mode (read ADC values)\n");
        printf("'t' - Turn180 mode (180-degree turn on black detection)\n");  // New option
        printf("'q' - Quit program\n");
        printf("Enter mode (m/a/i/t/q): ");  // Updated prompt
        scanf(" %c", &mode);
        
        if (mode == 'q') {
            break;
        } else if (mode == 'a') {
            auto_mode(CUSTOM_SPEED);
        } else if (mode == 't') {  // New mode
            turn180_mode();
        } else if (mode == 'm') {
            configure_terminal(&old_settings);
            
            // Clear screen and print fixed sections
            /*printf("\033[2J\033[H"); // Clear screen and move cursor to top
            printf("Manual Control Mode Instructions:\n");
            printf("--------------------------------\n");
            printf("Use arrow keys to control the robot:\n");
            printf("↑ - Forward\n");
            printf("↓ - Backward\n");
            printf("← - Left turn\n");
            printf("→ - Right turn\n");
            printf("Press 'q' to quit\n");
            printf("\n");
            printf("Sensor Readings:\n");
            printf("---------------\n");
            printf("TOF Distance: \n");
            printf("IR Sensors:   \n");*/
            
            int running = 1;
            while (running) {
                // Set up select for non-blocking input
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(STDIN_FILENO, &readfds);
                
                struct timeval timeout;
                timeout.tv_sec = 0;
                timeout.tv_usec = 50000; // 50ms timeout for sensor updates
                
                // Move cursor to TOF reading position and update
                /*printf("\033[12;14H"); // Move to line 12, column 14*/
                uint32_t distance = read_distance_low();
                /*printf("%4dmm     ", distance);
                
                // Move cursor to IR reading position and update
                printf("\033[13;14H"); // Move to line 13, column 14*/
                infrared_readings_t ir_readings = read_infrared();
                /*printf("L:%d C:%d R:%d     ", 
                       ir_readings.left, 
                       ir_readings.center, 
                       ir_readings.right);
                
                fflush(stdout);*/
                
                // Check for input with timeout
                if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
                    if (read(STDIN_FILENO, &key, 1) == 1) {
                        left_speed = CUSTOM_SPEED;
                        right_speed = CUSTOM_SPEED;
                        
                        // Move cursor to message area for movement feedback
                        /*printf("\033[15;1H\033[K"); // Move to line 15 and clear line*/
                        
                        if (key == '\033') { // Escape sequence
                            read(STDIN_FILENO, &key, 1); // Skip '['
                            read(STDIN_FILENO, &key, 1); // Get actual key
                            
                            switch (key) {
                                case 'A': // Up arrow
                                    /*printf("Moving Forward");*/
                                    stepper_set_speed(left_speed, right_speed);
                                    stepper_steps(SMALL_MANUAL_STEP, SMALL_MANUAL_STEP);
                                    break;
                                case 'B': // Down arrow
                                    /*printf("Moving Backward");*/
                                    stepper_set_speed(left_speed, right_speed);
                                    stepper_steps(-SMALL_MANUAL_STEP, -SMALL_MANUAL_STEP);
                                    break;
                                case 'C': // Right arrow
                                    /*printf("Turning Right");*/
                                    stepper_set_speed(left_speed, right_speed);
                                    stepper_steps(-SMALL_MANUAL_STEP, 0);
                                    break;
                                case 'D': // Left arrow
                                    /*printf("Turning Left");*/
                                    stepper_set_speed(left_speed, right_speed);
                                    stepper_steps(SMALL_MANUAL_STEP, 0);
                                    break;
                            }
                        } else if (key == 'q') {
                            /*printf("\033[2J\033[H"); // Clear screen before exiting*/
                            running = 0;
                        }
                    }
                }
            }
            restore_terminal(&old_settings);
        } else if (mode == 'i') {  // New mode
            ir_analog_mode();
        } else {
            printf("Invalid mode selected. Please try again.\n");
        }
    }
    
    printf("DEBUG: Cleaning up and shutting down...\n");
    stepper_disable();
    stepper_destroy();
    pynq_destroy();
    printf("DEBUG: Cleanup complete\n");
    return EXIT_SUCCESS;
}