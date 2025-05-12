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

    stepper_steps(-600, 600);
    sleep_msec(500);  // Small delay after turn
}

// Function to make a 90-degree left turn
void left_90(void) {
    printf("Performing 90-degree left turn\n");
    stepper_enable();
    stepper_steps(600, -600);  // 7 * 100 steps, inverse for left turn
    sleep_msec(500);  // Small delay after turn
}


void auto_mode(int custom_speed) {
    printf("Starting edge following auto mode with ADC IR sensors (threshold: 0.25V)\n");
    
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
    
    // Threshold for black/white detection in volts
    const double THRESHOLD = 0.4;      // 0.25V threshold for black detection
    
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
        
        // Convert to binary values using threshold (below threshold = 0 = black, above = 1 = white)
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
    
    stepper_set_speed(0, 0);
    stepper_disable();
    adc_destroy(); // Clean up ADC resources
    printf("\nEdge following complete\n");
}

void ir_analog_mode(void) {
    printf("Starting analog IR sensor mode\n");
    printf("Reading values from A0, A1, A2 pins\n");
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
        // Read ADC values from A0, A1, A2 pins (left, center, right IR sensors)
        double ir_left = adc_read_channel(ADC0);     // A0
        double ir_center = adc_read_channel(ADC2);   // A1
        double ir_right = adc_read_channel(ADC1);    // A2
        
        // Also get raw values (0-65535)
        uint32_t ir_left_raw = adc_read_channel_raw(ADC0);
        uint32_t ir_center_raw = adc_read_channel_raw(ADC2);
        uint32_t ir_right_raw = adc_read_channel_raw(ADC1);
        
        // Display values every 10 iterations to avoid flooding the console
        if (iterations % 10 == 0) {
            printf("\rIR Sensors - Left: %.2fV (%5u), Center: %.2fV (%5u), Right: %.2fV (%5u)    ", 
                   ir_left, ir_left_raw, 
                   ir_center, ir_center_raw, 
                   ir_right, ir_right_raw);
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

int main(void) {
    struct termios old_settings;
    char mode;
    char key;
    int left_speed, right_speed;
    const int STEP_COUNT = 10;
    const int CUSTOM_SPEED = 20000;
    
    // Calculate timing information
    const float US_PER_STEP = (30.0f * 3072) / CUSTOM_SPEED;
    int STEP_TIME_MS = (int)((STEP_COUNT * US_PER_STEP) / 1000);
    
    printf("DEBUG: Initializing program with STEP_COUNT=%d, CUSTOM_SPEED=%d\n", STEP_COUNT, CUSTOM_SPEED);
    printf("DEBUG: Calculated timing - US_PER_STEP=%.3f, STEP_TIME_MS=%d\n", US_PER_STEP, STEP_TIME_MS);
    
    // Initialize all systems
    printf("DEBUG: Initializing PYNQ and stepper motors...\n");
    pynq_init();
    stepper_init();
    stepper_enable();
    printf("DEBUG: Initialization complete\n");
    
    while(1) {  // Main loop to keep asking for mode
        // Mode selection
        printf("Select mode:\n");
        printf("'m' - Manual control (arrow keys)\n");
        printf("'a' - Auto mode (forward until black edge)\n");
        printf("'i' - IR analog mode (read ADC values)\n");  // New option
        printf("'q' - Quit program\n");
        printf("Enter mode (m/a/i/q): ");
        scanf(" %c", &mode);
        
        if (mode == 'q') {
            break;
        } else if (mode == 'a') {
            auto_mode(CUSTOM_SPEED);
        } else if (mode == 'm') {
            configure_terminal(&old_settings);
            
            printf("Manual mode - Use arrow keys to control. Press 'q' to quit.\n");
            printf("Each movement will take approximately %d milliseconds\n", STEP_TIME_MS);
            
            while (1) {
                if (read(STDIN_FILENO, &key, 1) == 1) {
                    left_speed = CUSTOM_SPEED;
                    right_speed = CUSTOM_SPEED;
                    
                    if (key == '\033') { // Escape sequence
                        read(STDIN_FILENO, &key, 1); // Skip '['
                        read(STDIN_FILENO, &key, 1); // Get actual key
                        
                        switch (key) {
                            case 'A': // Up arrow
                                printf("DEBUG: Moving FORWARD - Steps: %d,%d Speed: %d,%d\n", 
                                       STEP_COUNT, STEP_COUNT, left_speed, right_speed);
                                stepper_set_speed(left_speed, right_speed);
                                stepper_steps(STEP_COUNT, STEP_COUNT);
                                read_and_print_sensors();
                                break;
                            case 'B': // Down arrow
                                printf("DEBUG: Moving BACKWARD - Steps: %d,%d Speed: %d,%d\n", 
                                       -STEP_COUNT, -STEP_COUNT, left_speed, right_speed);
                                stepper_set_speed(left_speed, right_speed);
                                stepper_steps(-STEP_COUNT, -STEP_COUNT);
                                read_and_print_sensors();
                                break;
                            case 'C': // Right arrow
                                printf("DEBUG: Turning RIGHT - Steps: %d,%d Speed: %d,%d\n", 
                                       -STEP_COUNT, STEP_COUNT, left_speed, right_speed);
                                stepper_set_speed(left_speed, right_speed);
                                stepper_steps(-STEP_COUNT, 0);
                                read_and_print_sensors();
                                break;
                                case 'D': // Left arrow
                                printf("DEBUG: Turning LEFT - Steps: %d,%d Speed: %d,%d\n", 
                                       STEP_COUNT, -STEP_COUNT, left_speed, right_speed);
                                stepper_set_speed(left_speed, right_speed);
                                stepper_steps(STEP_COUNT, 0);  // Fixed missing parameter
                                read_and_print_sensors();
                                break;
                        }
                        printf("DEBUG: Movement complete\n");
                    } else if (key == 'q') {
                        printf("DEBUG: Quit command received\n");
                        break;
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