#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stepper.h>
#include <termios.h>
#include <unistd.h>

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

int main(void) {
    struct termios old_settings;
    char key;
    int left_speed, right_speed;
    const int STEP_COUNT = 100;  // Variable for step count
    const int CUSTOM_SPEED = 20000;  // Our custom speed
    
    printf("DEBUG: Initializing program with STEP_COUNT=%d, CUSTOM_SPEED=%d\n", STEP_COUNT, CUSTOM_SPEED);
    
    // Calculate microseconds per step at our custom speed
    // If 3072 steps/s = 30us, then 10000 steps/s = (30 * 3072) / 10000 us
    const float US_PER_STEP = (30.0f * 3072) / CUSTOM_SPEED;  // ≈ 9.216 us per step
    
    // Calculate total time for steps in milliseconds
    // Time = (steps * microseconds per step) / 1000 to get milliseconds
    int STEP_TIME_MS = (int)((STEP_COUNT * US_PER_STEP) / 1000);
    
    printf("DEBUG: Calculated timing - US_PER_STEP=%.3f, STEP_TIME_MS=%d\n", US_PER_STEP, STEP_TIME_MS);
    
    printf("DEBUG: Initializing PYNQ and stepper motors...\n");
    pynq_init();
    stepper_init();
    stepper_enable();
    printf("DEBUG: Initialization complete\n");
    
    // Configure terminal for immediate key reading
    configure_terminal(&old_settings);
    
    printf("Use arrow keys to control. Press 'q' to quit.\n");
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
                        break;
                    case 'B': // Down arrow
                        printf("DEBUG: Moving BACKWARD - Steps: %d,%d Speed: %d,%d\n", 
                               -STEP_COUNT, -STEP_COUNT, left_speed, right_speed);
                        stepper_set_speed(left_speed, right_speed);
                        stepper_steps(-STEP_COUNT, -STEP_COUNT);
                        break;
                    case 'C': // Right arrow
                        printf("DEBUG: Turning RIGHT - Steps: %d,%d Speed: %d,%d\n", 
                               -STEP_COUNT, STEP_COUNT, left_speed, right_speed);
                        stepper_set_speed(left_speed, right_speed);
                        stepper_steps(-STEP_COUNT, STEP_COUNT);
                        break;
                    case 'D': // Left arrow
                        printf("DEBUG: Turning LEFT - Steps: %d,%d Speed: %d,%d\n", 
                               STEP_COUNT, -STEP_COUNT, left_speed, right_speed);
                        stepper_set_speed(left_speed, right_speed);
                        stepper_steps(STEP_COUNT, -STEP_COUNT);
                        break;
                }
                printf("DEBUG: Movement complete\n");
            } else if (key == 'q') {
                printf("DEBUG: Quit command received\n");
                break;
            }
        }
    }
    
    printf("DEBUG: Cleaning up and shutting down...\n");
    // Cleanup
    restore_terminal(&old_settings);
    stepper_disable();
    stepper_destroy();
    pynq_destroy();
    printf("DEBUG: Cleanup complete\n");
    return EXIT_SUCCESS;
}
