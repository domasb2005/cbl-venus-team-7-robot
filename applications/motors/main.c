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
    const int STEP_COUNT = 200;  // Variable for step count
    const int CUSTOM_SPEED = 15000;  // Our custom speed
    
    // Calculate microseconds per step at our custom speed
    // If 3072 steps/s = 30us, then 10000 steps/s = (30 * 3072) / 10000 us
    const float US_PER_STEP = (30.0f * 3072) / CUSTOM_SPEED;  // ≈ 9.216 us per step
    
    // Calculate total time for steps in milliseconds
    // Time = (steps * microseconds per step) / 1000 to get milliseconds
    int STEP_TIME_MS = (int)((STEP_COUNT * US_PER_STEP) / 1000);
    // STEP_TIME_MS -= 100;
    
    pynq_init();
    stepper_init();
    stepper_enable();
    
    // Configure terminal for immediate key reading
    configure_terminal(&old_settings);
    
    printf("Use arrow keys to control. Press 'q' to quit.\n");
    printf("Each movement will take approximately %d milliseconds\n", STEP_TIME_MS);
    
    while (1) {
        if (read(STDIN_FILENO, &key, 1) == 1) {
            left_speed = CUSTOM_SPEED;   // Fixed: Using CUSTOM_SPEED instead of MAX_SPEED
            right_speed = CUSTOM_SPEED;
            
            if (key == '\033') { // Escape sequence
                read(STDIN_FILENO, &key, 1); // Skip '['
                read(STDIN_FILENO, &key, 1); // Get actual key
                
                switch (key) {
                    case 'A': // Up arrow
                        stepper_set_speed(left_speed, right_speed);
                        stepper_steps(STEP_COUNT, STEP_COUNT);
                        // sleep_msec(STEP_TIME_MS);
                        break;
                    case 'B': // Down arrow
                        stepper_set_speed(left_speed, right_speed);
                        stepper_steps(-STEP_COUNT, -STEP_COUNT);
                        // sleep_msec(STEP_TIME_MS);
                        break;
                    case 'C': // Right arrow
                        stepper_set_speed(left_speed, right_speed);
                        stepper_steps(-STEP_COUNT, STEP_COUNT);
                        // sleep_msec(STEP_TIME_MS);
                        break;
                    case 'D': // Left arrow
                        stepper_set_speed(left_speed, right_speed);
                        stepper_steps(STEP_COUNT, -STEP_COUNT);
                        // sleep_msec(STEP_TIME_MS);
                        break;
                }
            } else if (key == 'q') {
                break;
            }
        }
    }
    
    // Cleanup
    restore_terminal(&old_settings);
    stepper_disable();
    stepper_destroy();
    pynq_destroy();
    return EXIT_SUCCESS;
}
