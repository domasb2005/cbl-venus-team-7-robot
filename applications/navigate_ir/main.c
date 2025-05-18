#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libpynq.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <stepper.h>

// IR sensor pin definitions (ADC channels)
#define IR_2L ADC0  // Middle - left (a0)
#define IR_1L ADC3  // Closest to center - left (a1)
#define IR_3L ADC2  // Furthest from center - left (a2)
#define IR_3R ADC1  // Furthest from center - right (a3) -line follower
#define IR_1R ADC4  // Closest to center - right (a4) -line follower
#define IR_2R ADC5  // Middle - right (a5) - line follower

void right_90(void) {
    stepper_steps(-615, 615);
}

#define THRESHOLD 0.19

void small_step_left() {
    stepper_enable();
    stepper_set_speed(15000, 15000);  // Slower for finer control
    stepper_steps(5, -5);           // Small left turn
    sleep_msec(50);
}

void small_step_forward() {
    stepper_enable();
    stepper_set_speed(15000, 15000);
    stepper_steps(5, 5);
    sleep_msec(50);
}

void small_step_right() {
    stepper_enable();
    stepper_set_speed(15000, 15000);
    stepper_steps(-5, 5);  // Right turn: left backward, right forward
    sleep_msec(50);
}

void follow_line() {
    printf("Starting follow_line mode (raw-based)...\n");

    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    adc_init();
    stepper_reset();
    stepper_enable();

    int running = 1;
    while (running) {
        // Read raw ADC values
        uint32_t raw_r1 = adc_read_channel_raw(IR_1R);
        uint32_t raw_r2 = adc_read_channel_raw(IR_2R);
        uint32_t raw_r3 = adc_read_channel_raw(IR_3R);

        printf("\rRaw: R1=%4u  R2=%4u  R3=%4u    ", raw_r1, raw_r2, raw_r3);
        fflush(stdout);

        if (raw_r2 > raw_r1 && raw_r2 > raw_r3) {
            small_step_forward();
        }
        else if (raw_r3 > raw_r2 && raw_r3 > raw_r1) {
            printf("\nAdjusting RIGHT until R2 becomes dominant...\n");
            while (!(raw_r2 > raw_r1 && raw_r2 > raw_r3) && running) {
                small_step_right();
                raw_r1 = adc_read_channel_raw(IR_1R);
                raw_r2 = adc_read_channel_raw(IR_2R);
                raw_r3 = adc_read_channel_raw(IR_3R);
                printf("\rRaw: R1=%4u  R2=%4u  R3=%4u    ", raw_r1, raw_r2, raw_r3);
                fflush(stdout);

                // Check for exit
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(STDIN_FILENO, &readfds);
                struct timeval timeout = {0, 0};
                if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
                    char c;
                    if (read(STDIN_FILENO, &c, 1) > 0 && c == 'q') {
                        printf("\nExiting follow_line during left adjustment.\n");
                        running = 0;
                        break;
                    }
                }
            }
        }
        else if (raw_r1 > raw_r2 && raw_r1 > raw_r3) {
            printf("\nAdjusting LEFT until R2 becomes dominant...\n");
            while (!(raw_r2 > raw_r1 && raw_r2 > raw_r3) && running) {
                small_step_left();
                raw_r1 = adc_read_channel_raw(IR_1R);
                raw_r2 = adc_read_channel_raw(IR_2R);
                raw_r3 = adc_read_channel_raw(IR_3R);
                printf("\rRaw: R1=%4u  R2=%4u  R3=%4u    ", raw_r1, raw_r2, raw_r3);
                fflush(stdout);

                // Check for exit
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(STDIN_FILENO, &readfds);
                struct timeval timeout = {0, 0};
                if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
                    char c;
                    if (read(STDIN_FILENO, &c, 1) > 0 && c == 'q') {
                        printf("\nExiting follow_line during right adjustment.\n");
                        running = 0;
                        break;
                    }
                }
            }
        } else {
            // No clear max – wait briefly
            printf("\nNo dominant sensor. Waiting...\n");
            sleep_msec(50);
        }

        // Global non-blocking check for 'q'
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval timeout = {0, 0};
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0 && c == 'q') {
                printf("\nExiting follow_line mode.\n");
                running = 0;
            }
        }
    }

    adc_destroy();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}


void detect_line() {
    printf("Starting detect_line mode...\n");

    // Initialize
    adc_init();
    stepper_reset();
    stepper_enable();
    stepper_set_speed(20000, 20000);
    stepper_steps(20000, 20000);  // Start moving forward

    int running = 1;
    while (running) {
        // Read all sensors
        double ir_1l = adc_read_channel(IR_1L);
        double ir_2l = adc_read_channel(IR_2L);
        double ir_3l = adc_read_channel(IR_3L);
        double ir_1r = adc_read_channel(IR_1R);
        double ir_2r = adc_read_channel(IR_2R);
        double ir_3r = adc_read_channel(IR_3R);

        printf("\rIRs: 1L=%.2f 2L=%.2f 3L=%.2f | 1R=%.2f 2R=%.2f 3R=%.2f    ",
               ir_1l, ir_2l, ir_3l, ir_1r, ir_2r, ir_3r);
        fflush(stdout);

        // If 2R is on black, we're aligned
        if (ir_2r > THRESHOLD) {
            printf("\n2R is on black. Alignment complete.\n");
            break;
        }

        // If any left-side sensors are on black → turn left until 2R is on black
        if (ir_1l > THRESHOLD || ir_2l > THRESHOLD || ir_3l > THRESHOLD) {
            stepper_reset();
            printf("\nLeft sensors detected black. Turning LEFT until 2R is on black...\n");

            while (adc_read_channel(IR_2R) <= THRESHOLD) {
                small_step_left();
                printf("\r2R=%.2fV    ", adc_read_channel(IR_2R));
                fflush(stdout);
            }
            break;
        }

        // If R1 or R3 detect black, move forward until 2R is on black
        if (ir_1r > THRESHOLD || ir_3r > THRESHOLD) {
            stepper_reset();
            printf("\nR1 or R3 detected black. Moving FORWARD until 2R is on black...\n");

            while (adc_read_channel(IR_2R) <= THRESHOLD) {
                small_step_forward();
                printf("\r2R=%.2fV    ", adc_read_channel(IR_2R));
                fflush(stdout);
            }
            break;
        }

        sleep_msec(50);
    }

    printf("\nAlignment complete!\n");
    stepper_disable();
    adc_destroy();
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
    
    adc_init();
    
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
    
    adc_destroy();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}

int main() {
    // Initialize the PYNQ library and stepper
    pynq_init();
    stepper_init();  // Single stepper initialization

    printf("IR Sensor Reading Program\n");
    printf("Commands:\n");
    printf("  i: Start reading IR sensors\n");
    printf("  t: Turn 180 degrees mode\n");
    printf("  q: Quit program\n");

    char cmd;
    while (1) {
        printf("\nEnter command: ");
        scanf(" %c", &cmd);

        switch (cmd) {
            case 'i':
                printf("Starting IR sensor readings (press 'q' to stop)...\n");
                read_ir_sensors();
                break;
            
            case 'f':
                follow_line();
                break;
            
            case 'd':
                printf("Starting detect line mode...\n");
                detect_line();
                break;
            
            case 'q':
                printf("Exiting program\n");
                stepper_destroy();  // Single stepper cleanup
                pynq_destroy();
                return 0;
            
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