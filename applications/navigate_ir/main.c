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

volatile double current_x = 0.0;
volatile double current_y = 0.0;
volatile double heading_deg = 0.0;  // in degrees
pthread_mutex_t position_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DEG_PER_STEP_DIFF (90.0 / 1230.0)  // degrees per diff step
#define SMALL_TURN_DIFF 10.0              // from stepper_steps(5, -5)
#define SMALL_STEP_DEGREES (DEG_PER_STEP_DIFF * SMALL_TURN_DIFF)

#define FORWARD_STEP_SIZE 30.0  // same as stepper_steps(30, 30)

// IR sensor pin definitions (ADC channels)
#define IR_2L ADC0  // Middle - left (a0)
#define IR_1L ADC3  // Closest to center - left (a1)
#define IR_3L ADC2  // Furthest from center - left (a2)
#define IR_3R ADC1  // Furthest from center - right (a3) -line follower
#define IR_1R ADC4  // Closest to center - right (a4) -line follower
#define IR_2R ADC5  // Middle - right (a5) - line follower

// Add this at the top with other includes
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void* uart_position_publisher(void* arg) {
    (void)arg;  // Explicitly mark arg as unused to silence warning
    // Setup UART
    switchbox_init();
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    uart_init(UART0);
    uart_reset_fifos(UART0);

    // Setup MQTT
    struct mosquitto *mosq = NULL;
    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Error: Out of memory.\n");
        return NULL;
    }

    if (mosquitto_connect(mosq, "10.35.80.115", 1883, 60)) {
        fprintf(stderr, "Unable to connect to MQTT broker.\n");
        return NULL;
    }

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


void right_90(void) {
    stepper_steps(-615, 615);
}

#define THRESHOLD 0.19
#define LEFT_THRESHOLD 0.3

void normalize_heading() {
    while (heading_deg >= 360.0) heading_deg -= 360.0;
    while (heading_deg < 0.0) heading_deg += 360.0;
}

void small_step_left() {
    stepper_enable();
    stepper_set_speed(15000, 15000);
    stepper_steps(5, -5);
    sleep_msec(50);

    pthread_mutex_lock(&position_mutex);
    heading_deg += SMALL_STEP_DEGREES;
    normalize_heading();
    pthread_mutex_unlock(&position_mutex);
}


void small_step_forward() {
    stepper_enable();
    stepper_set_speed(30000, 30000);
    stepper_steps(FORWARD_STEP_SIZE, FORWARD_STEP_SIZE);
    sleep_msec(10);

    pthread_mutex_lock(&position_mutex);
    double rad = heading_deg * (M_PI / 180.0);  // Fixed M_PI usage
    current_x += cos(rad) * FORWARD_STEP_SIZE;
    current_y += sin(rad) * FORWARD_STEP_SIZE;
    pthread_mutex_unlock(&position_mutex);
}


void smaller_step_forward() {
    stepper_enable();
    stepper_set_speed(30000, 30000);
    stepper_steps(10, 10);
    sleep_msec(10);

    pthread_mutex_lock(&position_mutex);
    double rad = heading_deg * (M_PI / 180.0);  // Fixed M_PI usage
    current_x += cos(rad) * 10.0;
    current_y += sin(rad) * 10.0;
    pthread_mutex_unlock(&position_mutex);
}


void small_step_right() {
    stepper_enable();
    stepper_set_speed(15000, 15000);
    stepper_steps(-5, 5);
    sleep_msec(50);

    pthread_mutex_lock(&position_mutex);
    heading_deg -= SMALL_STEP_DEGREES;
    normalize_heading();
    pthread_mutex_unlock(&position_mutex);
}




void avoid_crater() {
    printf("\n[!] Crater detected — initiating avoidance maneuver...\n");

    stepper_reset();
    stepper_enable();

    // 1. Go 300 steps backward
    printf("[←] Reversing 300 steps...\n");
    stepper_set_speed(20000, 20000);
    stepper_steps(-300, -300);
    pthread_mutex_lock(&position_mutex);
double rad = heading_deg * M_PI / 180.0;
current_x -= cos(rad) * 300.0;
current_y -= sin(rad) * 300.0;
pthread_mutex_unlock(&position_mutex);
    sleep_msec(1000);

    // 2. Turn 90 degrees LEFT
    printf("[↺] Turning 90 degrees left...\n");
    stepper_steps(615, -615);
    pthread_mutex_lock(&position_mutex);
    heading_deg += 90.0;
    normalize_heading();
    pthread_mutex_unlock(&position_mutex);
    sleep_msec(1000);

    // 3. Move forward using small_step_forward() up to ~1300 steps total
    int total_steps = 0;
    printf("[↑] Moving forward incrementally (up to 1300 steps)...\n");

    while (total_steps < 1300) {
        double v_r1 = adc_read_channel(IR_1R);
        double v_r2 = adc_read_channel(IR_2R);
        double v_r3 = adc_read_channel(IR_3R);
        double v_l1 = adc_read_channel(IR_1L);
        double v_l2 = adc_read_channel(IR_2L);
        double v_l3 = adc_read_channel(IR_3L);

        // If right IR sees black → stop and return (line found)
        if (v_r1 > THRESHOLD || v_r2 > THRESHOLD || v_r3 > THRESHOLD) {
            printf("[✓] Right IR sees black — line reacquired. Exiting crater avoidance.\n");
            return;
        }

        // If left IR sees black → restart crater avoidance
        if (v_l1 > LEFT_THRESHOLD || v_l2 > LEFT_THRESHOLD || v_l3 > LEFT_THRESHOLD) {
            printf("[↺] Left IR sees black during forward — restarting crater avoidance...\n");
            avoid_crater();  // recursive call
            return;          // terminate this instance so old one doesn't continue
        }

        small_step_forward();
        total_steps += 40;  // each small_step_forward does 10 steps
        sleep_msec(100);
    }

    // 4. Turn 90 degrees RIGHT
    printf("[↻] Turning 90 degrees right...\n");
    stepper_steps(-615, 615);
    pthread_mutex_lock(&position_mutex);
    heading_deg -= 90.0;
    normalize_heading();
    pthread_mutex_unlock(&position_mutex);
    sleep_msec(1000);

    // 5. small_step_forward until any of R1, R2, R3 are ON
    printf("[→] Searching for line (right IRs)...\n");

    while (1) {
        double v_r1 = adc_read_channel(IR_1R);
        double v_r2 = adc_read_channel(IR_2R);
        double v_r3 = adc_read_channel(IR_3R);

        printf("Right IRs: R1=%.3f R2=%.3f R3=%.3f\n", v_r1, v_r2, v_r3);

        if (v_r1 > THRESHOLD || v_r2 > THRESHOLD || v_r3 > THRESHOLD) {
            printf("[✓] Line reacquired by right IR. Exiting crater avoidance.\n");
            break;
        }

        small_step_forward();
        sleep_msec(50);
    }
}


void follow_line() {
    printf("Starting follow_line mode (raw + cross-detection + recovery)...\n");

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
        // --- Sensor reads ---
        double v_l1 = adc_read_channel(IR_1L);
        double v_l2 = adc_read_channel(IR_2L);
        double v_l3 = adc_read_channel(IR_3L);
        double v_r1 = adc_read_channel(IR_1R);
        double v_r2 = adc_read_channel(IR_2R);
        double v_r3 = adc_read_channel(IR_3R);

        printf("Left IR values: v_l1=%.3f, v_l2=%.3f, v_l3=%.3f\n", v_l1, v_l2, v_l3);

        int left_on_black = (v_l1 > LEFT_THRESHOLD) || (v_l2 > LEFT_THRESHOLD) || (v_l3 > LEFT_THRESHOLD);
        int right_on_black = (v_r1 > THRESHOLD) || (v_r2 > THRESHOLD) || (v_r3 > THRESHOLD);
        int all_ir_off = !left_on_black && !right_on_black;

        // --- [1] CROSSING DETECTION (highest priority) ---
        if (left_on_black) {
            printf("\n[!] Crossing detected via left IR. Entering correction...\n");

            avoid_crater();
            printf("[✓] Crossing recovery complete. Resuming tracking...\n");
            continue;
        }

        // --- [3] RECOVERY IF NO RIGHT IR IS ON BLACK ---
        if (all_ir_off || !right_on_black) {
            printf("\n[!] No IRs or no right IR on black. Stepping forward until at least one right IR sees black...\n");

            int recovery_running = 1;
            while (recovery_running) {
                v_r1 = adc_read_channel(IR_1R);
                v_r2 = adc_read_channel(IR_2R);
                v_r3 = adc_read_channel(IR_3R);
                v_l1 = adc_read_channel(IR_1L);
                v_l2 = adc_read_channel(IR_2L);
                v_l3 = adc_read_channel(IR_3L);

                printf("Right IRs: R1=%.3f R2=%.3f R3=%.3f | Left IRs: L1=%.3f L2=%.3f L3=%.3f\n",
                       v_r1, v_r2, v_r3, v_l1, v_l2, v_l3);

                // Mid-recovery crossing detection
                if ((v_l1 > LEFT_THRESHOLD) || (v_l2 > LEFT_THRESHOLD) || (v_l3 > LEFT_THRESHOLD)) {
                    printf("[!] Crossing detected mid-recovery! Switching to crossing logic.\n");

                    avoid_crater();
                    printf("[✓] Mid-recovery crossing recovery complete. Resuming tracking...\n");
                    break;
                }

                if (v_r1 > THRESHOLD || v_r2 > THRESHOLD || v_r3 > THRESHOLD) {
                    printf("[✓] Right IR now sees black. Resuming normal follow...\n");
                    break;
                }

                small_step_forward();
                // sleep_msec(50);

                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(STDIN_FILENO, &readfds);
                struct timeval timeout = {0, 0};
                if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
                    char c;
                    if (read(STDIN_FILENO, &c, 1) > 0 && c == 'q') {
                        printf("\nExiting follow_line during recovery.\n");
                        running = 0;
                        recovery_running = 0;
                        break;
                    }
                }
            }
            continue;
        }

        // --- [4] RAW SENSOR FOLLOWING ---
        uint32_t raw_r1 = adc_read_channel_raw(IR_1R);
        uint32_t raw_r2 = adc_read_channel_raw(IR_2R);
        uint32_t raw_r3 = adc_read_channel_raw(IR_3R);

        printf("\rRaw: R1=%4u R2=%4u R3=%4u | L1=%.2f L2=%.2f L3=%.2f    ",
               raw_r1, raw_r2, raw_r3, v_l1, v_l2, v_l3);
        fflush(stdout);

        if (raw_r2 > raw_r1 && raw_r2 > raw_r3) {
            small_step_forward();
        } else if (raw_r3 > raw_r2 && raw_r3 > raw_r1) {
            while (!(adc_read_channel_raw(IR_2R) > adc_read_channel_raw(IR_1R) &&
                     adc_read_channel_raw(IR_2R) > adc_read_channel_raw(IR_3R)) && running) {
                // Check for crossing before making the turn
                double v_l1 = adc_read_channel(IR_1L);
                double v_l2 = adc_read_channel(IR_2L);
                double v_l3 = adc_read_channel(IR_3L);
                
                if (v_l1 > LEFT_THRESHOLD || v_l2 > LEFT_THRESHOLD || v_l3 > LEFT_THRESHOLD) {
                    printf("[!] Crossing detected during right turn. Switching to crater avoidance...\n");
                    avoid_crater();
                    break;
                }
                
                smaller_step_forward();
                small_step_right();
            }
        } else if (raw_r1 > raw_r2 && raw_r1 > raw_r3) {
            while (!(adc_read_channel_raw(IR_2R) > adc_read_channel_raw(IR_1R) &&
                     adc_read_channel_raw(IR_2R) > adc_read_channel_raw(IR_3R)) && running) {
                // Check for crossing before making the turn
                double v_l1 = adc_read_channel(IR_1L);
                double v_l2 = adc_read_channel(IR_2L);
                double v_l3 = adc_read_channel(IR_3L);
                
                if (v_l1 > LEFT_THRESHOLD || v_l2 > LEFT_THRESHOLD || v_l3 > LEFT_THRESHOLD) {
                    printf("[!] Crossing detected during left turn. Switching to crater avoidance...\n");
                    avoid_crater();
                    break;
                }
                
                small_step_left();
            }
        } else {
            sleep_msec(50);
        }

        // Exit key check
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
    // stepper_set_speed(20000, 20000);
    // stepper_steps(20000, 20000);  // Start moving forward

    int running = 1;
    while (running) {
        // Read all sensors

        small_step_forward();
        double ir_1l = adc_read_channel(IR_1L);
        double ir_2l = adc_read_channel(IR_2L);
        double ir_3l = adc_read_channel(IR_3L);
        double ir_1r = adc_read_channel(IR_1R);
        double ir_2r = adc_read_channel(IR_2R);
        double ir_3r = adc_read_channel(IR_3R);

        printf("\rIRs: 1L=%.2f 2L=%.2f 3L=%.2f | 1R=%.2f 2R=%.2f 3R=%.2f    ",
               ir_1l, ir_2l, ir_3l, ir_1r, ir_2r, ir_3r);
        fflush(stdout);

        // ✅ New goal: any right IR sensor detects black
        if (ir_1r > THRESHOLD || ir_2r > THRESHOLD || ir_3r > THRESHOLD) {
            printf("\n[✓] One or more right IR sensors are on black. Alignment complete.\n");
            break;
        }

        // If any left-side sensors are on black → turn left until ANY right IR is on black
        if (ir_1l > LEFT_THRESHOLD || ir_2l > LEFT_THRESHOLD || ir_3l > LEFT_THRESHOLD) {
            stepper_reset();
            printf("\n[↺] Left sensors detected black. Turning LEFT until right IRs see black...\n");

            while (!(adc_read_channel(IR_1R) > THRESHOLD ||
                     adc_read_channel(IR_2R) > THRESHOLD ||
                     adc_read_channel(IR_3R) > THRESHOLD)) {
                small_step_left();
                printf("\rRight IRs: R1=%.2f R2=%.2f R3=%.2f    ",
                       adc_read_channel(IR_1R),
                       adc_read_channel(IR_2R),
                       adc_read_channel(IR_3R));
                fflush(stdout);
            }
            break;
        }

        // If R1 or R3 detect black → move forward until any right IR is on black
        if (ir_1r > THRESHOLD || ir_3r > THRESHOLD) {
            stepper_reset();
            printf("\n[↑] R1 or R3 detected black. Moving FORWARD until right IRs see black...\n");

            while (!(adc_read_channel(IR_1R) > THRESHOLD ||
                     adc_read_channel(IR_2R) > THRESHOLD ||
                     adc_read_channel(IR_3R) > THRESHOLD)) {
                small_step_forward();
                printf("\rRight IRs: R1=%.2f R2=%.2f R3=%.2f    ",
                       adc_read_channel(IR_1R),
                       adc_read_channel(IR_2R),
                       adc_read_channel(IR_3R));
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

void manual_mode() {
    printf("Starting manual mode...\n");
    printf("Controls:\n");
    printf("  W: Forward 100 steps\n");
    printf("  S: Backward 100 steps\n");
    printf("  A: Turn left 100 steps\n");
    printf("  D: Turn right 100 steps\n");
    printf("  Q: Exit manual mode\n");

    const int manual_distance = 100;
    
    struct termios old_settings, new_settings;
    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    stepper_reset();
    stepper_enable();
    stepper_set_speed(20000, 20000);

    int running = 1;
    while (running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval timeout = {0, 0};
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                switch(c) {
                    case 'w':
                        printf("\r[↑] Moving forward %d steps...\n", manual_distance);
                        stepper_steps(manual_distance, manual_distance);
                        break;
                    case 's':
                        printf("\r[↓] Moving backward %d steps...\n", manual_distance);
                        stepper_steps(-manual_distance, -manual_distance);
                        break;
                    case 'a':
                        printf("\r[←] Turning left %d steps...\n", manual_distance);
                        stepper_steps(manual_distance, -manual_distance);
                        break;
                    case 'd':
                        printf("\r[→] Turning right %d steps...\n", manual_distance);
                        stepper_steps(-manual_distance, manual_distance);
                        break;
                    case 'q':
                        printf("\nExiting manual mode\n");
                        running = 0;
                        break;
                }
            }
        }
        sleep_msec(50);  // Small delay to prevent CPU hogging
    }

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
            
            case 'm':
                manual_mode();
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