#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <stdint.h>
#include <libpynq.h>
#include <stepper.h>
#include <uart.h>
#include <switchbox.h>
#include <mosquitto.h>
#include <cjson/cJSON.h>

#define BROKER_IP "100.104.111.51"
#define BROKER_PORT 1883
#define TOPIC "robot/A"
#define left_threshold 0.3
#define right_threshold 0.2
#define tape_backoff_steps 100 // Steps to back off when tape is detected


#define IR_2R ADC0
#define IR_3L ADC3  // Closest to center - left (a1)
#define IR_3R ADC2  // Furthest from center - left (a2)
#define IR_1R ADC1  // Furthest from center - right (a3) -line follower
#define IR_1L ADC4  // Closest to center - right (a4) -line follower
#define IR_2L ADC5  // Middle - right (a5) - line follower


#define M_PI 3.14159265358979323846

volatile double current_x = 0.0;
volatile double current_y = 0.0;
volatile double heading_deg = 0.0;

pthread_mutex_t position_mutex = PTHREAD_MUTEX_INITIALIZER;

void normalize_heading() {
    while (heading_deg >= 360.0) heading_deg -= 360.0;
    while (heading_deg < 0.0) heading_deg += 360.0;
}

void update_position_from_steps(int16_t left_steps, int16_t right_steps, char movement_type) {
    pthread_mutex_lock(&position_mutex);
    
    // Calculate the average magnitude of steps
    int16_t avg_steps_magnitude = (abs(left_steps) + abs(right_steps)) / 2;
    
    // Direction is determined by the movement_type, not by the sign of steps
    double distance = (movement_type == 'f') ? avg_steps_magnitude : -avg_steps_magnitude;
    
    // Convert heading to radians
    double rad = heading_deg * (M_PI / 180.0);
    
    if (movement_type == 'f' || movement_type == 'b') {
        // With 0 degrees:
        // - Forward/backward should affect Y
        // - Left/right rotation to 90 degrees then forward should affect X
        
        // At 0 degrees heading:
        // - cos(0) = 1, sin(0) = 0 → movement along Y axis
        // At 90 degrees heading:
        // - cos(90°) = 0, sin(90°) = 1 → movement along X axis
        
        current_y += cos(rad) * distance;  // Y increases when heading = 0
        current_x += sin(rad) * distance;  // X increases when heading = 90
        
        printf("Distance calculation: type=%c, avg_magnitude=%d, final_distance=%.2f, heading=%.2f\n", 
               movement_type, avg_steps_magnitude, distance, heading_deg);
    } else if (movement_type == 'l' || movement_type == 'r') {
        // Rotation handling - unchanged
        double step_diff = right_steps - left_steps;
        double heading_change = step_diff * 0.072;
        heading_deg += heading_change;
        normalize_heading();
    }
    pthread_mutex_unlock(&position_mutex);
}

void handle_command(struct mosquitto* mosq, const char* cmd_str, int speed, int steps) {
    int16_t left = 0, right = 0;
    char type = 'f';

    if (strcmp(cmd_str, "forward") == 0) {
        stepper_enable();
        left = steps;
        right = steps;
        type = 'f';
    } else if (strcmp(cmd_str, "backward") == 0) {
        stepper_enable();
        left = -steps;
        right = -steps;
        type = 'b';
    } else if (strcmp(cmd_str, "rotate_left") == 0) {
        stepper_enable();
        left = steps;
        right = -steps;
        type = 'l';
    } else if (strcmp(cmd_str, "rotate_right") == 0) {
        stepper_enable();
        left = -steps;
        right = steps;
        type = 'r';
    } else if (strcmp(cmd_str, "stop") == 0) {
        stepper_reset();
        stepper_disable();
        cJSON *completed_msg = cJSON_CreateObject();
        cJSON_AddStringToObject(completed_msg, "status", "completed");
        char *completed_str = cJSON_PrintUnformatted(completed_msg);
        mosquitto_publish(mosq, NULL, TOPIC, strlen(completed_str), completed_str, 0, false);
        free(completed_str);
        cJSON_Delete(completed_msg);
        return;
    } else if (strcmp(cmd_str, "reset") == 0) {
        // Reset position and heading to zero
        pthread_mutex_lock(&position_mutex);
        current_x = 0.0;
        current_y = 0.0;
        heading_deg = 0.0;
        pthread_mutex_unlock(&position_mutex);
        
        printf("Position and heading reset to (0.0, 0.0, 0.0)\n");
        
        // Publish completion message
        cJSON *completed_msg = cJSON_CreateObject();
        cJSON_AddStringToObject(completed_msg, "status", "completed");
        cJSON_AddStringToObject(completed_msg, "action", "reset");
        char *completed_str = cJSON_PrintUnformatted(completed_msg);
        mosquitto_publish(mosq, NULL, TOPIC, strlen(completed_str), completed_str, 0, false);
        free(completed_str);
        cJSON_Delete(completed_msg);
        return;
    } else if (strcmp(cmd_str, "ping") == 0) {
        cJSON *pong = cJSON_CreateObject();
        cJSON_AddStringToObject(pong, "cmd", "pong");
        char *response = cJSON_PrintUnformatted(pong);
        mosquitto_publish(mosq, NULL, TOPIC, strlen(response), response, 0, false);
        free(response);
        cJSON_Delete(pong);
        return;
    } else {
        printf("Unknown command: %s\n", cmd_str);
        return;
    }

    stepper_set_speed(speed, speed);
    stepper_steps(left, right);
    while(!stepper_steps_done()) {
        double ir_1l = adc_read_channel(IR_1L);
        double ir_1r = adc_read_channel(IR_1R);
        double ir_2l = adc_read_channel(IR_2L);
        double ir_2r = adc_read_channel(IR_2R);
        double ir_3l = adc_read_channel(IR_3L);
        double ir_3r = adc_read_channel(IR_3R);
        if (ir_1l > left_threshold || ir_1r > right_threshold ||
            ir_2l > left_threshold || ir_2r > right_threshold ||
            ir_3l > left_threshold || ir_3r > right_threshold) {
            printf("Tape detected! Stopping movement.\n");
            printf("%s | IR values: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                   cmd_str, ir_1l, ir_1r, ir_2l, ir_2r, ir_3l, ir_3r);
            stepper_get_completed_steps(&left, &right);
            printf("Completed steps: left=%d, right=%d\n", left, right);
            stepper_reset();
	sleep_msec(1000);
            stepper_enable();
            stepper_set_speed(speed, speed);
            stepper_steps(-tape_backoff_steps, -tape_backoff_steps); // Reverse to avoid overshoot
            printf("Reversing from tape...\n");
            while(!stepper_steps_done()) {
                sleep_msec(1); // Wait for the stepper to finish reversing
            }
            left -= tape_backoff_steps;
            right -= tape_backoff_steps;
            break;
        }
        sleep_msec(1); // Wait for the stepper to finish to do later now ignore
    }
                //publish mqtt message "completed""
    cJSON *completed_msg = cJSON_CreateObject();
    cJSON_AddStringToObject(completed_msg, "status", "completed");
    cJSON_AddNumberToObject(completed_msg, "left", left);
    cJSON_AddNumberToObject(completed_msg, "right", right);
    char *completed_str = cJSON_PrintUnformatted(completed_msg);
    mosquitto_publish(mosq, NULL, TOPIC, strlen(completed_str), completed_str, 0, false);
    free(completed_str);
    cJSON_Delete(completed_msg);
    update_position_from_steps(left, right, type);

    printf("Executed %s | Position: (%.2f, %.2f), Heading: %.2f\n",
           cmd_str, current_x, current_y, heading_deg);
}

void message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg) {
    (void)userdata;

    if (msg == NULL || msg->payloadlen == 0) return;

    cJSON *root = cJSON_Parse((char*)msg->payload);
    if (!root) return;

    // Only handle messages that contain a cmd
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (!cmd || !cJSON_IsString(cmd)) {
        cJSON_Delete(root); // Ignore telemetry
        return;
    }

    const char* cmd_str = cmd->valuestring;

    int speed = 0, steps = 0;
    cJSON *spd = cJSON_GetObjectItem(root, "speed");
    cJSON *stp = cJSON_GetObjectItem(root, "steps");

    if ((spd && cJSON_IsNumber(spd)) && (stp && cJSON_IsNumber(stp))) {
        speed = spd->valueint;
        steps = stp->valueint;
    }

    handle_command(mosq, cmd_str, speed, steps);
    cJSON_Delete(root);
}

void* mqtt_listener_thread(void* arg) {
    (void)arg;

    struct mosquitto *mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Failed to create mosquitto instance\n");
        return NULL;
    }

    if (mosquitto_connect(mosq, BROKER_IP, BROKER_PORT, 60)) {
        fprintf(stderr, "Unable to connect to MQTT broker\n");
        return NULL;
    }

    mosquitto_subscribe(mosq, NULL, TOPIC, 0);
    mosquitto_message_callback_set(mosq, message_callback);

    printf("Connected to MQTT broker at %s:%d\n", BROKER_IP, BROKER_PORT);
    mosquitto_loop_forever(mosq, -1, 1);
    mosquitto_destroy(mosq);
    return NULL;
}

void* telemetry_thread(void* arg) {
    (void)arg;

    switchbox_init();
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    uart_init(UART0);
    uart_reset_fifos(UART0);

    struct mosquitto *mosq = mosquitto_new(NULL, true, NULL);
    mosquitto_connect(mosq, BROKER_IP, BROKER_PORT, 60);

    char msg[512];
    while (1) {
        pthread_mutex_lock(&position_mutex);
        double x = current_x, y = current_y, h = heading_deg;
        pthread_mutex_unlock(&position_mutex);

        // Read IR values
        double ir_1l = adc_read_channel(IR_1L);
        double ir_1r = adc_read_channel(IR_1R);
        double ir_2l = adc_read_channel(IR_2L);
        double ir_2r = adc_read_channel(IR_2R);
        double ir_3l = adc_read_channel(IR_3L);
        double ir_3r = adc_read_channel(IR_3R);

        // TOF placeholders
        int ttof_l = -1;
        int ttof_r = -1;
        int btof = -1;

        snprintf(msg, sizeof(msg),
            "{\"x\":%.2f,\"y\":%.2f,\"heading\":%.2f,"
            "\"IR_1L\":%.2f,\"IR_1R\":%.2f,"
            "\"IR_2L\":%.2f,\"IR_2R\":%.2f,"
            "\"IR_3L\":%.2f,\"IR_3R\":%.2f,"
            "\"tTOF_L\":%d,\"tTOF_R\":%d,\"bTOF\":%d}",
            x, y, h,
            ir_1l, ir_1r,
            ir_2l, ir_2r,
            ir_3l, ir_3r,
            ttof_l, ttof_r, btof
        );

        // Publish to MQTT
        mosquitto_publish(mosq, NULL, TOPIC, strlen(msg), msg, 0, false);

        // Also send over UART
        uint32_t len = strlen(msg);
        uint8_t* len_bytes = (uint8_t*)&len;
        for (int i = 0; i < 4; i++) uart_send(UART0, len_bytes[i]);
        for (uint32_t i = 0; i < len; i++) uart_send(UART0, msg[i]);

        sleep_msec(100);
    }

    uart_destroy(UART0);
    mosquitto_destroy(mosq);
    return NULL;
}


int main() {
    pynq_init();
    stepper_init();
    stepper_enable();
    mosquitto_lib_init();
    adc_init();

    pthread_t mqtt_thread, telemetry_thread_id;
    pthread_create(&mqtt_thread, NULL, mqtt_listener_thread, NULL);
    pthread_create(&telemetry_thread_id, NULL, telemetry_thread, NULL);

    printf("Robot control system initialized. Listening for commands.\n");

    pthread_join(mqtt_thread, NULL);
    pthread_join(telemetry_thread_id, NULL);

    stepper_destroy();
    pynq_destroy();
    return 0;
}
