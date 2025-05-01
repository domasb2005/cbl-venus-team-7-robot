#include <libpynq.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <switchbox.h>
#include <pinmap.h>

// Pin assignments
#define S0         IO_AR11
#define S1         IO_AR12
#define S2         IO_AR8
#define S3         IO_AR9
#define sensorOut  IO_AR7

// Get time in microseconds
uint64_t get_time_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000000 + tv.tv_usec;
}

// Sleep in milliseconds (provided by libpynq)
extern void sleep_msec(int);

// Reads LOW pulse width on sensorOut (microseconds)
int read_pulse_width() {
    uint64_t timeout = get_time_us() + 1000000; // 1s timeout

    while (gpio_get_level(sensorOut) == GPIO_LEVEL_HIGH) {
        if (get_time_us() > timeout) return -1;
    }
    while (gpio_get_level(sensorOut) == GPIO_LEVEL_LOW) {
        if (get_time_us() > timeout) return -1;
    }

    uint64_t start = get_time_us();
    while (gpio_get_level(sensorOut) == GPIO_LEVEL_LOW) {
        if (get_time_us() - start > 1000000) return -1;
    }

    uint64_t end = get_time_us();
    return (int)(end - start);
}

// Map function like Arduino's map()
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    if (in_max == in_min) return out_min; // prevent divide-by-zero
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Optional function: check if sensorOut is toggling
void debug_sensor_pin_levels() {
    printf("Sampling sensorOut pin levels (0 = LOW, 1 = HIGH):\n");
    for (int i = 0; i < 20; ++i) {
        gpio_level_t level = gpio_get_level(sensorOut);
        printf("%d ", level);
        sleep_msec(50);
    }
    printf("\n");
}

int main() {
    pynq_init();
    switchbox_init();

    // Set all pins to GPIO mode
    switchbox_set_pin(S0, SWB_GPIO);
    switchbox_set_pin(S1, SWB_GPIO);
    switchbox_set_pin(S2, SWB_GPIO);
    switchbox_set_pin(S3, SWB_GPIO);
    switchbox_set_pin(sensorOut, SWB_GPIO);

    // Set directions
    gpio_set_direction(S0, GPIO_DIR_OUTPUT);
    gpio_set_direction(S1, GPIO_DIR_OUTPUT);
    gpio_set_direction(S2, GPIO_DIR_OUTPUT);
    gpio_set_direction(S3, GPIO_DIR_OUTPUT);
    gpio_set_direction(sensorOut, GPIO_DIR_INPUT);

    // Set frequency scaling to 20%: S0 = HIGH, S1 = LOW
    gpio_set_level(S0, GPIO_LEVEL_HIGH);
    gpio_set_level(S1, GPIO_LEVEL_LOW);

    printf("Starting TCS3200 color sensing (debug mode)...\n");

    // Uncomment this to test if signal toggles at all:
    // debug_sensor_pin_levels();

    while (1) {
        int pulse;
        int r = 0, g = 0, b = 0;

        // --- RED ---
        gpio_set_level(S2, GPIO_LEVEL_LOW);
        gpio_set_level(S3, GPIO_LEVEL_LOW);
        sleep_msec(100);
        pulse = read_pulse_width();
        if (pulse < 0) {
            printf("RED pulse read failed\n");
        } else {
            printf("RED pulse width: %d us  ", pulse);
            r = map(pulse, 25, 72, 255, 0);
            if (r < 0) r = 0;
            if (r > 255) r = 255;
        }

        // --- GREEN ---
        gpio_set_level(S2, GPIO_LEVEL_HIGH);
        gpio_set_level(S3, GPIO_LEVEL_HIGH);
        sleep_msec(100);
        pulse = read_pulse_width();
        if (pulse < 0) {
            printf("GREEN pulse read failed\n");
        } else {
            printf("GREEN pulse width: %d us  ", pulse);
            g = map(pulse, 30, 90, 255, 0);
            if (g < 0) g = 0;
            if (g > 255) g = 255;
        }

        // --- BLUE ---
        gpio_set_level(S2, GPIO_LEVEL_LOW);
        gpio_set_level(S3, GPIO_LEVEL_HIGH);
        sleep_msec(100);
        pulse = read_pulse_width();
        if (pulse < 0) {
            printf("BLUE pulse read failed\n");
        } else {
            printf("BLUE pulse width: %d us\n", pulse);
            b = map(pulse, 25, 70, 255, 0);
            if (b < 0) b = 0;
            if (b > 255) b = 255;
        }

        printf("→ R = %3d   G = %3d   B = %3d\n", r, g, b);
        sleep_msec(500);
    }

    // Never reached
    switchbox_destroy();
    pynq_destroy();
    return 0;
}