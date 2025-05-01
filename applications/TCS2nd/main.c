#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <libpynq.h>
#include <time.h>

// --- Pin Definitions ---
#define PIN_S0 IO_AR11
#define PIN_S1 IO_AR12
#define PIN_S2 IO_AR8
#define PIN_S3 IO_AR9
#define PIN_OUT IO_AR7

#define FREQ_SCALE_2   0
#define FREQ_SCALE_20  1
#define FREQ_SCALE_100 2
#define FREQ_SCALE_OFF 3
int fs = FREQ_SCALE_2;

#define FILTER_RED    0
#define FILTER_BLUE   1
#define FILTER_CLEAR  2
#define FILTER_GREEN  3

double cal_r = 1.0, cal_g = 1.0, cal_b = 1.0;

// --- Function Prototypes ---
void setFrequencyScaling(int scale);
void setColorFilter(int color);
double readFrequency(int num_pulses);
long long timespec_diff_ns(struct timespec start, struct timespec end);
void calibrateSensor(void);
void runClassificationLoop(void);

int main(void) {
    pynq_init();
    gpio_init();

    gpio_set_direction(PIN_S0, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S1, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S2, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_S3, GPIO_DIR_OUTPUT);
    gpio_set_direction(PIN_OUT, GPIO_DIR_INPUT);

    setFrequencyScaling(fs);

    printf("Calibrate or run loop? (c/r): ");
    char choice = getchar();
    getchar(); // consume newline

    if (choice == 'c' || choice == 'C') {
        FILE *fout = fopen("color_calibration.csv", "w");
        if (!fout) {
            perror("Failed to open output file");
            return EXIT_FAILURE;
        }

        calibrateSensor();
        fprintf(fout, "Color,Sample,Raw_R,Raw_G,Raw_B,Norm_R,Norm_G,Norm_B\n");

        const char* colors[] = {"RED", "GREEN", "BLUE", "BLACK", "WHITE"};

        for (int c = 0; c < 5; ++c) {
            printf("\nPlace the sensor against the %s cube and press Enter...", colors[c]);
            getchar();

            for (int i = 0; i < 5; ++i) {
                setColorFilter(FILTER_RED); sleep_msec(200);
                double r = readFrequency(100);

                setColorFilter(FILTER_GREEN); sleep_msec(200);
                double g = readFrequency(100);

                setColorFilter(FILTER_BLUE); sleep_msec(200);
                double b = readFrequency(100);

                double nr = r / cal_r;
                double ng = g / cal_g;
                double nb = b / cal_b;

                printf("%s Sample %d: Raw: R=%.2f G=%.2f B=%.2f | Normalized: R=%.3f G=%.3f B=%.3f\n",
                       colors[c], i+1, r, g, b, nr, ng, nb);

                fprintf(fout, "%s,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
                        colors[c], i+1, r, g, b, nr, ng, nb);

                sleep(1);
            }
        }

        fclose(fout);
        printf("\nCalibration data saved to color_calibration.csv\n");
    } else {
        calibrateSensor();  // still needed before classification
        runClassificationLoop(); // enter detection loop
    }

    pynq_destroy();
    return 0;
}

// --- Classification Mode ---
void runClassificationLoop(void) {
    printf("\nEntering continuous classification mode...\n");

    while (1) {
        setColorFilter(FILTER_RED); sleep_msec(200);
        double r = readFrequency(100);
        setColorFilter(FILTER_GREEN); sleep_msec(200);
        double g = readFrequency(100);
        setColorFilter(FILTER_BLUE); sleep_msec(200);
        double b = readFrequency(100);

        double nr = r / cal_r;
        double ng = g / cal_g;
        double nb = b / cal_b;

        printf("→ R=%.3f G=%.3f B=%.3f  | ", nr, ng, nb);

        if (nr > 2 * ng && nr > 1.8 * nb) {
            printf("Detected RED\n");
        } else if (ng > 1.6 * nr && ng > 1.5 * nb) {
            printf("Detected GREEN\n");
        } else if (nb > 1.8 * nr && nb > 1.7 * ng) {
            printf("Detected BLUE\n");
        } else if (nr < 0.1 && ng < 0.1 && nb < 0.1) {
            printf("Detected BLACK\n");
        } else if (nr > 0.9 && ng > 0.9 && nb > 0.9) {
            printf("Detected WHITE\n");
        } else {
            printf("Color UNDETERMINED\n");
        }

        sleep(1);
    }
}

// --- Support Functions ---
void setFrequencyScaling(int scale) {
    switch (scale) {
        case FREQ_SCALE_2:
            gpio_set_level(PIN_S0, GPIO_LEVEL_LOW);
            gpio_set_level(PIN_S1, GPIO_LEVEL_HIGH); break;
        case FREQ_SCALE_20:
            gpio_set_level(PIN_S0, GPIO_LEVEL_HIGH);
            gpio_set_level(PIN_S1, GPIO_LEVEL_LOW); break;
        case FREQ_SCALE_100:
            gpio_set_level(PIN_S0, GPIO_LEVEL_HIGH);
            gpio_set_level(PIN_S1, GPIO_LEVEL_HIGH); break;
        default:
            gpio_set_level(PIN_S0, GPIO_LEVEL_LOW);
            gpio_set_level(PIN_S1, GPIO_LEVEL_LOW); break;
    }
}

void setColorFilter(int color) {
    switch (color) {
        case FILTER_RED:
            gpio_set_level(PIN_S2, GPIO_LEVEL_LOW);
            gpio_set_level(PIN_S3, GPIO_LEVEL_LOW); break;
        case FILTER_BLUE:
            gpio_set_level(PIN_S2, GPIO_LEVEL_LOW);
            gpio_set_level(PIN_S3, GPIO_LEVEL_HIGH); break;
        case FILTER_CLEAR:
            gpio_set_level(PIN_S2, GPIO_LEVEL_HIGH);
            gpio_set_level(PIN_S3, GPIO_LEVEL_LOW); break;
        case FILTER_GREEN:
        default:
            gpio_set_level(PIN_S2, GPIO_LEVEL_HIGH);
            gpio_set_level(PIN_S3, GPIO_LEVEL_HIGH); break;
    }
}

long long timespec_diff_ns(struct timespec start, struct timespec end) {
    return (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
}

void calibrateSensor(void) {
    const int num = 5;
    double temp_r = 0, temp_g = 0, temp_b = 0;

    printf("\n--- White Calibration ---\n");
    printf("Place white object close to sensor and press Enter...");
    getchar();

    for (int i = 0; i < num; ++i) {
        setColorFilter(FILTER_RED); sleep_msec(50); temp_r += readFrequency(100);
        setColorFilter(FILTER_GREEN); sleep_msec(50); temp_g += readFrequency(100);
        setColorFilter(FILTER_BLUE); sleep_msec(50); temp_b += readFrequency(100);
        printf(".");
        fflush(stdout);
        sleep_msec(100);
    }
    printf("\n");

    cal_r = temp_r / num;
    cal_g = temp_g / num;
    cal_b = temp_b / num;

    if (cal_r <= 0) cal_r = 1.0;
    if (cal_g <= 0) cal_g = 1.0;
    if (cal_b <= 0) cal_b = 1.0;

    printf("Calibration done: R=%.2f G=%.2f B=%.2f\n\n", cal_r, cal_g, cal_b);
    sleep(1);
}

double readFrequency(int num_pulses) {
    if (num_pulses <= 0) return 0.0;

    struct timespec start, end;
    int count = 0;
    gpio_level_t current, last = gpio_get_level(PIN_OUT);

    while (1) {
        current = gpio_get_level(PIN_OUT);
        if (current == GPIO_LEVEL_HIGH && last == GPIO_LEVEL_LOW) {
            clock_gettime(CLOCK_MONOTONIC, &start);
            break;
        }
        last = current;
    }

    while (count < num_pulses) {
        current = gpio_get_level(PIN_OUT);
        if (current == GPIO_LEVEL_HIGH && last == GPIO_LEVEL_LOW)
            count++;
        last = current;
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    long long ns = timespec_diff_ns(start, end);
    return (ns > 0) ? ((double)num_pulses * 1e9 / ns) : 0.0;
}
