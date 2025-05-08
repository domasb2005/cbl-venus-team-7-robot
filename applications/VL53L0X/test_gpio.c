#include <libpynq.h>
#include <stdio.h>

// Define XSHUT pins - using the same pins as in read_three.c
#define XSHUT1 IO_AR4
#define XSHUT2 IO_AR5
#define XSHUT3 IO_AR6

int test_gpio(void) {
    // Initialize PYNQ
    pynq_init();

    // Initialize XSHUT pins as outputs
    gpio_set_direction(XSHUT1, GPIO_DIR_OUTPUT);
    gpio_set_direction(XSHUT2, GPIO_DIR_OUTPUT);
    gpio_set_direction(XSHUT3, GPIO_DIR_OUTPUT);

    // Set all XSHUT pins HIGH
    gpio_set_level(XSHUT1, GPIO_LEVEL_HIGH);
    gpio_set_level(XSHUT2, GPIO_LEVEL_LOW);
    gpio_set_level(XSHUT3, GPIO_LEVEL_HIGH);

    printf("All XSHUT pins set to HIGH. Program will run indefinitely.\n");
    printf("Press Ctrl+C to exit.\n");

    // Infinite loop to keep pins HIGH
    while(1) {
        sleep_msec(1000);  // Sleep to prevent CPU hogging
    }

    // This will never be reached due to infinite loop
    pynq_destroy();
    return EXIT_SUCCESS;
}