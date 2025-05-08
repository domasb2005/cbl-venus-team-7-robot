#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stepper.h>

#define MIN_SPEED 60000
#define MAX_SPEED 10000
#define NUM_INCREMENTS 20

int sweep_distance = 2000; // Total number of steps to move during accel/decel

void sweep(int distance) {
    int speed;
    int step_chunk = distance / NUM_INCREMENTS;

    // Accelerate: MIN_SPEED -> MAX_SPEED
    for (int i = 0; i < NUM_INCREMENTS; i++) {
        speed = MIN_SPEED - ((MIN_SPEED - MAX_SPEED) * i) / NUM_INCREMENTS;
        stepper_set_speed(speed, speed);
        stepper_steps(step_chunk, step_chunk);
    }

    // Decelerate: MAX_SPEED -> MIN_SPEED
    for (int i = 0; i < NUM_INCREMENTS; i++) {
        speed = MAX_SPEED + ((MIN_SPEED - MAX_SPEED) * i) / NUM_INCREMENTS;
        stepper_set_speed(speed, speed);
        stepper_steps(step_chunk, step_chunk);
    }
}

int main(void) {
    pynq_init();
    stepper_init();
    stepper_enable();

    while (1) {
        sweep(sweep_distance);
        // Optional: reverse direction
        sweep(-sweep_distance);
    }

    stepper_disable();
    stepper_destroy();
    pynq_destroy();
    return EXIT_SUCCESS;
}
