#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include "vl53l0x.h"
#include <stepper.h>
#include <iic.h>

#define SPEED 15000
#define STEPS 20000
#define TOF_ADDR 0x29  // Using default address for single sensor

void robot_move(){
    stepper_enable();
    stepper_set_speed(20000, 20000);
    stepper_steps(10000, 10000);
}

int main(void) {
    pynq_init();
    stepper_init();

    // Initialize I2C
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    // Verify sensor connectivity
    printf("Verifying sensor connectivity...\n");
    if (tofPing(IIC0, TOF_ADDR) != 0) {
        printf("Error: TOF sensor not responding\n");
        return -1;
    }
    printf("TOF sensor responding OK\n");

    // Create and initialize sensor struct
    vl53x sensor;
    if (tofInit(&sensor, IIC0, TOF_ADDR, 0) != 0) {
        printf("Failed to initialize sensor\n");
        return -1;
    }

    uint32_t iDistance;
    uint8_t model, revision;
    
    printf("VL53L0X device successfully opened.\n");
    tofGetModel(&sensor, &model, &revision);
    printf("Model ID - %d\n", model);
    printf("Revision ID - %d\n", revision);
    fflush(NULL);

    robot_move();
    
    for (int i = 0; i < 1200; i++) {
        iDistance = tofReadDistance(&sensor);
        printf("Distance = %dmm\n", iDistance);

        if (iDistance <= 100) {
            // stepper_disable();
            // stepper_destroy();
            stepper_reset();
            // stepper_init(); // Reinitialize stepper for new ste

            stepper_enable();
            stepper_set_speed(50000, 50000);
            stepper_steps(450, 450);
        }

        sleep_msec(100);
    }

    stepper_disable();
    stepper_destroy();
    pynq_destroy();
    return EXIT_SUCCESS;
}