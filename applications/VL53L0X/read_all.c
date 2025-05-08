#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

#define MUX_ADDRESS 0x70
#define TOF_ADDRESS 0x29  // Default VL53L0X address
#define NUM_CHANNELS 8    // TCA9548A has 8 channels (0-7)

// Function to select I2C multiplexer channel
static void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;
    uint8_t data = 1 << channel;
    iic_write_register(IIC0, MUX_ADDRESS, 0, &data, 1);
}

int read_all(void) {
    pynq_init();
    
    // Initialize I2C
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    vl53x sensors[NUM_CHANNELS];
    bool sensor_active[NUM_CHANNELS] = {false};
    int active_count = 0;

    // Initialize each sensor on each channel
    for (int i = 0; i < NUM_CHANNELS; i++) {
        selectMuxChannel(i);
        sleep_msec(10); // Give the multiplexer time to switch

        printf("\nChecking channel %d:\n", i);
        
        // Try to initialize the sensor on this channel
        if (tofPing(IIC0, TOF_ADDRESS) == 0) {
            printf("- Sensor found on channel %d\n", i);
            
            // Initialize the sensor
            if (tofInit(&sensors[i], IIC0, TOF_ADDRESS, 0) == 0) {
                sensor_active[i] = true;
                active_count++;
                
                // Get and print sensor info
                uint8_t model, revision;
                tofGetModel(&sensors[i], &model, &revision);
                printf("- Model ID: %d\n", model);
                printf("- Revision ID: %d\n", revision);
            } else {
                printf("- Failed to initialize sensor\n");
            }
        } else {
            printf("- No sensor detected\n");
        }
    }

    printf("\nFound %d active sensors\n", active_count);
    if (active_count == 0) {
        printf("No sensors found, exiting\n");
        iic_destroy(IIC0);
        pynq_destroy();
        return EXIT_FAILURE;
    }

    // Main reading loop
    printf("\nStarting continuous reading...\n");
    while(1) {
        printf("\nDistance readings:\n");
        for (int i = 0; i < NUM_CHANNELS; i++) {
            if (sensor_active[i]) {
                selectMuxChannel(i);
                sleep_msec(10); // Give the multiplexer time to switch
                
                uint32_t distance = tofReadDistance(&sensors[i]);
                printf("Channel %d: %dmm\n", i, distance);
            }
        }
        sleep_msec(500); // Update every 500ms
    }

    // Cleanup (this part won't be reached due to infinite loop)
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}