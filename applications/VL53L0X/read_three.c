#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

// Define number of sensors to use (1-3)
#define NUM_SENSORS 2  // Change this value to 1, 2, or 3 for debugging

// Define the new addresses for sensors
#define TOF_DEFAULT_ADDR 0x29
#define TOF_ADDR1 0x30
#define TOF_ADDR2 0x31
#define TOF_ADDR3 0x32

// Define XSHUT pins
#define XSHUT1 IO_AR4
#define XSHUT2 IO_AR5
#define XSHUT3 IO_AR6

int read_three(void) {
    pynq_init();

    // Initialize I2C
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    // Initialize XSHUT pins as outputs (only for the number of sensors we're using)
    gpio_set_direction(XSHUT1, GPIO_DIR_OUTPUT);
    if (NUM_SENSORS >= 2) gpio_set_direction(XSHUT2, GPIO_DIR_OUTPUT);
    if (NUM_SENSORS == 3) gpio_set_direction(XSHUT3, GPIO_DIR_OUTPUT);


    gpio_set_direction(IO_AR8, GPIO_DIR_OUTPUT);
	gpio_set_level(IO_AR8, GPIO_LEVEL_LOW);


    // Initially set all active XSHUT pins LOW to disable sensors
    gpio_set_level(XSHUT1, GPIO_LEVEL_LOW);
    if (NUM_SENSORS >= 2) gpio_set_level(XSHUT2, GPIO_LEVEL_LOW);
    if (NUM_SENSORS == 3) gpio_set_level(XSHUT3, GPIO_LEVEL_LOW);
    sleep_msec(10);

    // Initialize first sensor
    gpio_set_level(XSHUT1, GPIO_LEVEL_HIGH);
    sleep_msec(10);
    if (tofSetAddress(IIC0, TOF_DEFAULT_ADDR, TOF_ADDR1) != 0) {
        if (tofPing(IIC0, TOF_ADDR1) == 0) {
            printf("Sensor 1 already configured with address 0x%02X\n", TOF_ADDR1);
        } else {
            printf("Failed to set address for sensor 1\n");
            return -1;
        }
    }

    // Initialize second sensor if needed
    if (NUM_SENSORS >= 2) {
        gpio_set_level(XSHUT2, GPIO_LEVEL_HIGH);
        sleep_msec(10);
        if (tofSetAddress(IIC0, TOF_DEFAULT_ADDR, TOF_ADDR2) != 0) {
            if (tofPing(IIC0, TOF_ADDR2) == 0) {
                printf("Sensor 2 already configured with address 0x%02X\n", TOF_ADDR2);
            } else {
                printf("Failed to set address for sensor 2\n");
                return -1;
            }
        }
    }

    // Initialize third sensor if needed
    if (NUM_SENSORS == 3) {
        gpio_set_level(XSHUT3, GPIO_LEVEL_HIGH);
        sleep_msec(10);
        if (tofSetAddress(IIC0, TOF_DEFAULT_ADDR, TOF_ADDR3) != 0) {
            if (tofPing(IIC0, TOF_ADDR3) == 0) {
                printf("Sensor 3 already configured with address 0x%02X\n", TOF_ADDR3);
            } else {
                printf("Failed to set address for sensor 3\n");
                return -1;
            }
        }
    }

    // Initialize sensor structures
    vl53x sensor1, sensor2, sensor3;
    
    // Initialize sensors based on NUM_SENSORS
    if (tofInit(&sensor1, IIC0, TOF_ADDR1, 0) != 0) {
        printf("Failed to initialize sensor 1\n");
        return -1;
    }
    
    if (NUM_SENSORS >= 2) {
        if (tofInit(&sensor2, IIC0, TOF_ADDR2, 0) != 0) {
            printf("Failed to initialize sensor 2\n");
            return -1;
        }
    }
    
    if (NUM_SENSORS == 3) {
        if (tofInit(&sensor3, IIC0, TOF_ADDR3, 0) != 0) {
            printf("Failed to initialize sensor 3\n");
            return -1;
        }
    }

    printf("All %d sensors initialized successfully\n", NUM_SENSORS);
	    // Verify all sensors are responding before starting readings
    printf("Verifying sensor connectivity...\n");
    
    if (tofPing(IIC0, TOF_ADDR1) != 0) {
        printf("Error: Sensor 1 (0x%02X) not responding\n", TOF_ADDR1);
        return -1;
    }
    printf("Sensor 1 (0x%02X) responding OK\n", TOF_ADDR1);
    
    if (NUM_SENSORS >= 2) {
        if (tofPing(IIC0, TOF_ADDR2) != 0) {
            printf("Error: Sensor 2 (0x%02X) not responding\n", TOF_ADDR2);
            return -1;
        }
        printf("Sensor 2 (0x%02X) responding OK\n", TOF_ADDR2);
    }
    
    if (NUM_SENSORS == 3) {
        if (tofPing(IIC0, TOF_ADDR3) != 0) {
            printf("Error: Sensor 3 (0x%02X) not responding\n", TOF_ADDR3);
            return -1;
        }
        printf("Sensor 3 (0x%02X) responding OK\n", TOF_ADDR3);
    }
    
    printf("All sensors verified successfully\n");

    // Main reading loop
    while(1) {
        uint32_t dist1 = tofReadDistance(&sensor1);
        printf("Sensor1: %dmm", dist1);
        
        if (NUM_SENSORS >= 2) {
            uint32_t dist2 = tofReadDistance(&sensor2);
            printf(", Sensor2: %dmm", dist2);
        }
        
        if (NUM_SENSORS == 3) {
            uint32_t dist3 = tofReadDistance(&sensor3);
            printf(", Sensor3: %dmm", dist3);
        }
        
        printf("\n");
        sleep_msec(500); // Wait 500ms between readings
    }

    // Cleanup (this won't be reached due to infinite loop)
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}
