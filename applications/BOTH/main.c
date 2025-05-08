#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>
#include "tcs3472.h"


// Define number of sensors to use (1-3)
#define NUM_SENSORS 3  // Change this value to 1, 2, or 3 for debugging

// Define the new addresses for sensors
#define TOF_DEFAULT_ADDR 0x29
#define TOF_ADDR1 0x30
#define TOF_ADDR2 0x31
#define TOF_ADDR3 0x32

// Define XSHUT pins
#define XSHUT1 IO_AR4
#define XSHUT2 IO_AR5
#define XSHUT3 IO_AR6

// Add this function before main()
void print_colour(uint16_t red, uint16_t green, uint16_t blue) {
    printf("\033[3F"); // Move cursor back 3 lines
    printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", (uint8_t)(red >> 4), (uint8_t)(green >> 4), (uint8_t)(blue >> 4));
    printf("R: %hu\n", red);
    printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", (uint8_t)(red >> 4), (uint8_t)(green >> 4), (uint8_t)(blue >> 4));
    printf("G: %hu\n", green);
    printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", (uint8_t)(red >> 4), (uint8_t)(green >> 4), (uint8_t)(blue >> 4));
    printf("B: %hu\n", blue);
    fflush(NULL);
}

int main(void) {
    pynq_init();

    // Initialize I2C
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    // Initialize XSHUT pins as outputs (only for the number of sensors we're using)
    gpio_set_direction(XSHUT1, GPIO_DIR_OUTPUT);
    if (NUM_SENSORS >= 2) gpio_set_direction(XSHUT2, GPIO_DIR_OUTPUT);
    if (NUM_SENSORS == 3) gpio_set_direction(XSHUT3, GPIO_DIR_OUTPUT);

// turn off the tcs3472 temporarily;
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
    
    printf("All TOF sensors verified successfully\n");

    // Initialize color sensor
//    tcs3472 sensor = TCS3472_EMPTY;
  //  int integration_time_ms = 60;
    //tcs_set_integration(&sensor, tcs3472_integration_from_ms(integration_time_ms));
    //tcs_set_gain(&sensor, x60);

    // Enable color sensor
    gpio_set_level(IO_AR8, GPIO_LEVEL_HIGH);
    sleep_msec(10);

   // if (tcs_init(IIC0, &sensor) != TCS3472_SUCCES) {
     //   printf("Failed to initialize color sensor\n");
       // return -1;
   // }
    printf("Color sensor initialized successfully\n");
    
    // Wait for first integration cycle
//    sleep_msec(integration_time_ms);
    printf("\n\n\n"); // Buffer space for color output

    // Main reading loop
    while(1) {
        // Read TOF sensors
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

        // Read color sensor
       // tcsReading rgb;
     //   int color_status = tcs_get_reading(&sensor, &rgb);
       // if (color_status == TCS3472_SUCCES) {
         //   printf("RGB: %hu, %hu, %hu\n", rgb.red, rgb.green, rgb.blue);
       // } else {
        //    printf("Color sensor reading failed\n");
       // }
        
        sleep_msec(500); // Wait 500ms between readings
    }

    // Cleanup (this won't be reached due to infinite loop)
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_SUCCESS;
}
