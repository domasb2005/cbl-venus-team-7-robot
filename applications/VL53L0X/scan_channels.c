#include <libpynq.h>
#include <iic.h>
#include <stdio.h>
#include <stdbool.h>

#define TCAADDR 0x70

// Function to select TCA9548A channel
static void tcaselect(uint8_t channel) {
    if (channel > 7) return;
    
    uint8_t data = 1 << channel;
    iic_write_register(IIC0, TCAADDR, 0, &data, 1);
}

int scan_channels(void) {
    pynq_init();
    
    // Initialize I2C
    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
    iic_init(IIC0);

    printf("\nTCA9548A Scanner ready!\n");

    // Scan each TCA9548A channel
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
        sleep_msec(10); // Give multiplexer time to switch
        
        printf("\nTCA Port #%d:\n", t);

        // Scan all possible I2C addresses on this channel
        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            // Don't scan the TCA9548A itself
            if (addr == TCAADDR) continue;

            uint8_t dummy_data;
            // Try to read from the device
            int response = iic_read_register(IIC0, addr, 0x00, &dummy_data, 1);
            
            if (response == 0) {
                printf("Found I2C 0x%02X\n", addr);
            }
        }
        
        // Delay between channel scans
        sleep_msec(1000);
    }

    printf("\nScan completed.\n");

    // Cleanup
    iic_destroy(IIC0);
    pynq_destroy();
    
    return EXIT_SUCCESS;
}