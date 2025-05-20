#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

 /*
 *  TU/e 5EID0::LIBPYNQ Driver for VL53L0X TOF Sensor
 *  Example Code 
 * 
 *  Original: Larry Bank
 *  Adapted for PYNQ: Walthzer
 * 
 */

extern int vl53l0x_example_single();
extern int vl53l0x_example_dual();
extern int change_address();
extern int scan_address();
extern int read_all();
extern int scan_channels(void);
extern int read_three();
extern int test_gpio();
// extern int selectMuxChannel(uint8_t channel);
/** This Example program REQUIRES ALL OF:
 * - single.c
 * - dual.c
 * To be present.
**/
#define MUX_ADDRESS 0x70
void selectMuxChannel(uint8_t channel) {

    if (channel > 7) return;
    uint8_t data = 1 << channel;
    iic_write_register(IIC0, MUX_ADDRESS, 0, &data, 1);
}
int scan_address(void) {


    uint8_t start_address = 0x08;  // Start from 0x08 to avoid reserved addresses
    uint8_t end_address = 0x77;    // End at 0x77 (7-bit address range)
    uint8_t found_devices = 0;
    uint8_t dummy_data;

    printf("\nI2C Scanner ready!");
    printf("\nScanning I2C bus from 0x%02X to 0x%02X...\n\n", start_address, end_address);

    // Scan through all addresses
    for (uint8_t addr = start_address; addr <= end_address; addr++) {
        // Try to read from the device
        int result = iic_read_register(IIC0, addr, 0x00, &dummy_data, 1);
        
        printf("0x%02X: ", addr);
        if (result == 0) {
            printf("Device found!");
            found_devices++;
        } else {
            printf("--");
        }
        
        // New line every 8 addresses
        if ((addr + 1) % 8 == 0) {
            printf("\n");
        } else {
            printf("\t");
        }
    }

    printf("\nScan completed! Found %d devices.\n", found_devices);


    
    return EXIT_SUCCESS;
}
int main(void) {
  	pynq_init();

	//Setting up the buttons & LEDs
	//Init the IIC pins
	switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
	switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
	iic_init(IIC0);

	/**Test Scripts: Select ONE!
	 * - Single Sensor -> vl53l0x_example_single();
	 * - Dual Sensor -> vl53l0x_example_dual();
	**/	
	
	/** Connect one sensor to the IIC bus and enjoy! **/
	// selectMuxChannel(7);
	// sleep_msec(1000);
	scan_address();
	// vl53l0x_example_single();
	// // read_all();
	// // scan_channels();
	// selectMuxChannel(7);
	// sleep_msec(1000);
//	 vl53l0x_example_single();

//	read_three();
	// test_gpio();

	//  vl53l0x_example_single();
	//  read_three();
// vl53l0x_example_single();	


	/** Connect two sensors to the IIC bus
	 * ONLY CONNECT ONE TO 5V and GND
	 * --If the second sensor is connected to either 5v or GND
	 * --Before the first is initialized, they conflict.
	 * Run the program and follow instructions! **/
	//vl53l0x_example_dual();

	iic_destroy(IIC0);
	pynq_destroy();
	return EXIT_SUCCESS;
}
