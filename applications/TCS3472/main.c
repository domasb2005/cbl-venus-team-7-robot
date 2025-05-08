#include <libpynq.h>
#include <iic.h>
#include "tcs3472.h"
#include <stdio.h>


#define XSHUT1 IO_AR4
#define XSHUT2 IO_AR5
#define XSHUT3 IO_AR6



void print_colour(uint16_t red, uint16_t green, uint16_t blue)
{
	printf("\033[3F"); //Move cursor back 3 lines
	printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", CLAMP_255((red >> 4)), CLAMP_255((green >> 4)), CLAMP_255((blue >> 4))) ;//print uint8_t's
	printf("R: %hu\n",red); //print uint16_t
	printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", CLAMP_255((red >> 4)), CLAMP_255((green >> 4)), CLAMP_255((blue >> 4))) ;//print uint8_t's
	printf("G: %hu\n", green); //print uint16_t
	printf("\033[48;2;%hhu;%hhu;%hhum      \033[0m", CLAMP_255((red >> 4)), CLAMP_255((green >> 4)), CLAMP_255((blue >> 4))) ;//print uint8_t's
	printf("B: %hu\n", blue); //print uint16_t
	fflush(NULL);
}


int main(void) {
  	pynq_init();

	  gpio_set_direction(XSHUT1, GPIO_DIR_OUTPUT);
	  gpio_set_direction(XSHUT2, GPIO_DIR_OUTPUT);
	  gpio_set_direction(XSHUT3, GPIO_DIR_OUTPUT);
	  
	  gpio_set_level(XSHUT1, GPIO_LEVEL_LOW);
	  gpio_set_level(XSHUT2, GPIO_LEVEL_LOW);
	  gpio_set_level(XSHUT3, GPIO_LEVEL_LOW);
	  

	gpio_set_direction(IO_AR8, GPIO_DIR_OUTPUT);
	gpio_set_level(IO_AR8, GPIO_LEVEL_HIGH);
	//Init the IIC pins
	switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
	switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
	iic_init(IIC0);

	/******** Simple connection test *******/
	uint8_t id;
	int i = tcs_ping(IIC0, &id);
	printf("---Detection: ");
	if(i != TCS3472_SUCCES)
	{
		printf("Fail\n");
		return 1;
	}
	printf("Succes\n");
	printf("-- ID: %#X\n", id);
	/***************************************/

	//Due to 50hz powergrind and thus 50hz flicker in indoor lighting.
	//Always use a 20ms increment in the integration time.
	//Period of 50hz is 20ms
	int integration_time_ms = 60; 

	/********** Preconfigure sensor ********/
	tcs3472 sensor = TCS3472_EMPTY;
	tcs_set_integration(&sensor, tcs3472_integration_from_ms(integration_time_ms));
	tcs_set_gain(&sensor, x16);

	//enable sensor -> loads preconfiguration
	i = tcs_init(IIC0, &sensor);
	printf("---Sensor Init: ");
	if(i != TCS3472_SUCCES)
	{
		printf("Fail\n");
		return 1;
	}
	printf("Succes\n");
	fflush(NULL);
	/***************************************/


	//Wait until one integration cycle is completed
	//Else the readings are 0 0 0 0
	sleep_msec(integration_time_ms);

	tcsReading rgb;
	printf("        \n        \n        \n"); //Buffer some space
	while(1)
	{
		i = tcs_get_reading(&sensor, &rgb);
		print_colour(rgb.red, rgb.green, rgb.blue); //Used to print colour to screen
		sleep_msec(integration_time_ms + 20);
	}

	iic_destroy(IIC0);
	pynq_destroy();
	return EXIT_SUCCESS;
}
