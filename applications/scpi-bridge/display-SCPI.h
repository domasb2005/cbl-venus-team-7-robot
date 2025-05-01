#ifndef display_SCPI
#define display_SCPI

#include <libpynq.h>

#define CHAR_PER_LINE 20


typedef enum {
    CONNECTED,
    DISCONNECTED
} SCPI_status; 

/*
 * @brief Support file for use of display during SCPI execution.
*/

void initialiseDisplay();
// display_init + displayString + initialise static font

void displayMessage(uint8_t *Message);
// Display text on the display

void displayMeasurement(int measurement); 
// Display data on the display 

void displaySCPIStatus(SCPI_status status);
// Display message when connection status changes

void displayDestroy();

void displayUpdateThrobber();

#endif