#include <display-SCPI.h>
#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <ifaddrs.h>

static FontxFile font[2];    
static display_t display;
static SCPI_status currentSCPIStatus;
static int throbberIndex = 0;

#define CHARS_PER_LINE 20
#define MIN(x, y) ((x) < (y) ? (x) : (y))

void readIPAddress() {    
    char ip_address[100];

    // Get and print IP addresses for each network interface
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1) {
        perror("Error getting network interface addresses");
        return;
    }
    int itteration = 0;
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }

        // Convert the binary IP address to a string
        inet_ntop(AF_INET, &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr, ip_address, sizeof(ip_address));
        if (strncmp(ip_address, "127", 3) == 0) {
            continue;  // Skip the loop for addresses starting with "127"
        }
        // Print the IP address
        if (!itteration) {
            displayDrawString(&display, font, 20, 220, (uint8_t *) ip_address,
                        RGB_BLACK);
            itteration = 1;
        }
        else if (itteration) {
            displayDrawString(&display, font, 120, 220, (uint8_t *) ip_address,
                        RGB_BLACK);
        }
    }
    // Free the linked list of interface addresses
    freeifaddrs(ifaddr);
}

void initialiseDisplay() {
    InitFontx(font, "/fonts/ILGH16XB.FNT", ""); 
    display_init(&display);
    displayFillScreen(&display, RGB_WHITE);
    displaySCPIStatus(DISCONNECTED);
    FILE *file;
    file = fopen("/boot/version.txt", "r");    
    char version[100];
    if (file != NULL) {
        if (fgets(version, sizeof(version), file) != NULL) {
            if (version[strlen(version) - 1] == '\n') {
                version[strlen(version) - 1] = '\0';
            }
            displayMessage((uint8_t *)version);
            
        }
    }
}


void displayDestroy() {
    displaySCPIStatus(DISCONNECTED);
    displayDrawFillRect(&display,0,0,DISPLAY_WIDTH-1,DISPLAY_HEIGHT-1,RGB_BLACK);
    display_destroy(&display);
}

int findIndex(const char *str, char target) {
    int index = -1;  // Initialize index to -1 (not found)

    // Loop through the string until the end or until the target character is found
    for (int i = 0; str[i] != '\0'; ++i) {
        if (str[i] == target) {
            index = i;  // Update index if target character is found
            break;      // Exit the loop once the target character is found
        }
    }

    return index;
}

void displayMessage( uint8_t *Message) {
    //ToDo: Create bounding box around the message
    displayDrawFillRect(&display,0,0,DISPLAY_WIDTH-1,DISPLAY_HEIGHT-61, RGB_WHITE);

    int messageLength = strlen( (char*) Message);
    int remainingChars = messageLength;
    int currentline = 0;
    int index = 0;
    do {
        int charsToDisplay = (remainingChars < CHARS_PER_LINE ) ? remainingChars : CHARS_PER_LINE ;
        char substring[CHARS_PER_LINE+ 1];
        strncpy(substring, (char *) Message + index, charsToDisplay);
        substring[charsToDisplay] = '\0';
        int newlinePos = findIndex(substring, '~');
        if (newlinePos != -1) {   
            // Adjust the substring to end at the newline character
            charsToDisplay = newlinePos;
            substring[charsToDisplay] = '\0'; // Null-terminate the adjusted substring
            index += 1;
        }    
        index = index+charsToDisplay;    
        // Move to the next line
        currentline++;
        displayDrawString(&display, font, 40, 20 + 20*currentline, (uint8_t*) substring,
                    RGB_BLACK);
        if (currentline > 6) {
            return;
        }
    } while (index < messageLength);
}



void displayMeasurement(int measurement) {
    char str[32];
    sprintf(str, "%d", measurement);
    displayDrawString(&display, font, DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2, (uint8_t*) str,
                    RGB_BLACK);
}

void displaySCPIStatus(SCPI_status status) {
    //display_init(&display);
    displayUpdateThrobber();
    if (currentSCPIStatus == status) {
        return;
    }
    currentSCPIStatus = status;
    switch (status)
    {
    case CONNECTED:
        displayDrawFillRect(&display,0,180,DISPLAY_WIDTH-1,DISPLAY_HEIGHT-1, RGB_GREEN);
        displayDrawString(&display, font, 20, 200, (uint8_t *)"SCPI STATUS: CONNECTED",
                    RGB_BLACK);
        //Print connected succesfully + connection info
        break;
    case DISCONNECTED :
        displayDrawFillRect(&display,0,180,DISPLAY_WIDTH-1,DISPLAY_HEIGHT-1, RGB_WHITE);
        displayDrawString(&display, font, 20, 200, (uint8_t *) "SCPI STATUS: DISCONNECTED",
                    RGB_BLACK);
        //Print ERROR information
        break;
    default:
        break;
    }
    displayDrawLine(&display, 0,180,DISPLAY_WIDTH-1,180, RGB_BLACK);
    readIPAddress();
}


void displayUpdateThrobber() {
    throbberIndex = (throbberIndex + 1 )% 4;
    int x = 220, y = 215, distance = 10;
    if (currentSCPIStatus == CONNECTED ) {
    displayDrawFillRect(&display,x-distance,y-distance,x+distance,y+distance, RGB_GREEN);
    } else {
    displayDrawFillRect(&display,x-distance,y-distance,x+distance,y+distance, RGB_WHITE);
    
    }
    switch(throbberIndex) {
    case 0 :
        displayDrawLine(&display, x,y-distance,x,y, RGB_BLACK);
        break;
    case 1 :
        displayDrawLine(&display, x,y,x+distance,y, RGB_BLACK);
        break;
    case 2 :
        displayDrawLine(&display, x,y,x,y+distance, RGB_BLACK);
        break;
    case 3 :
        displayDrawLine(&display, x-distance,y,x,y, RGB_BLACK);
        break;
    default:
        break;
    }
}