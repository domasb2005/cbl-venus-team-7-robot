#include <libpynq.h>

int main(void) {
  pynq_init();
  buttons_init();

  // Setting up the buttons & LEDs
  gpio_set_direction(IO_BTN0, GPIO_DIR_INPUT);
  gpio_set_direction(IO_BTN1, GPIO_DIR_INPUT);
  gpio_set_direction(IO_BTN2, GPIO_DIR_INPUT);
  gpio_set_direction(IO_BTN3, GPIO_DIR_INPUT);

  gpio_set_direction(IO_LD0, GPIO_DIR_OUTPUT);
  gpio_set_direction(IO_LD1, GPIO_DIR_OUTPUT);
  gpio_set_direction(IO_LD2, GPIO_DIR_OUTPUT);
  gpio_set_direction(IO_LD3, GPIO_DIR_OUTPUT);

  // Testing library functions
  uint32_t waitMsec = 3000;
  printf("waiting until button 0 is pressed...\n");
  printf("Waited %d milliseconds\n\n", wait_until_button_pushed(0));
  printf("waiting until button 0 is released...\n");
  printf("Waited %d milliseconds\n\n", wait_until_button_released(0));
  printf(
      "Waiting %1.f seconds to see if IO_BTN0 was pushed (not returning early)\n",
      waitMsec / 1000.0);
  sleep_msec_button_pushed(0, waitMsec) ? printf("Pushed!\n\n")
                                        : printf("Not pushed!\n\n");
  printf("Checking all button states for %1.f seconds...\n",
         waitMsec * 2 / 1000.0);
  int buttonStates[NUM_BUTTONS];
  sleep_msec_buttons_pushed(buttonStates, waitMsec * 2);
  printf("IO_BTN0\tIO_BTN1\tIO_BTN2\tIO_BTN3\n");
  for (int i = 0; i < NUM_BUTTONS; i++) {
    printf("%i\t", buttonStates[i]);
  }

  if ((int)NUM_BUTTONS != (int)NUM_GREEN_LEDS) {
    pynq_error("NUM_BUTTONS = %d != NUM_GREEN_LEDS = %d\n", NUM_BUTTONS,
               NUM_GREEN_LEDS);
  }
  printf("\n\nPlease press one or more buttons simultaneously (^C to stop)\n");
  io_t buttons[NUM_BUTTONS] = {IO_BTN0, IO_BTN1, IO_BTN2, IO_BTN3},
         leds[NUM_GREEN_LEDS] = {IO_LD0, IO_LD1, IO_LD2, IO_LD3};
  gpio_level_t states[NUM_BUTTONS] = {0, 0, 0, 0},
               lastStates[NUM_BUTTONS] = {0, 0, 0, 0};
  int pressedButtons[NUM_BUTTONS] = {-1, -1, -1, -1}, nrPressedButtons = 0;

  while (true) {
    bool updateStates = false;
    // Set the states of all buttons compared to their current state - pressed
    // or not
    for (int i = 0; i < NUM_BUTTONS; i++) {
      states[i] = gpio_get_level(buttons[i]);
      gpio_set_level(leds[i], states[i]);
      if (states[i] != lastStates[i]) {
        updateStates = true;
      }
    }

    if (updateStates) {
      // Part that handles the response of the terminal to a button press
      nrPressedButtons = 0;
      // Set what buttons have been pressed and how many have been pressed
      for (int i = 0; i < NUM_BUTTONS; i++) {
        if (states[i]) {
          pressedButtons[nrPressedButtons] = i;
          nrPressedButtons++;
        }
      }
      for (int i = 0; i < NUM_BUTTONS; i++) {
        if (states[i] != lastStates[i]) {
          if (states[i] == 1) {
            printf("Press button %i -- ", i);
          } else {
            printf("Release button %i -- ", i);
          }
        }
      }

      if (nrPressedButtons == 0) {
        printf("all LEDs should be off");
      } else if (nrPressedButtons == 1) {
        printf("LED %i should be on", pressedButtons[0]);
      } else {
        printf("LEDs %i ", pressedButtons[0]);
        for (int i = 1; i < nrPressedButtons; i++) {
          printf("& %i ", pressedButtons[i]);
        }
        printf("should be on");
      }
      printf("\n");
    }
    for (int i = 0; i < NUM_BUTTONS; i++) {
      lastStates[i] = states[i];
    }
    sleep_msec(1000);
  }

  buttons_destroy();
  pynq_destroy();
  return EXIT_SUCCESS;
}
