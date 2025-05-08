#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stepper.h>

int main(void) {
    pynq_init();
    stepper_init();
    stepper_enable();
  
  
   // while (1) {
  
  
  
      stepper_set_speed(13000, 13000);
      stepper_steps(1200, 1200);
//  sleep_sec(5);
  
      sleep_msec(5000);
   // }
  
    stepper_disable();
    stepper_destroy();
    pynq_destroy();
    return EXIT_SUCCESS;
  }
