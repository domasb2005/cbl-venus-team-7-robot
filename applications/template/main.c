#include <libpynq.h>

int main(void) {
  pynq_init();

        switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
        switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
        iic_init(IIC0);
  // your code here
  sleep_msec(100000);
  pynq_destroy();
  return EXIT_SUCCESS;
}
