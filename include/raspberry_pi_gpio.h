#ifndef RASPBERRY_PI_GPIO_H
#define RASPBERRY_PI_GPIO_H

#include <embedded_gpio.h>
#include <bcm2835.h>

class RaspberryPi_GPIO : public Embedded_GPIO {
public:

  RaspberryPi_GPIO();

  ~RaspberryPi_GPIO();

  virtual void init(int gpio_address);

  virtual void deinit(int gpio_address);

  virtual void set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode);

  virtual void set_output(int gpio_address, Embedded_GPIO::gpio_state state);

  virtual Embedded_GPIO::gpio_state read_input(int gpio_address);

};


#endif //RASPBERRY_PI_GPIO_H
