#include <ad7616.h>

AD7616::AD7616(Embedded_SPI *dev, Embedded_GPIO *gpio, int trigger_measurement_gpio)
{
  dev_ = dev;
  gpio_ = gpio;
  trigger_measurement_gpio_ = trigger_measurement_gpio;
}

void AD7616::prepareChannel(uint8_t channel, int cs)
{
  channel_select_command_[1] = channel | channel << 4;
  channel_select_command_[3] = channel | channel << 4;

  dev_->transferSPI(cs, 4, channel_select_command_, rx_buffer_);

  //Dummy read to get rid of values of the last measurement
  gpio_->set_output(trigger_measurement_gpio_, Embedded_GPIO::gpio_state::ON);
  gpio_->set_output(trigger_measurement_gpio_, Embedded_GPIO::gpio_state::OFF);


  dev_->transferSPI(cs, 4, channel_select_command_, rx_buffer_);
}

uint32_t  AD7616::getMeasurementPair(int cs, uint8_t channel)
{
  prepareChannel(channel, cs);
  //cs = RPI_GPIO_P1_07

  gpio_->set_output(trigger_measurement_gpio_, Embedded_GPIO::gpio_state::ON);
  gpio_->set_output(trigger_measurement_gpio_, Embedded_GPIO::gpio_state::OFF);

  dev_->transferSPI(cs, 4, tx_buffer_, rx_buffer_);
//    char rx_buffer_[4] = {0, 0, 0, 0};
//    char tx_buffer_[4] = {0, 0, 0, 0};
  uint32_t  ret = 0;
  ret = (uint8_t)rx_buffer_[0] << 24 | (uint8_t)rx_buffer_[1] << 16 | (uint8_t)rx_buffer_[2] << 8 | (uint8_t)rx_buffer_[3];

  return ret;
}