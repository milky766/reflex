//ファイバーセンサのドライバ
#ifndef AD7616_H
#define AD7616_H

#include <math.h>
#include <stdint.h>
#include <embedded_spi.h>
#include <embedded_gpio.h>

class AD7616 {
public:
  /**
   * Constructor
   * @param dev interface to use the platform's SPI bus
   */
  AD7616(Embedded_SPI *dev, Embedded_GPIO *gpio, int trigger_measurement_gpio);

  /**
   * Issues a measurement of a certain channel pair [VAx & VBx]
   * @param cs cs chip-select id of ADC
   * @param channel_pair the channel-pair to get a measurement from
   * @return
   */
  uint32_t getMeasurementPair(int cs, uint8_t channel_pair);

private:
  Embedded_SPI *dev_;
  Embedded_GPIO *gpio_;
  int trigger_measurement_gpio_;

  void prepareChannel(uint8_t channel, int cs);

  char rx_buffer_[4] = {0, 0, 0, 0};
  char tx_buffer_[4] = {0, 0, 0, 0};
  char channel_select_command_[4] = {(char)0x86, (char)0x00, (char)0x86, (char)0x00};
};


#endif //AD7616_H
