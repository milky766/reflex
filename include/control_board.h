#ifndef CONTROL_BOARD_H
#define CONTROL_BOARD_H

#include <vector>

#include <raspberry_pi_spi.h>
#include <raspberry_pi_gpio.h>
#include <bcm2835.h>
#include <ad5360.h>
#include <ad7616.h>
#include <ad7730.h>
#include <string>

#define NUMBER_OF_TRANSDUCERS 8
#define V_REF 5
#define RESISTOR_VALUE 0.10245

class ControlBoard{
public:
  ControlBoard();
  ~ControlBoard();

  void update_inputs();

  //AD7616

  uint16_t getADCdata(uint8_t input);
  
  double getInputVoltage(uint8_t input);

  double getInputPressure(uint8_t input);

  double getMuscleSpindleData(uint8_t input, std::string command);

  double getPotentiometerData(uint8_t input);

  //AD7730
  uint16_t  getLoadCellData(uint8_t input);

  void setOutputVoltage(uint8_t output, double voltage);

  void setOutputNormalized(uint8_t output, double value);
 
  void toggleHeartbeat();

private:
  AD5360 *dac_;
  AD7616 *adc_;
  AD7730 *load_cell_[NUMBER_OF_TRANSDUCERS];
  RaspberryPi_SPI *spi_;
  RaspberryPi_GPIO *gpio_;

  int dac_latch_pin_ = RPI_V2_GPIO_P1_11;
  int adc_conversion_pin_ = RPI_V2_GPIO_P1_05;

  int heartbeat_pin_ = RPI_V2_GPIO_P1_03;
  bool heartbeat_state_ = true;

  int adc_cs_ = RPI_V2_GPIO_P1_07;
  int dac_cs_ = RPI_V2_GPIO_P1_12;

  int load_cell_cs_[NUMBER_OF_TRANSDUCERS] = {RPI_V2_GPIO_P1_29, RPI_V2_GPIO_P1_26, RPI_V2_GPIO_P1_24, RPI_V2_GPIO_P1_22,
                                              RPI_V2_GPIO_P1_40, RPI_V2_GPIO_P1_38, RPI_V2_GPIO_P1_37, RPI_V2_GPIO_P1_36};

  int load_cell_rdy_[NUMBER_OF_TRANSDUCERS] = {RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_16, RPI_V2_GPIO_P1_18,
                                               RPI_V2_GPIO_P1_31, RPI_V2_GPIO_P1_32, RPI_V2_GPIO_P1_33, RPI_V2_GPIO_P1_35};

  uint16_t adc_data_[16];
  uint16_t load_cell_data_[NUMBER_OF_TRANSDUCERS];

};

#endif //CONTROL_BOARD_H
