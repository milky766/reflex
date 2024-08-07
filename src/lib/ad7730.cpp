#include "ad7730.h"
#include <cstdio>

AD7730::AD7730(Embedded_SPI *dev, Embedded_GPIO *gpio, uint8_t chip_select, uint8_t ready_signal)
{
  dev_ = dev;
  gpio_ = gpio;
  chip_select_ = chip_select;
  ready_signal_ = ready_signal;
}

void AD7730::setup()
{
  char filter[3] = {1,2,3};
  readRegister(CR_FILTER_REGISTER, filter);
  writeRegister(CR_FILTER_REGISTER, (char*)filter_register_);
  readRegister(CR_FILTER_REGISTER, filter);
  initConversion();
}

void AD7730::softReset()
{
  uint8_t command[4] = { 0xFF, 0xFF, 0xFF, 0xFF};
  dev_->transferSPI(chip_select_, 4, (char*)command);
}

void AD7730::setCommunicationMode(uint8_t com_type, uint8_t reg_type)
{
  char mode_command = com_type | reg_type;
  char command[1] = {mode_command};
  dev_->transferSPI(chip_select_, 1, command);
}

void AD7730::readRegister(uint8_t reg, char data[])
{
  setCommunicationMode(CR_SINGLE_READ, reg);
  char dummy[3] = {0,0,0};
  dev_->transferSPI(chip_select_, register_sizes_[reg], dummy, data);
}

void AD7730::writeRegister(uint8_t reg, char data[])
{
  setCommunicationMode(CR_SINGLE_WRITE, reg);
  dev_->transferSPI(chip_select_, register_sizes_[reg], data);
}

void AD7730::initConversion()
{
  //char conversion_command[2] = {MR1_MODE_CONTINUOUS | CURRENT_MODE_1_SETTINGS, (char)CURRENT_MODE_0_SETTINGS};
  writeRegister(CR_MODE_REGISTER, mode_register_2_);
  setCommunicationMode(CR_CONTINUOUS_READ_START, CR_DATA_REGISTER);
}

bool AD7730::isReady()
{
  return gpio_->read_input(ready_signal_) == Embedded_GPIO::OFF;
}

void AD7730::test()
{
  printf("Checking LCDAC (CS:%d and RDY:%d)\n", chip_select_, ready_signal_);

  char filter[3] = {1,2,3};
  readRegister(CR_FILTER_REGISTER, filter);
  printf("Default Filter: 0x%.2X,0x%.2X,0x%.2X\n", filter[0], filter[1], filter[2]);

  printf("Setting Up Filter Register 0x%.2X,0x%.2X,0x%.2X\n", filter_register_[0], filter_register_[1], filter_register_[2]);
  writeRegister(CR_FILTER_REGISTER, (char*)filter_register_);

  readRegister(CR_FILTER_REGISTER, filter);
  printf("Read Filter as: 0x%.2X,0x%.2X,0x%.2X\n", filter[0], filter[1], filter[2]);

  initConversion();
  getResult();
}

uint16_t AD7730::getResult()
{
  if(isReady())
  {
    char data[2];
    char dummy[2] = {0,0};
    dev_->transferSPI(chip_select_, 2, dummy, data);
    //printf("\nConversion Result: 0x%.2X,0x%.2X,0x%.2X\n", data[0], data[1]);
    latest_data_ = (uint16_t) data[0] << 8 | (uint16_t) data[1];
    return latest_data_;
  } else {
    return latest_data_;
  }
}
