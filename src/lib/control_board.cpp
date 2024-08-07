#include <control_board.h>
#include <cstdio>

ControlBoard::ControlBoard()
{

  if (bcm2835_init() != 1)
  {
    printf("bcm2835_init failed.");
  }

  gpio_ = new RaspberryPi_GPIO();
  spi_ = new RaspberryPi_SPI(gpio_);
  dac_ = new AD5360(spi_);
  adc_ = new AD7616(spi_, gpio_, adc_conversion_pin_);

  //ADC
  gpio_->set_mode(adc_conversion_pin_, Embedded_GPIO::gpio_mode::OUTPUT);
  gpio_->set_output(adc_conversion_pin_, Embedded_GPIO::gpio_state::ON);
  gpio_->set_mode(adc_cs_, Embedded_GPIO::gpio_mode::OUTPUT);
  gpio_->set_output(adc_cs_, Embedded_GPIO::gpio_state::ON);

  //DAC
  gpio_->set_mode(dac_latch_pin_, Embedded_GPIO::gpio_mode::OUTPUT);
  gpio_->set_output(dac_latch_pin_, Embedded_GPIO::gpio_state::OFF);
  gpio_->set_mode(dac_cs_, Embedded_GPIO::gpio_mode::OUTPUT);
  gpio_->set_output(dac_cs_, Embedded_GPIO::gpio_state::ON);

  //Heartbeat
  gpio_->set_mode(heartbeat_pin_, Embedded_GPIO::gpio_mode::OUTPUT);
  gpio_->set_output(heartbeat_pin_, Embedded_GPIO::gpio_state::OFF);

  //LoadCells
  for(uint8_t idx = 0; idx < NUMBER_OF_TRANSDUCERS; idx++)
  {
    gpio_->set_mode(load_cell_cs_[idx], Embedded_GPIO::gpio_mode::OUTPUT);
    gpio_->set_output(load_cell_cs_[idx], Embedded_GPIO::gpio_state::ON);
    gpio_->set_mode(load_cell_rdy_[idx], Embedded_GPIO::gpio_mode::INPUT);
    load_cell_[idx] = new AD7730(spi_, gpio_, load_cell_cs_[idx], load_cell_rdy_[idx]);
    load_cell_[idx]->softReset();
    load_cell_[idx]->setup();
  }
}

ControlBoard::~ControlBoard() {
  delete adc_;
  delete dac_;
  delete spi_;
  delete gpio_;

  for(auto & idx : load_cell_)
  {
    delete idx;
  }

  bcm2835_close();
}

void ControlBoard::update_inputs()
{
  //ADC
  for(uint8_t idx = 0; idx < 8; idx++)
  {
    uint32_t adc_rx = adc_->getMeasurementPair(adc_cs_, idx);
    adc_data_[idx] = (uint16_t) ((adc_rx & 0xFFFF0000) >> 16);
    adc_data_[idx + 8] = (uint16_t) (adc_rx & 0x0000FFFF);
  }

  //LOAD CELLS
  for(uint8_t idx = 0; idx < NUMBER_OF_TRANSDUCERS; idx++)
  {
    load_cell_data_[idx] = load_cell_[idx]->getResult();
  }

}

uint16_t ControlBoard::getADCdata(uint8_t input)
{
  return adc_data_[input];
}

double ControlBoard::getInputVoltage(uint8_t input)
{
  return (10.0 / 32768.0) * adc_data_[input];
}

double ControlBoard::getInputPressure(uint8_t input)
{
  return (1.0 / 4.0) * ((getInputVoltage(input)) - 1.0);
}

double ControlBoard::getMuscleSpindleData(uint8_t input, std::string command)
{
    if (command == "mshi") //high resistance
    {
        double voltage = getInputVoltage( input);
        return (V_REF - voltage) / (voltage / 1000); //get the resistance of the fabric sensor. unit is ohm
    }
    else if (command == "mslo") //low resistance
    {
        double voltage = getInputVoltage(input);
        return voltage / RESISTOR_VALUE;
    }
    else
    {
        return -1;
    }
}

double ControlBoard::getPotentiometerData(uint8_t input) {
    double voltage = getInputVoltage(input);
    return voltage * 72;
}

uint16_t ControlBoard::getLoadCellData(uint8_t input)
{
  return load_cell_data_[input];
}

void ControlBoard::setOutputVoltage(uint8_t output, double voltage)
{
  dac_->setVoltage(dac_cs_, output / 8, output % 8, voltage);
}

void ControlBoard::setOutputNormalized(uint8_t output, double value)
{
  dac_->setNormalized(dac_cs_, output / 8, output % 8, value);
}

void ControlBoard::toggleHeartbeat()
{
  if(heartbeat_state_)
  {
    gpio_->set_output(heartbeat_pin_, Embedded_GPIO::gpio_state::OFF);
    heartbeat_state_ = !heartbeat_state_;
  } else
  {
    gpio_->set_output(heartbeat_pin_, Embedded_GPIO::gpio_state::ON);
    heartbeat_state_ = !heartbeat_state_;
  }
}