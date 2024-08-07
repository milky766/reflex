//テンションセンサのドライバ
#ifndef AD7730_H
#define AD7730_H

#include <math.h>
#include <stdint.h>
#include <embedded_spi.h>
#include <embedded_gpio.h>

#define AD7730_SPI_TX_BUFFER_LEN 5
#define AD7730_SPI_RX_BUFFER_LEN 64
#define AD7730_USING_DATA_LENGTH 3

#define READ_ONLY 0xFF
//Communication Register Values
#define CR_SINGLE_WRITE 0x00
#define CR_SINGLE_READ 0x10
#define CR_CONTINUOUS_READ_START 0x20
#define CR_CONTINUOUS_READ_STOP 0x30

#define CR_COMMUNICATION_REGISTER 0x00 //Write only
#define CR_STATUS_REGISTER 0x00 //Read only
#define CR_DATA_REGISTER 0x01
#define CR_MODE_REGISTER 0x02
#define CR_FILTER_REGISTER 0x03
#define CR_DAC_REGISTER 0x04
#define CR_OFFSET_REGISTER 0x05
#define CR_GAIN_REGISTER 0x06
#define CR_TEST_REGISTER 0x07

//Mode Register Values
#define MR1_MODE_IDLE 0x00
#define MR1_MODE_CONTINUOUS 0x20 //Standard Operation
#define MR1_MODE_SINGLE 0x40
#define MR1_MODE_STANDBY 0x60
#define MR1_MODE_INTERNAL_ZERO_CALIBRATION 0x80
#define MR1_MODE_INTERNAL_FULL_CALIBRATION 0xA0
#define MR1_MODE_SYSTEM_ZERO_CALIBRATION 0xC0
#define MR1_MODE_SYSTEM_FULL_CALIBRATION 0xE0
#define MR1_BU_BIPOLAR 0x00 //+- voltage defined by MR0_RANGE
#define MR1_BU_UNIPOLAR 0x10 //0 to voltage deifined by MRO_RANGE
#define MR1_WL_24_BIT 0x01
#define MR1_WL_16_BIT 0x00

#define MR0_HIREF_5V 0x80
#define MR0_HIREF_2P5V 0x00
#define MR0_RANGE_10MV 0x00
#define MR0_RANGE_20MV 0x01
#define MR0_RANGE_40MV 0x02
#define MR0_RANGE_80MV 0x03
#define MR0_CHANNEL_1 0x00
#define MR0_CHANNEL_2 0x01
#define MR0_CHANNEL_SHORT_1 0x02 //Used for internal noise check
#define MR0_CHANNEL_NEGATIVE_1_2 0x03 //Unknown use
#define MRO_BURNOUT_ON 0x04 //Advanced, to check if loadcell is burnt out

//Filter Register Values
#define FR2_SINC_AVERAGING_2048 0x80  //Base sample rate of 50 Hz
#define FR2_SINC_AVERAGING_1024 0x40  //Base sample rate of 100 Hz
#define FR2_SINC_AVERAGING_512 0x20   //Base sample rate of 200 Hz
#define FR2_SINC_AVERAGING_256 0x10   //Base sample rate of 400 Hz

#define FR1_SKIP_ON 0x02 //the FIR filter on the part is bypassed
#define FR1_SKIP_OFF 0x00
#define FR1_FAST_ON 0x01 //FIR is replaced with moving average on large step, sinc filter averages are used to compensate
#define FR1_FAST_OFF 0x00

#define FR0_CHOP_ON 0x10 //When the chop mode is enabled, the part is effectively chopped at its input and output to remove all offset and offset drift errors on the part.
#define FR0_CHOP_OFF 0x00 //Increases sample rate by x3

//DAC Register Values
#define DACR_OFFSET_SIGN_POSITIVE 0x00
#define DACR_OFFSET_SIGN_NEGATIVE 0x20
#define DACR_OFFSET_40MV 0x10
#define DACR_OFFSET_20MV 0x08
#define DACR_OFFSET_10MV 0x04
#define DACR_OFFSET_5MV 0x02
#define DACR_OFFSET_2P5MV 0x01
#define DACR_OFFSET_NONE 0x00

//current settings
#define CURRENT_MODE_1_SETTINGS (MR1_BU_UNIPOLAR | MR1_WL_24_BIT)
#define CURRENT_MODE_0_SETTINGS (MR0_HIREF_5V | MR0_RANGE_80MV | MR0_CHANNEL_1)



class AD7730 {
public:
  AD7730(Embedded_SPI *dev, Embedded_GPIO *gpio, uint8_t chip_select, uint8_t ready_signal);

  void setup();

  void softReset();

  void initConversion();

  bool isReady();

  void test();

  uint16_t getResult();

private:
  Embedded_SPI *dev_;
  Embedded_GPIO *gpio_;
  uint8_t chip_select_;
  uint8_t ready_signal_;
  uint8_t spi_tx_buffer_[AD7730_SPI_TX_BUFFER_LEN];
  char spi_rx_buffer_[AD7730_SPI_RX_BUFFER_LEN];
  char tx_command_buffer_[AD7730_SPI_RX_BUFFER_LEN];
  uint8_t filter_register_[3] = {FR2_SINC_AVERAGING_1024, FR1_SKIP_OFF | FR1_FAST_OFF, FR0_CHOP_OFF};
  uint8_t mode_register_[2] = {0x51, 0xB4};
  char mode_register_2_[2] = {(char)0b00110000, (char)0b10110100};
  uint8_t register_sizes_[8] = {1, 3, 2, 3, 1, 3, 3, 3};

  uint16_t latest_data_ = 0;

  void readRegister(uint8_t reg, char data[]);

  void writeRegister(uint8_t reg, char data[]);

  void setCommunicationMode(uint8_t com_type, uint8_t reg_type);

};


#endif //AD7730_H
