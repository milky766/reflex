/*
 * @Author: error: git config user.name && git config user.email & please set dead value or install git
 * @Date: 2021-07-16 20:43:50
 * @LastEditors: error: git config user.name && git config user.email & please set dead value or install git
 * @LastEditTime: 2022-06-29 01:45:13
 * @FilePath: \control_board_yanlin_revise\src\lib\muscle.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <muscle.h>

Muscle::Muscle(Muscle::muscle_cfg_t muscle_config)
//把外部的configuration参数传入新建的muscle对象
{
  adc_index_ = muscle_config.adc_index;
  dac_index_ = muscle_config.dac_index;
  tension_sensor_index_ = muscle_config.tension_sensor_index;
  muscle_spindle_index_ = muscle_config.muscle_spindle_index;
  board_ = muscle_config.board; //board指针
    //pid_controller pointer -- 新建一个PidController对象
  pid_controller_ = new PidController(muscle_config.pid_cfg.p, muscle_config.pid_cfg.i, muscle_config.pid_cfg.d,
                                      muscle_config.pid_cfg.upper_clamp, muscle_config.pid_cfg.lower_clamp);

  current_activation_ = 0.0;
  mslo_mshi_ = muscle_config.mslo_mshi;
  initialize_valves();
}

Muscle::~Muscle()
{
  delete pid_controller_;
}           

Muscle::muscle_state_t Muscle::updateMuscle(Muscle::muscle_cmd_t muscle_cmd)
{
  if(muscle_cmd.control_mode == Muscle::ControlMode::activation)
  {
    board_->setOutputNormalized(dac_index_, muscle_cmd.goal_activation);
    current_activation_ = muscle_cmd.goal_activation;
  } else
  {
    double goal_pressure = muscle_cmd.goal_pressure;    
    double current_pressure = board_->getInputPressure(adc_index_);
    double error = goal_pressure - current_pressure;

    double command_voltage = pid_controller_->update(error) + VALVE_NEUTRAL_VOLTAGE;

    if(command_voltage < 0.0 || command_voltage > 10.0)
    {
      std::cerr << "Voltage out of Range with: " << command_voltage << "V" << std::endl;
    }else
    {
      board_->setOutputVoltage(dac_index_, command_voltage);
      //Calculate activation value from voltage
      // (out_range * (input / input_max)) - out_range_half
      current_activation_ = (2 * (command_voltage / 10.0)) - 1;
    }
  }

  return {.current_activation = current_activation_,
          .current_pressure = board_->getInputPressure(adc_index_),
          .current_tension_sensor_feedback = board_->getLoadCellData(tension_sensor_index_),
            .current_ms_resistance = board_->getMuscleSpindleData(muscle_spindle_index_, mslo_mshi_)
  };
}

Muscle::muscle_state_t Muscle::printMuscle(Muscle::muscle_cmd_t muscle_cmd)
{
  std::cout << "@@************************************************@@"<< std::endl;
  std::cout << "current Muscle  " << adc_index_ << "  state :"<< std::endl;
  std::cout << " goal pressure is                " << muscle_cmd.goal_pressure << std::endl
            << "while current pressure is        "<< board_->getInputPressure(adc_index_) <<std::endl;
  std::cout << "ADC7616 pressure data is       0d" << board_->getADCdata(adc_index_) << std::endl;
  std::cout << "ADC7616 Muscle spindal data is 0d" << board_->getADCdata(muscle_spindle_index_) << std::endl;
  std::cout << "ADC7730 Tension data is        0d" << board_->getLoadCellData(tension_sensor_index_) << std::endl;

  return {.current_activation = current_activation_,
          .current_pressure = board_->getInputPressure(adc_index_),
          .current_tension_sensor_feedback = board_->getLoadCellData(tension_sensor_index_),
            .current_ms_resistance = board_->getMuscleSpindleData(muscle_spindle_index_, mslo_mshi_)
  };
}

void Muscle::initialize_valves(){
  board_->setOutputVoltage(dac_index_, 7.0);
  usleep(100);
  board_->setOutputVoltage(dac_index_, 4.0);
}

