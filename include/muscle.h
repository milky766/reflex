#ifndef MUSCLE_H
#define MUSCLE_H

#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include <control_board.h>
#include <pid_controller.h>
#include <string>

#define VALVE_NEUTRAL_VOLTAGE 5.0

class Muscle {
public:
  enum ControlMode {activation, pressure};

  struct pid_cfg_t{
    double p;
    double i;
    double d;
    double lower_clamp;
    double upper_clamp;
  };

  struct muscle_cfg_t{
      //设置接口
    uint16_t adc_index;
    //两个通道同步采样，V0A只能和V0B同时采样
    uint16_t dac_index;
    uint16_t tension_sensor_index; //どのセンサーから情報を取得するか、というindex
    uint16_t muscle_spindle_index;
    pid_cfg_t pid_cfg;
    ControlBoard *board;
    std::string mslo_mshi;
  };

  struct muscle_state_t{
      //muscle状态
    double current_activation;
    double current_pressure;
    uint16_t current_tension_sensor_feedback;

    double current_ms_resistance;
  };

  struct muscle_cmd_t{
      //muscle指令
    ControlMode control_mode;
    double goal_pressure;
    double goal_activation;


  };

  Muscle(muscle_cfg_t muscle_config); //コンストラクタ（記憶域確保）、muscle_configを引数に取る
  //根据configation设置每个肌肉的接口 
  ~Muscle(); //デストラクタ（記憶域解放）
 
  //根据muscle指令，更新muscle状态 メンバ関数
  muscle_state_t updateMuscle(muscle_cmd_t muscle_cmd);
  //打印muscle状态
  muscle_state_t printMuscle(muscle_cmd_t muscle_cmd);
  //得到muscle当前状态
  muscle_state_t getMuscleState();

private:
  uint16_t adc_index_;
  uint16_t dac_index_;
  uint16_t tension_sensor_index_;
  uint16_t muscle_spindle_index_;
  ControlBoard *board_;
  PidController *pid_controller_;

  double current_activation_;
  std::string mslo_mshi_;

  void initialize_valves(); //voidは返し値がないことを示す

};

#endif //MUSCLE_H
