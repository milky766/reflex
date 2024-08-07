//
// Created by Sherry Wang on 2021/08/23.
//

#ifndef CONTROL_BOARD_TOP_CONTROLLER_H
#define CONTROL_BOARD_TOP_CONTROLLER_H
#include <math.h>
#include <cmath>
#include <cfenv>
#include <string>
using namespace std;
#define PI acos(-1)
#define MS_PER_SEC 1000
#define HZ 100

class topController {
public:
    struct goal_pressure{
        double  goal_pressure_m1;
        double goal_pressure_m2;
    };

    topController(double static_val);
    topController(int left, int right, double max_val, double min_val, int reach_duration, int update_rate);
    topController(double max_val, double min_val, int cycle_duration, int update_rate);

    ~topController();

    goal_pressure static_status();

    goal_pressure rhythmic_movement();

    goal_pressure reaching_movement();

    goal_pressure get_pattern(int mode);
    goal_pressure get_pattern(int mode, int &parameter);
    goal_pressure get_pattern(int mode, double PreAlpha_m1, double PreAlpha_m2);

    int isStart();
    int isStart(int mode);

private:
    goal_pressure pre_command;
    goal_pressure next_command;
    double _sin_trans_func(double t);
    double _linear_trans_func(double t);
//    double _goal_pressure_m1;
//    double _goal_pressure_m2;
    double _max_val;
    double _min_val;
    int _cycle_duration;
    int _update_rate;
    int last_para_a;
    int last_para_b;
    double last_alpha_m1;
    double last_alpha_m2;
    double _t;
    double _step_size;
    double _last_t;
    bool right_to_left;
    bool left_to_right;
    double g, g_pre;
    double _mid_val, _delta_p;
    double _total_steps;
};

#endif //CONTROL_BOARD_TOP_CONTROLLER_H
