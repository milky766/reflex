//
// Created by Sherry Wang on 2021/08/23.
//

#include <top_controller.h>
#include <iostream>

//STATIC
topController::topController(double static_val) {
    _max_val = static_val;
    _min_val = static_val;
    pre_command = {0.1, 0.1};
    next_command = {0.1, 0.1};
}

//ROTATION
topController::topController(double max_val, double min_val, int cycle_duration, int update_rate)
{
    _max_val = max_val;
    _min_val = min_val;
    _cycle_duration = cycle_duration;
    _update_rate = update_rate;
    _t = 0.0;
    _last_t = 0.0;
    _step_size = (2 * PI / _update_rate) / cycle_duration;
    pre_command = {0.1, 0.1};
    next_command = {0.1, 0.1};
}

//REACHING
topController::topController(int left, int right, double max_val, double min_val, int reach_duration, int update_rate) {
    _max_val = max_val;
    _min_val = min_val;
    _delta_p = _max_val - _min_val;
    _update_rate = update_rate;
    _t = -1;
    _total_steps = reach_duration * _update_rate;
    pre_command = {0.1, 0.1};
    next_command = {0.1, 0.1};
}


topController::~topController() {}


topController::goal_pressure topController::static_status() {
    return {
        .goal_pressure_m1 = _min_val,
        .goal_pressure_m2 = _max_val
    };
}

topController::goal_pressure topController::rhythmic_movement() {
    _last_t = _t;
    //_t = _t + _step_size;
    _t = fmod(_t + _step_size, 2*PI);
    return{
        .goal_pressure_m1 = _sin_trans_func(_t),
        .goal_pressure_m2 = _sin_trans_func(_t + PI)
    };
}

topController::goal_pressure topController::reaching_movement() {
    //匀速
    _t++;
    return {
        .goal_pressure_m1 = _linear_trans_func(_t) + _min_val,
        .goal_pressure_m2 = -_linear_trans_func(_t) + _max_val
    };
}

topController::goal_pressure topController::get_pattern(int mode) {
//    topController::goal_pressure next_command = {0.1, 0.1};
//    _t = _t + _step_size;
    last_para_a = 0;
	last_para_b = 0;
	switch(mode){
        case 0 : next_command = static_status(); break;
        case 1 : next_command = rhythmic_movement(); break;
        case 2 : next_command = reaching_movement(); break;
    }
    pre_command = next_command;
    return next_command;
}

topController::goal_pressure topController::get_pattern(int mode, int &parameter) {

    //std::cout << parameter << std::endl;

    switch(mode){
        case 0 : next_command = static_status(); break;
        case 1 : next_command = rhythmic_movement(); break;
        case 2 : next_command = reaching_movement(); break;
    }
    pre_command = next_command;
    return next_command;
}

topController::goal_pressure topController::get_pattern(int mode, double PreAlpha_m1, double PreAlpha_m2) {//名前が同じだが引数が異なる関数を複数定義することをオーバーロードという	_t = 0;
    //std::cout << parameter << std::endl;
    _max_val = PreAlpha_m2;
    _min_val = PreAlpha_m1;

        switch(mode){
        case 0 : next_command = static_status(); break;
        case 1 : next_command = rhythmic_movement(); break;
        case 2 : next_command = reaching_movement(); break;
    }
    pre_command = next_command;
    return next_command;

}

double topController::_sin_trans_func(double t) {
    //t = fmod(t, 2*PI);
    double g = (_max_val - _min_val) / 2 * sin(t) + (_max_val + _min_val) / 2;
    return g;
}
/*
double topController::_cos_trans_func(double t) {
		double g = (_max_val - _min_val) / 2 * cos(t) + (_max_val + _min_val) / 2;
		return g;
}
*/
double topController::_linear_trans_func(double t) {

    if (std::abs(t) > _total_steps) {
        return g_pre;
    }
    g = (_delta_p / _total_steps ) * t;
    g_pre = g;
//    std::cout << g << std::endl;
    return g;
}

int topController::isStart(int mode) {
    if (mode == 1) {
        return _t < _last_t ? 1 : 0;
    } else {
        return (_t > -1.0 && _t < 1.0) ? 1 : 0;
    }
}

int topController :: isStart() {
    return _t < _last_t ? 1 : 0;
}
