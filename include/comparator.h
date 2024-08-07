//
// Created by Sherry Wang on 2021/08/30.
//

#ifndef CONTROL_BOARD_COMPARATOR_H
#define CONTROL_BOARD_COMPARATOR_H

#include <interneuron.h>
#include <rt_ave.h>

#define PROP 1.0
#define WINDOW_SIZE 5
//#define ACT_TIME 20
#define CYCLE_SHORT 50
#define CYCLE_LONG 100 //每秒检测一次
#define LOCK_TIME 50
#define OSCI_TIME 10
struct cTimer{
    bool is_slept = false;
    int timer_cycle_s = 0;  //刚碰撞时，高频检测
    int timer_cycle_l = 0;  //进入休眠态后，低频检测
    int triggered = 0;
    int phase_shift_counter = 0;    //相位补偿。
    int timer_lock = 0;
    int timer_osci = 0;
    void init_cTimer(void){
        //timer_AGI = ACT_TIME;
        timer_cycle_s = CYCLE_SHORT;        //confirm there is no overshoot of dF.
        timer_cycle_l = CYCLE_LONG;         //ger rid off the influence caused by stretch reflex.
    }
    void init_determinator(){
        timer_osci = OSCI_TIME;
        timer_lock = LOCK_TIME;
    }
    void count_for_osci(){
        if(timer_lock > 0) {
            --timer_lock;
        }
        if (timer_osci > 0) {
            --timer_osci;
        }

    }
    void count_for_update(){
        if (is_slept == true) {
            timer_cycle_l--;
        }else {
            timer_cycle_s--;
        }
    }
};

class Comparator {
public:
    Comparator(Interneuron *neuron_1, Interneuron *neuron_2);
    ~Comparator();
    struct tension_feedback_command{
        double feedback_m1;
        double feedback_m2;
    };
    void update(int &muscle_spindle_reflex_signal);
    void force_change_point_detection(int &mutex);
    tension_feedback_command tension_feedback();
    int get_phase_shift();
    int get_GTO_status();
private:
    double tension_error;
    double tension_error_ave;
    double rate;
    tension_feedback_command feedback_command;
    Interneuron *neuron_1_;
    Interneuron *neuron_2_;
    RTLoopHistory *tension_error_buffer;
    int cnt;
    cTimer ctimer_;
    double res_m1, res_m2;
    //int mutex_;
};


#endif //CONTROL_BOARD_COMPARATOR_H
