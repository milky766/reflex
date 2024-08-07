//
// Created by Sherry Wang on 2021/6/7.
//

//创建一个internon 类，分别输出两侧的数据。 函数重载？？？
#ifndef CONTROL_BOARD_INTERNEURON_H
#define CONTROL_BOARD_INTERNEURON_H

#include <muscle.h>
#include "../src/lib/circular_buffer.hpp"
#include <rt_ave.h>
#include <math.h>
#include <algorithm>
#define PI acos(-1)
#define WINDOW_LEN 5
#define ACTIVE_TIME 20
#define CYCLE_TIME 60
//活跃时间：30ms，可考虑是否适当延长activate time

//struct inputSignals{
//    double muscle_spindle;
//    uint16_t tension;
//
//};
//引入信号量，控制interneuron的两种状态：1-> activated; 2-> inhibited.
//这就意味着，一个 interneuron 同时输出 stretch reflex unit 和  reciprocal inhibition unit 的输出。另一个 interneuron, 什么都不输出
// I can not control a signal of one object by the other object if there is no relationship between them.
// As a result, an external parameter is needed to change the internal signal.
struct Timer{
    int timer_SR = 0;
    int timer_RI = 0;
    int timer_cycle = 0;
    bool triggered = false;
    void init_cycle(){
        timer_SR = ACTIVE_TIME;
        timer_RI = ACTIVE_TIME;
        timer_cycle = CYCLE_TIME;
        //triggered = true;
    }

};

class Interneuron {
public:
    Interneuron(Muscle* agonist, Muscle* antagonist);
    Interneuron(Muscle* agonist);
    ~Interneuron();

    void update_sensor_info();
    //void initialize_sensor_info();

    double get_stretch_speed();
    double get_stretch();
    uint16_t get_tension();

    void check_if_stretched();
    double stretch_reflex();

    double reciprocal_inhibition();
    //The relaxation of the antagonist that occurs simultaneously when a muscle spindle’s contraction of its associated muscle occurs is called reciprocal inhibition(相互抑制).
    //double autogenic_inhibition();
    //When the GTO is activated during stretching, it inhibits muscle spindle activity within the working muscle (agonist) so a deeper stretch can be achieved.
    uint64_t micros();
//    bool force_change_point_detection();
//    bool force_change_point_detection_inverse();
//    uint16_t tension_error;
//    uint16_t tension_pre_ave = 3000;
    double rate_len, rate_v;
    void print_timer();
    static void p();
    static void v();
    static int get_mutex();
private:

    Muscle *agonist_;
    Muscle *antagonist_;
    Timer timer_;
    //Timer *timer_;

    uint16_t  agonist_tension;
    double agonist_len;
    double agonist_v;
    double temp_agonist_len;
    //double rate_len, rate_v;
    double res_SR, res_RI;
    double t1, t2, dt;
    RTLoopHistory *m_buffer_len;
    RTLoopHistory *m_buffer_speed;
    double len_pre_ave = 0, speed_pre_ave = 0;
    int cnt;
    static int mutex;
    bool isValid;
};
#endif //CONTROL_BOARD_INTERNEURON_H
