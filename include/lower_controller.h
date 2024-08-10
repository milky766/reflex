//
// Created by Sherry Wang on 2021/11/15.
//

#ifndef CONTROL_BOARD_LOWER_CONTROLLER_H  //if not defined then include the following
#define CONTROL_BOARD_LOWER_CONTROLLER_H

#include "muscle.h"
#include "comparator.h"
#include "iir_filter.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#define MS_ACTIVE_TIME 20
#define GTO_ACTIVE_TIME 20
#define MS_CYCLE_TIME 60
#define GTO_CYCLE_TIME 60
#define PROP 1.0


#define SUB_MS_ACTIVE_TIME 20
#define SUB_MS_CYCLE_TIME 60

//#define F_COMPENSATE 3000
#define F_COMPENSATE 0 //重力補償?
#define F_LIMIT 3800

#define Sub_F_LIMIT 2500

#define TensionMapping_Slope 0.848869
#define TensionMapping_Intercept 0.426219

#define Ia_a_cap 260
#define Ia_anta_cap 2500 //threshhold for Ia_anta 調整が必要

#define Ia_a_cap_model 50000
#define Ia_anta_cap_model 50000 //threshhold for Ia_anta 調整が必要

#define sub_Ia_a_cap 150 //co-contraction
#define sub_Ia_anta_cap 1.5

#define G_Compensator_factor 1
#define Harden_Magnitude 0.05

//追加 ひずみゲージからの電気信号を力に変換するためのパラメータ,実験で求めて入力する
#define tension_voltage_slope_agonist 69.76
#define tension_voltage_slope_antagonist 92.85

//追加 パラメータを手動入力する
//1回目の実験、low(0.2-0.6MPa)
#define a_3_agonist -0.20082
#define a_2_agonist 7.001792
#define a_1_agonist 0.256173
#define a_0_agonist 0.91102

#define a_3_antagonist 0.245392
#define a_2_antagonist 0.348521
#define a_1_antagonist -0.03648
#define a_0_antagonist 6.317713

// a_i_agonist_new = a_i_agonist*tension_voltage_slope_agonist

#define a_3_agonist_new -14.01
#define a_2_agonist_new 488.45
#define a_1_agonist_new 17.87
#define a_0_agonist_new 63.55

#define a_3_antagonist_new 22.78
#define a_2_antagonist_new 32.36
#define a_1_antagonist_new -3.39
#define a_0_antagonist_new 586.60

#define  natural_length_slope_agonist -60.1
#define  natural_length_slope_antagonist -70.3
#define  natural_length_intercept_agonist 170.1
#define  natural_length_intercept_antagonist 178.5


class SpinalCord 
{
public:
    SpinalCord(Muscle* agonist, Muscle* antagonist); //宣言はheader fileで、定義はcpp fileで行う
    ~SpinalCord();
    struct GTO_switcher{
        bool triggered = false;
        int timer_reflex = 0;
        int timer_wait = 0;

        void init_var(){
            timer_reflex = GTO_ACTIVE_TIME;
            timer_wait = GTO_CYCLE_TIME;
        }
    };
    struct Timer {
        int timer_SR = 0;
        int timer_cycle = 0;
        bool triggered = false;

        bool subtriggered = false;

        void init_cycle() {
            timer_SR = MS_ACTIVE_TIME;
            timer_cycle = MS_CYCLE_TIME;
        }
        
        //void subinit_cycle(){
        //    timer_SR = SUB_MS_ACTIVE_TIME;
        //    timer_cycle = SUB_MS_CYCLE_TIME;
        //}

    };

    struct Subtimer {
        int subtimer_SR = 0;
        int subtimer_cycle = 0;
        bool triggered = false;

        void init_cycle(){
            subtimer_SR = SUB_MS_ACTIVE_TIME;
            subtimer_cycle = SUB_MS_CYCLE_TIME;
        }
    };

    struct Reflex_feedback //reflexと書いてあるが、実際にはIaの反射のみを扱っている
    {
        double res_SR, res_RI;
        bool isValid;
        void init_feedback()
        {
            res_SR = 0;
            res_RI = 0;
            isValid = false;
            
        }
    };

    struct GTO_feedback{
        double res_AI, res_RE;
        bool isValid;
        void init_feedback(){
            res_AI = 0;
            res_RE = 0;
            isValid = false;
        }
    };

    struct Sensor_info
    {
        double muscle_len, muscle_v, muscle_filtered_v;
        double muscle_tension;
    };

    struct Base_sensor_info
    {
        double base_agonist_len, base_antagonist_len;
        uint16_t base_agonist_tension, base_antagonist_tension;
    };



    // struct Length_Model{
    //     double a3_model, a2_model, a1_model, a0_model;
    // };

    void update_base_sensor_info();
    void update_base_sensor_info_model();


    Base_sensor_info getBaseSensorInfo();
    Base_sensor_info getBaseSensorInfo_model();

    void update_sensor_info(int &parameter_a,int &parameter_b);

    void check_if_stretched(int &parameter_a);
    void check_if_stretched_model(int &parameter_a);
    void check_if_force(int &parameter_b);

    void check_if_Hardern();

    Reflex_feedback agonist_Ia_innervation();
    Reflex_feedback antagonist_Ia_innervation();
    Reflex_feedback agonist_Ia_innervation_model();
    Reflex_feedback antagonist_Ia_innervation_model();

    GTO_feedback agonist_Ib_innervation();
    GTO_feedback antagonist_Ib_innervation();

    Sensor_info get_agonist_sensor_info();
    Sensor_info get_antagonist_sensor_info();
    Sensor_info get_agonist_sensor_info_model();
    Sensor_info get_antagonist_sensor_info_model();

    double calculateLength(int muscle_idx, uint16_t voltage, double pressure);
    double calculateDeformation(int muscle_idx, uint16_t voltage, double pressure);

    uint64_t micros();

    struct MaxTracker
    {
        double a_Ia, anta_Ia, a_Ib, anta_Ib;

        MaxTracker()
        {
            a_Ia = 0;
            anta_Ia = 0;
            a_Ib = 0;
            anta_Ib = 0;
        }

        void init_Ia()
        {
            a_Ia = 0;
            anta_Ia = 0;
        }
        
        void init_Ib()
        {
            a_Ib = 0;
            anta_Ib = 0;
        }


    };

    MaxTracker MaxTracker_;
    MaxTracker MaxTracker_model_; 

    struct Cordinator
    {
        double Alpha1_feedback, Alpha2_feedback;

        Cordinator() 
        {
            Alpha1_feedback = 0;
            Alpha2_feedback = 0;
        }
    };

    Cordinator Cordinator_;

    Cordinator Output_Cordinate(int flag);

    int FeedbackFlag_Ia, FeedbackFlag_Ib;
    int FeedbackFlag_Ia_model, FeedbackFlag_Ib_model;

    double G_Compensation;
    double General_Harden;


private:
    Muscle *agonist_;
    Muscle *antagonist_;
    IIRFilter *agonist_filter_;
    IIRFilter *antagonist_filter_;
    IIRFilter *agonist_filter_model_;
    IIRFilter *antagonist_filter_model_;

    GTO_switcher gtoSwitcher_;
    Timer timer_;

    Subtimer subtimer_;


    Reflex_feedback agonist_feedback_;
    Reflex_feedback antagonist_feedback_;
    Reflex_feedback agonist_feedback_model_;
    Reflex_feedback antagonist_feedback_model_;
    
    GTO_feedback agonist_gto_feedback_;
    GTO_feedback antagonist_gto_feedback_;

    double t1, t2, dt;
    double agonist_len_, temp_agonist_len_, agonist_v_, filtered_agonist_v_;
    double agonist_tension_;
    double antagonist_len_, temp_antagonist_len_, antagonist_v_, filtered_antagonist_v_;
    double antagonist_tension_;
    double relative_tension_;

    double base_agonist_len_, base_antagonist_len_;
    uint16_t base_agonist_tension_, base_antagonist_tension_;
    Base_sensor_info Base_sensor_info_;
    Base_sensor_info Base_sensor_info_model_;

    //追加 tension_model_はtension_と全く同じになるので作らない
    double agonist_len_model_, temp_agonist_len_model_, agonist_v_model_, filtered_agonist_v_model_;
    double antagonist_len_model_, temp_antagonist_len_model_, antagonist_v_model_, filtered_antagonist_v_model_;
    double base_agonist_len_model_, base_antagonist_len_model_;


    double agonist_len_percentage_, antagonist_len_percentage_,agonist_tension_percentage_, antagonist_tension_percentage_;

    

    int mutex_a, mutex_b, mutex_c; //mutex locks a and b for dual reflexes

	bool isAct;
    bool temp_timer;

    

    int flag_;

    

};

#endif //CONTROL_BOARD_LOWER_CONTROLLER_H
