//
// Created by Sherry Wang on 2021/6/7.
//

#include "interneuron.h"

Interneuron::Interneuron(Muscle *agonist, Muscle *antagonist) {
    this->agonist_ = agonist;
    this->antagonist_ = antagonist;
    t1 = micros() * 1e-6;
    agonist_len = agonist_->getMuscleState().current_ms_resistance;
    temp_agonist_len = agonist_len;
    agonist_v = 0;
    agonist_tension = agonist_->getMuscleState().current_tension_sensor_feedback;
//
//    antagonist_tension = antagonist_->getMuscleState().current_tension_sensor_feedback;
//    tension_error = 0;
//
//    m_buffer_ten = new RTLoopHistory(WINDOW_LEN, 3000);
    m_buffer_len = new RTLoopHistory(WINDOW_LEN, agonist_len);
    m_buffer_speed = new RTLoopHistory(WINDOW_LEN, agonist_v);
}

Interneuron::Interneuron(Muscle *agonist) {
    this->agonist_ = agonist;
    t1 = micros() * 1e-6;
    agonist_len = agonist_->getMuscleState().current_ms_resistance;
    temp_agonist_len = agonist_len;
    agonist_v = 0;
    agonist_tension = agonist_->getMuscleState().current_tension_sensor_feedback;
    m_buffer_len = new RTLoopHistory(WINDOW_LEN, agonist_len);
    m_buffer_speed = new RTLoopHistory(WINDOW_LEN, agonist_v);
    cnt = 0;
    //timer_ = &timer1;
    //timer_.init_cycle();
    timer_.init_cycle();
    res_SR = 0;
    res_RI = 0;
    isValid = false;
}

Interneuron::~Interneuron() {
    delete agonist_;
    delete antagonist_;
    delete m_buffer_speed;
    delete m_buffer_len;
    //delete timer_;
}

void Interneuron::update_sensor_info() {
    temp_agonist_len = agonist_->getMuscleState().current_ms_resistance;
    t2 = micros() * 1e-6;
    dt = t2 - t1;
    t1 = t2;

    speed_pre_ave = m_buffer_speed->average();
    agonist_v= (temp_agonist_len - agonist_len) / dt;
    m_buffer_speed->sample(agonist_v);

    len_pre_ave = m_buffer_len->average();
    agonist_len = temp_agonist_len;
    m_buffer_len->sample(agonist_len);

    agonist_tension = agonist_->getMuscleState().current_tension_sensor_feedback;

    check_if_stretched();
    cnt++;
}

void Interneuron::check_if_stretched(){
    rate_len = agonist_len - len_pre_ave;
    rate_v = agonist_v - speed_pre_ave;
//    if (cnt >= 400 && cnt <= 410) {
//        rate_len = 600;
//        rate_v = 35000;
//    } else {
//        rate_len = 0;
//        rate_v = 0;
//    }
    --timer_.timer_cycle;
    if (rate_len > 300 && timer_.triggered == false) {
        timer_.init_cycle();
        timer_.triggered = true;
        res_RI = 0;
        res_SR = 0;
        Interneuron::p();
        if (mutex == 0) {
            isValid = true;
        }
    }
    if (timer_.triggered == true && timer_.timer_cycle <= 0) {
        //timer_.init_cycle();
        timer_.triggered = false;
        res_RI = 0;
        res_SR = 0;
        Interneuron::v();
        if (mutex == 1) {
            isValid = false;
        }
    }
}

double Interneuron::stretch_reflex() {
    //double res = 0;
    if (isValid == false) {
        return 0;
    }
    if (timer_.triggered == true && timer_.timer_SR >= 0) {
        --timer_.timer_SR;
       res_SR = std::max(rate_len / 1000 + rate_v / 30000, res_SR);     //rate_len相当于速度，rate_v相当于加速度。
       //res = 1;
	    //std::cout << "v: \t" << rate_len << "\t a: \t" << rate_v << std:: endl;
	   // std::cout << res << std::endl;
    } else if (timer_.triggered == true && timer_.timer_SR < 0) {
        res_SR = 0;
    }
    //std::cout << res << std::endl;
    return res_SR;
}

double Interneuron::reciprocal_inhibition() {
    // double res = 0;
    //if (mutex < 0) {
    if (isValid == false) {
        return 0;
    }
    if (timer_.triggered == true && timer_.timer_RI >= 0) {
        --timer_.timer_RI;
        res_RI = std::max(rate_len / 1000 + rate_v / 30000, res_RI);
	// res = 1;
    } else if (timer_.triggered == true && timer_.timer_RI < 0) {
        res_RI = 0;
    }
    return res_RI;
}
//The relaxation of the antagonist that occurs simultaneously when a muscle spindle’s contraction of its associated muscle occurs is called reciprocal inhibition(相互抑制).
//double Interneuron::autogenic_inhibition() {
//    //update_sensor_info();
//    double stretch_force = 0;
//    stretch_force = (tension_error - tension_pre_ave) / 500; // < 0
//    //stretch_force = tension_buffer.getAverage() - tension_pre_ave;
//    if (force_change_point_detection()) {
//        return stretch_force;
//    } else {
//        return 0;
//    }

//    if (stretch_force > 1000) {
//        return stretch_force / 10000;
//    } else {
//        return 0;
//    }
//}
double Interneuron::get_stretch(){
    return this->agonist_len;
}
double Interneuron::get_stretch_speed(){
    return this->agonist_v;
    //return this->rate_len;
}

uint16_t Interneuron::get_tension() {
    return this->agonist_tension;
    //return this->rate_v;
}
uint64_t Interneuron::micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                                                                        now().time_since_epoch()).count();
    return us;
}

void Interneuron::print_timer() {
    std::cout << cnt << ":\t" << "cycle time: \t" << timer_.timer_cycle << "\t SR: \t" << timer_.timer_SR << "\t RI: "
    "\t" <<
    timer_.timer_RI << "\t trg: \t" << timer_.triggered << std::endl;
}

void Interneuron::p() {
    mutex--;
}

void Interneuron::v() {
    //mutex++;
    mutex = 1;
}

int Interneuron::get_mutex() {
    return mutex;
}
int Interneuron::mutex = 1;