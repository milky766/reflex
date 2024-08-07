//
// Created by Sherry Wang on 2021/08/30.
//
#include <comparator.h>

Comparator::Comparator(Interneuron *neuron_1, Interneuron *neuron_2) {
    this->neuron_1_ = neuron_1;
    this->neuron_2_ = neuron_2;
    tension_error_buffer = new RTLoopHistory(WINDOW_SIZE, 0);
    cnt = 0;
    res_m1 = 0.0;
    res_m2 = 0.0;
    ctimer_.init_cTimer();
}

Comparator::~Comparator() {
    delete neuron_1_;
    delete neuron_2_;
    delete tension_error_buffer;
}

void Comparator::update(int &muscle_spindle_reflex_signal){
    //mutex_ = muscle_spindle_reflex_signal;
    tension_error_ave = tension_error_buffer->average();
    tension_error = neuron_1_->get_tension() - PROP * neuron_2_->get_tension();
    tension_error_buffer->sample(tension_error);
    cnt++;

    force_change_point_detection(muscle_spindle_reflex_signal);

}

void Comparator::force_change_point_detection(int &mutex) {
//    if (cnt == 100) {
//        rate = 0.2;
//    }else if (cnt == 128){
//        rate = 0;
//    }else if (cnt == 160) {
//        rate = -0.1;
//    } else if (cnt == 168) {
//        rate = 0;
//    }

    ctimer_.count_for_osci();
    rate = (tension_error - tension_error_ave) / 3000;
    if (ctimer_.timer_osci == 0 && ctimer_.timer_lock == 0 && mutex == 1) {

        if (rate > 0.05) {                   //force towards to right
            if (ctimer_.triggered < 1) {    //when triggered = 0 or -1
                ctimer_.triggered += 1;     //triggered then = 1 or 0
                std::cout << "trg" << std::endl;
                //if (ctimer_.triggered > 0) { // surpass 0 at the first time. which means it is activated from static
                    // status.
                    ctimer_.init_determinator();    //to omit the oscillation during a moment.
                //}
            }
        }else if (rate < -0.05) {                  //force towards to left
            if (ctimer_.triggered > -1) {
                ctimer_.triggered -= 1;
                std::cout << "release" << std::endl;
                //if (ctimer_.triggered < 0) {
                    ctimer_.init_determinator();
                //}
            }
        }

        // ctimer_.timer_cycle_l--;
        // ctimer_.timer_osci--;
    }
    //std::cout<< "rate: \t" << rate << "\tlocker: \t" << ctimer_.timer_lock  << "\tosci: \t" << ctimer_.timer_osci <<std::endl;

}

Comparator::tension_feedback_command Comparator::tension_feedback(){
//    if (force_change_point_detection()) {
//        //std::cout << "1" << std::endl;
//        return {.feedback_m1 = rate * 0.2, .feedback_m2 = -rate * 0.2};
//    } else {
//        //std::cout << "0" << std::endl;
//        return {.feedback_m1 = 0, .feedback_m2 = 0};
//    }
    if (ctimer_.timer_osci > 0 && ctimer_.triggered == 0) {
        ctimer_.timer_lock = LOCK_TIME;
    }
    if (ctimer_.timer_lock > 0) {
        ctimer_.timer_lock--;
        res_m1 = 0;
        res_m2 = 0;
    }

//    if (ctimer_.triggered != 0 && ctimer_.timer_AGI >= 0) {
//        ctimer_.timer_AGI--;
//    }
    return {.feedback_m1 = res_m1, .feedback_m2 = res_m2};
}

// I think there should be a time offset. keep record of the state where it stops
// there also will be an updater, detect if the small tension diff still exists, or if there is an external
// tension release behavior.     
// the stretch reflex will not be triggered by the hit experiments. check the threshold still work?  
// it need a timer.

int Comparator::get_phase_shift() {
    return ctimer_.phase_shift_counter;
}

int Comparator::get_GTO_status(){
    return ctimer_.triggered;
}