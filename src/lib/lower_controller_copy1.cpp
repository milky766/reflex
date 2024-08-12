//
// Created by Sherry Wang on 2021/11/15.
//

#include "lower_controller.h"


SpinalCord::SpinalCord(Muscle* agonist, Muscle* antagonist) {
    agonist_ = agonist;
    antagonist_ = antagonist;
    t1 = micros() * 1e-6;

    agonist_len_ = agonist_->getMuscleState().current_ms_resistance;
    temp_agonist_len_ = agonist_len_;
    agonist_v_ = 0;
    agonist_tension_ = agonist_->getMuscleState().current_tension_sensor_feedback;
    agonist_filter_ = new IIRFilter(0.0);

    antagonist_len_ = antagonist_->getMuscleState().current_ms_resistance;
    temp_antagonist_len_ = antagonist_len_;
    antagonist_v_ = 0;
    antagonist_tension_ = antagonist_->getMuscleState().current_tension_sensor_feedback;
    antagonist_filter_ = new IIRFilter(0.0);

    timer_.init_cycle();

    subtimer_.init_cycle();

    gtoSwitcher_.init_var();


    mutex_a = 1;
	mutex_b = 1;

    General_Harden = 0;

    agonist_feedback_.init_feedback();
    antagonist_feedback_.init_feedback();   
	isAct = false;

    FeedbackFlag_Ia = 0;
    FeedbackFlag_Ib = 0;


}
SpinalCord::~SpinalCord()
{
    delete agonist_;
    delete antagonist_;
    delete agonist_filter_;
    delete antagonist_filter_;
}

void SpinalCord::update_base_sensor_info()
{
    base_agonist_len_ = agonist_->getMuscleState().current_ms_resistance;
    base_antagonist_len_= antagonist_->getMuscleState().current_ms_resistance;
    base_agonist_tension_ = agonist_->getMuscleState().current_tension_sensor_feedback;
    base_antagonist_tension_ = antagonist_->getMuscleState().current_tension_sensor_feedback;
    Base_sensor_info_ = {base_agonist_len_, base_antagonist_len_, base_agonist_tension_, base_antagonist_tension_};
}

void SpinalCord::update_sensor_info(int &parameter_a, int &parameter_b)
{
    temp_agonist_len_ = agonist_->getMuscleState().current_ms_resistance; 
    temp_antagonist_len_ = antagonist_->getMuscleState().current_ms_resistance;
    
    

    t2 = micros() * 1e-6; dt = t2 - t1; t1 = t2;
		
    agonist_v_ = (temp_agonist_len_ - agonist_len_) / dt;
    antagonist_v_ = (temp_antagonist_len_ - antagonist_len_) / dt;

    filtered_agonist_v_= agonist_filter_->sample(agonist_v_);
    filtered_antagonist_v_ = antagonist_filter_->sample(antagonist_v_);

    MaxTracker_.a_Ia = std::max(filtered_agonist_v_/25e4, MaxTracker_.a_Ia);
    MaxTracker_.anta_Ia = std::max(filtered_antagonist_v_/10e5, MaxTracker_.anta_Ia);

    agonist_len_ = temp_agonist_len_; antagonist_len_ = temp_antagonist_len_;

    agonist_tension_ = agonist_->getMuscleState().current_tension_sensor_feedback;
    antagonist_tension_ = antagonist_->getMuscleState().current_tension_sensor_feedback;
    relative_tension_ = agonist_tension_ - ((antagonist_tension_ * TensionMapping_Slope) + TensionMapping_Intercept); 
    G_Compensation = G_Compensator_factor * relative_tension_;


    MaxTracker_.a_Ib = std::max((relative_tension_ - F_LIMIT - F_COMPENSATE) /500, MaxTracker_.a_Ib);
    MaxTracker_.anta_Ib = std::max((F_COMPENSATE - F_LIMIT - relative_tension_) /500, MaxTracker_.anta_Ib);
    
	std::cout << "agonist =" << filtered_agonist_v_ << std::endl;
    std::cout << "agonist cap = " << Ia_a_cap << std::endl;
    std::cout << "antagonist =" << filtered_antagonist_v_ << std::endl;
    std::cout << "antagonist cap = " << Ia_anta_cap << std::endl;
    //std::cout << "MT_a_Ia =" << MaxTracker_.a_Ia << std::endl;
    //std::cout << "MT_anta_Ia =" << MaxTracker_.anta_Ia << std::endl; 
    
    std::cout << "a_Ib cap = > " << (F_LIMIT + F_COMPENSATE) << std::endl;

    std::cout << "relative_tension =" << relative_tension_ << std::endl;

    std::cout << "anta_Ib cap = < " << (F_COMPENSATE - F_LIMIT) << std::endl;
    
    std::cout << "a_tension =" << agonist_tension_ << std::endl;
    std::cout << "anta_tension =" << antagonist_tension_ << std::endl;
    //std::cout << "MT_a_Ib =" << MaxTracker_.a_Ib << std::endl;
    //std::cout << "MT_anta_Ib =" << MaxTracker_.anta_Ib << std::endl;
    	
    //check_if_Hardern();
    check_if_stretched(parameter_a);
    check_if_force(parameter_b);
}


void SpinalCord::check_if_Hardern()
{
    --subtimer_.subtimer_cycle;
    if(mutex_a < 1 || mutex_b <1){
        return;
    }
    else {
        if (filtered_agonist_v_ < Ia_a_cap && filtered_agonist_v_ > sub_Ia_a_cap && subtimer_.triggered == false){
            subtimer_.init_cycle();
            subtimer_.triggered = true;

            

            
            agonist_feedback_.isValid = true;
            General_Harden = Harden_Magnitude;
        }
        else if (filtered_antagonist_v_ < Ia_anta_cap && filtered_antagonist_v_ > sub_Ia_anta_cap && subtimer_.triggered == false){
            subtimer_.init_cycle();
            subtimer_.triggered = true;

            

            
            antagonist_feedback_.isValid = true;
            General_Harden = Harden_Magnitude;
        }

        if (subtimer_.triggered == true && subtimer_.subtimer_cycle <= 0){

            subtimer_.triggered = false;
            
            agonist_feedback_.init_feedback();
            antagonist_feedback_.init_feedback();
        }
    }
}

void SpinalCord::check_if_stretched(int &parameter_a)
{
    --timer_.timer_cycle;
    //--subtimer_.subtimer_cycle;
    // if(mutex_b < 1) {   //mutex lock
    //     return;
    // } 
    
    else {

        if (filtered_agonist_v_ > Ia_a_cap && timer_.triggered == false){
                std::cout << "left stretched" << std::endl;

        timer_.init_cycle();
        timer_.triggered = true;

        subtimer_.triggered = false;

        mutex_a = 0;
       
        agonist_feedback_.isValid = true;
        FeedbackFlag_Ia = 1;
        } 


        //----------------------------------------------------------------
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // else if (filtered_agonist_v_ < Ia_a_cap && filtered_agonist_v_ > sub_Ia_a_cap && subtimer_.triggered == false){
        
        //    subtimer_.init_cycle();
        //     timer_.subtriggered = true;
        //    subtimer_.triggered = true;

        //    timer_.triggered = false;

        //    mutex_a = 0;
        //    agonist_feedback_.isValid = true;
        //    FeedbackFlag_Ia = 1;
        //    General_Harden = Harden_Magnitude;

        // }
        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        //----------------------------------------------------------------


        else if (filtered_antagonist_v_ > Ia_anta_cap && timer_.triggered == false) {
				std::cout << "right stretched" << std::endl;

            timer_.init_cycle();
            timer_.triggered = true;

            subtimer_.triggered = false;

            mutex_a = 0;

            antagonist_feedback_.isValid = true;
            FeedbackFlag_Ia = 1;

        }


        //----------------------------------------------------------------
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        //else if (filtered_antagonist_v_ < Ia_anta_cap && filtered_antagonist_v_ > sub_Ia_anta_cap && subtimer_.triggered == false){

        //    subtimer_.init_cycle();
            //timer_.subtriggered = true;
        //    subtimer_.triggered = true;

        //    timer_.triggered = false;


        //    mutex_a = 0;
        //    antagonist_feedback_.isValid = true;
        //    FeedbackFlag_Ia = 1;
        //    General_Harden = Harden_Magnitude;
        //}
        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        //----------------------------------------------------------------



		//if (timer_.triggered == true && timer_.triggered == false) {
		//	++parameter_a;
		//}

        if (timer_.triggered == true && timer_.timer_cycle <= 0) {
		    		std::cout << "stretch response end" << std::endl;
            timer_.triggered = false;

            //timer_.subtriggered = false;
            subtimer_.triggered = false;

            

            agonist_feedback_.init_feedback();
            antagonist_feedback_.init_feedback();
            MaxTracker_.init_Ia();

            mutex_a = 1;
            FeedbackFlag_Ia = 0;

            //General_Harden = 0;

		    //para = 1 means Ia reflex end
            parameter_a += 1;

        }

        //if (subtimer_.triggered == true && subtimer_.subtimer_cycle <= 0) {
        //            std::cout << "stretch response end" << std::endl;
        //    timer_.triggered = false;

            //timer_.subtriggered = false;
        //    subtimer_.triggered = false;

            


        //    agonist_feedback_.init_feedback();
        //    antagonist_feedback_.init_feedback();
            

        //    mutex_a = 1;
        //    FeedbackFlag_Ia = 0;

            //General_Harden = 0;

            //para = 1 means Ia reflex end
        //    parameter_a += 1;

        //}


    }
	//	std::cout << agonist_feedback_.isValid << " " << antagonist_feedback_.isValid << std::endl; 
}

    void SpinalCord::check_if_force(int &parameter_b)
    {
        --gtoSwitcher_.timer_wait;
        // if (mutex_a < 1) {
        //     return; 
        //     } //mutex lock


        else if (relative_tension_ > F_LIMIT + F_COMPENSATE && gtoSwitcher_.triggered == false) {
                    std::cout << "left force" << std::endl;
            gtoSwitcher_.triggered = true;
            gtoSwitcher_.init_var();

            mutex_b --;

            FeedbackFlag_Ib = 1;
            
            agonist_gto_feedback_.isValid = true;
            } 
        else if (relative_tension_ < F_COMPENSATE - F_LIMIT && gtoSwitcher_.triggered == false) {
                    std::cout << "right force" << std::endl;
            gtoSwitcher_.triggered = true;
            gtoSwitcher_.init_var();

            mutex_b --;

            FeedbackFlag_Ib = 1;

            antagonist_gto_feedback_.isValid = true;

        }   
        if (gtoSwitcher_.triggered == true && gtoSwitcher_.timer_wait <= 0) {
                    std::cout << "force response end" << std::endl;
            gtoSwitcher_.triggered = false;
            agonist_gto_feedback_.init_feedback();
            antagonist_gto_feedback_.init_feedback();

            mutex_b = 1;

            FeedbackFlag_Ib = 0;

            MaxTracker_.init_Ib();
            //para = 2 refers to Ib ref lex end		
            parameter_b += 1;
                    //std::cout << parameter << std::endl;
        }

    }

    SpinalCord::Reflex_feedback SpinalCord::agonist_Ia_innervation()
    {
        if (agonist_feedback_.isValid == false) {
            return {.res_SR = 0,
                    .res_RI = 0};
        }
        //double res_SR_ = 0, res_RI = 0;
        if (timer_.triggered == true && timer_.timer_SR >= 0)
        {
            --timer_.timer_SR;
            //agonist_feedback_.res_SR = std::max(filtered_agonist_v_/80000, agonist_feedback_.res_SR);
            //agonist_feedback_.res_RI = std::max(filtered_agonist_v_/80000, agonist_feedback_.res_RI);
        
            agonist_feedback_.res_SR = MaxTracker_.a_Ia;
            agonist_feedback_.res_RI = MaxTracker_.a_Ia;
                    //std::cout << "agonist_feedback: " << agonist_feedback_.res_SR << ", when filtered_agonist_v_ is: " << filtered_agonist_v_ << std::endl; 
        } 
        
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        if (subtimer_.triggered == true && subtimer_.subtimer_SR >= 0){

            --subtimer_.subtimer_SR;

            agonist_feedback_.res_SR = General_Harden;
            agonist_feedback_.res_RI = General_Harden;

        }
        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        
        
        else if (timer_.triggered == true && timer_.timer_SR < 0) {
            agonist_feedback_.res_SR = 0;
            agonist_feedback_.res_RI = 0;
        }

        else if (subtimer_.triggered == true && subtimer_.subtimer_SR < 0) {
            antagonist_feedback_.res_SR = 0;
            antagonist_feedback_.res_RI = 0;
        }

        return agonist_feedback_;
    }

    SpinalCord::Reflex_feedback SpinalCord::antagonist_Ia_innervation()
    {
        if (antagonist_feedback_.isValid == false) {
            return {.res_SR = 0,
                    .res_RI = 0};
        }
        if (timer_.triggered == true && timer_.timer_SR >= 0)
        {
            --timer_.timer_SR;
            antagonist_feedback_.res_SR = MaxTracker_.anta_Ia;
            antagonist_feedback_.res_RI = MaxTracker_.anta_Ia;
                    
                    //std::cout << "antagonist_feedback: " << antagonist_feedback_.res_SR << ", when filtered_antagonist_v_ is: "<< filtered_antagonist_v_ << std::endl; 

        } 
        
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        else if (subtimer_.triggered == true && subtimer_.subtimer_SR >= 0){

            --subtimer_.subtimer_SR;

            antagonist_feedback_.res_SR = General_Harden;
            antagonist_feedback_.res_RI = General_Harden;

        }
        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        
        
        
        else if (timer_.triggered == true && timer_.timer_SR < 0) {
            antagonist_feedback_.res_SR = 0;
            antagonist_feedback_.res_RI = 0;
        }

        else if (subtimer_.triggered == true && subtimer_.subtimer_SR < 0) {
            antagonist_feedback_.res_SR = 0;
            antagonist_feedback_.res_RI = 0;
        }
        
        return antagonist_feedback_;
    }

    SpinalCord::GTO_feedback SpinalCord::agonist_Ib_innervation() {
    // if (agonist_gto_feedback_.isValid == false || !isAct ){
        if (agonist_gto_feedback_.isValid == false){
            return{.res_AI = 0.0, .res_RE = 0.0};
            
        }
        if (gtoSwitcher_.triggered == true && gtoSwitcher_.timer_reflex >= 0) {
            --gtoSwitcher_.timer_reflex;
            agonist_gto_feedback_.res_AI = MaxTracker_.a_Ib;
            agonist_gto_feedback_.res_RE = MaxTracker_.a_Ib;
                    //agonist_gto_feedback_.res_AI = 0.15;
                    //agonist_gto_feedback_.res_RE = 0.15;

                    //std::cout << "agonist_gto_feedback: " << agonist_gto_feedback_.res_AI  << ", when relative tension is: " << relative_tension_ << std::endl;
        }
        //在timer_reflex 和 timer_wait 之间，可以引入主动调节量
    //    else if (gtoSwitcher_.triggered == true && gtoSwitcher_.timer_wait < 0) {
    //        agonist_gto_feedback_.res_AI = 0.0;
    //        agonist_gto_feedback_.res_RE = 0.0;
    //    }
        return agonist_gto_feedback_;
    }

    SpinalCord::GTO_feedback SpinalCord::antagonist_Ib_innervation() {
        // if (antagonist_gto_feedback_.isValid == false || !isAct ) {
        if (antagonist_gto_feedback_.isValid == false) {
            return{.res_AI = 0.0, .res_RE = 0.0};
        }
        if (gtoSwitcher_.triggered == true && gtoSwitcher_.timer_reflex >= 0) {
            --gtoSwitcher_.timer_reflex;
            antagonist_gto_feedback_.res_AI = MaxTracker_.anta_Ib;
            antagonist_gto_feedback_.res_RE = MaxTracker_.anta_Ib;

                    //antagonist_gto_feedback_.res_AI = 0;
                    //antagonist_gto_feedback_.res_RE = 0;
    //				std::cout << "antagonist_gto_feedback: " << antagonist_gto_feedback_.res_AI  << ", when relative tension is: " << relative_tension_ << std::endl;
        }
        return antagonist_gto_feedback_;
    }

//double SpinalCord::reciprocal_inhibition_response();
SpinalCord::Sensor_info SpinalCord::get_agonist_sensor_info()
{
    return {.muscle_len = agonist_len_, .muscle_v = agonist_v_,
            .muscle_filtered_v = filtered_agonist_v_, .muscle_tension = agonist_tension_};
}
SpinalCord::Sensor_info SpinalCord::get_antagonist_sensor_info() {
    return {.muscle_len = antagonist_len_, .muscle_v = antagonist_v_,
            .muscle_filtered_v = filtered_antagonist_v_, .muscle_tension = antagonist_tension_};
}

uint64_t SpinalCord::micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
            now().time_since_epoch()).count();
    return us;
}


Base_sensor_info getBaseSensorInfo(){
    return Base_sensor_info_;
}
