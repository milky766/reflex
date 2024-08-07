//
// Created by Sherry Wang on 2021/11/15.
//

#include <chrono>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>

#include <control_board.h>
#include <lower_controller.h>
#include <muscle.h>
#include <rate_loop.h>
#include <top_controller.h>


#define PI acos(-1)
#define STATIC 0
#define RHYTHMIC 1
#define REACHING 2
#define NOREFLEX 0
#define IA 1
#define IB 2
#define IAIB 3



static ControlBoard control_board;

[[noreturn]] void *controlLoop(void *)
{
    //Neutral Position of Valve is at 5V. So clamps are set to -4.5V and 4.5V for effective control voltages of 0.5 to 9.5V
    Muscle::pid_cfg_t pid_conf = {.p = 30.0, .i = 20.0, .d = 0.01, .lower_clamp = -4.5, .upper_clamp = 4.5};
    Muscle::pid_cfg_t pid_conf2 = {.p = 30.0, .i = 5.0, .d = 0.01, .lower_clamp = -4.5, .upper_clamp = 4.5};

    //Muscles are numbered by their order of labels in the setup. Indices are based on the hardware's channel numbers
    Muscle::muscle_cfg_t muscle_conf_0 = {.adc_index = 0, .dac_index = 8, .tension_sensor_index = 0, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_1 = {.adc_index = 1, .dac_index = 9, .tension_sensor_index = 1, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_2 = {.adc_index = 2, .dac_index = 10, .tension_sensor_index = 2, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_3 = {.adc_index = 3, .dac_index = 11, .tension_sensor_index = 3, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_4 = {.adc_index = 4, .dac_index = 12, .tension_sensor_index = 4, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_5 = {.adc_index = 5, .dac_index = 13, .tension_sensor_index = 0, .muscle_spindle_index = 8,
            .pid_cfg = pid_conf2, .board = &control_board,.mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_6 = {.adc_index = 6, .dac_index = 14, .tension_sensor_index = 6, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_7 = {.adc_index = 7, .dac_index = 15, .tension_sensor_index = 2, .muscle_spindle_index = 9,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    uint8_t potentiometer_idx = 12;

    // Using auto to avoid repeating Classname
    auto *muscle_0 = new Muscle(muscle_conf_0);
    auto *muscle_1 = new Muscle(muscle_conf_1);
    auto *muscle_2 = new Muscle(muscle_conf_2);
    auto *muscle_3 = new Muscle(muscle_conf_3);
    auto *muscle_4 = new Muscle(muscle_conf_4);
    auto *muscle_5 = new Muscle(muscle_conf_5);
    auto *muscle_6 = new Muscle(muscle_conf_6);
    auto *muscle_7 = new Muscle(muscle_conf_7);

    // Muscles' memory should be freed properly, but currently we don't catch any signals.
    // The program ends and we rely on the OS to clean up our mess.
    // Shouldn't be a real problem at the end of a program but it is not a good style.
    // All memory allocated on the heap should be freed at one point.
    Muscle::muscle_cmd_t m_cmd_0 = {.control_mode = Muscle::ControlMode::activation, .goal_pressure = 0.0, .goal_activation = 0.0};
    Muscle::muscle_state_t s_0 = muscle_0->updateMuscle(m_cmd_0);
    Muscle::muscle_cmd_t m_cmd_1 = {.control_mode = Muscle::ControlMode::activation, .goal_pressure = 0.0, .goal_activation = 0.0};
    Muscle::muscle_state_t s_1 = muscle_1->updateMuscle(m_cmd_1);
    Muscle::muscle_cmd_t m_cmd_2 = {.control_mode = Muscle::ControlMode::activation, .goal_pressure = 0.0, .goal_activation = 0.0};
    Muscle::muscle_state_t s_2 = muscle_2->updateMuscle(m_cmd_2);
    Muscle::muscle_cmd_t m_cmd_3 = {.control_mode = Muscle::ControlMode::activation, .goal_pressure = 0.0, .goal_activation = 0.0};
    Muscle::muscle_state_t s_3 = muscle_3->updateMuscle(m_cmd_3);
    Muscle::muscle_cmd_t m_cmd_4 = {.control_mode = Muscle::ControlMode::activation, .goal_pressure = 0.0, .goal_activation = 0.0};
    Muscle::muscle_state_t s_4 = muscle_4->updateMuscle(m_cmd_4);
    Muscle::muscle_cmd_t m_cmd_6 = {.control_mode = Muscle::ControlMode::activation, .goal_pressure = 0.0, .goal_activation = 0.0};
    Muscle::muscle_state_t s_6 = muscle_6->updateMuscle(m_cmd_6);
//change mode : 1
    auto *top_con = new topController(0.25);
    //auto *top_con = new topController(0.25, 0.25, 3, 100);
    //auto *top_con = new topController(0.40, 0.15, 3, 100);
    //auto *top_con = new topController(0, 0, 0.40, 0.15, 10.0, 100);
    //topController top_con(0.40, 0.15, 3, HZ);
    //topController top_con(0.3, 0.3, 3, HZ);
    topController::goal_pressure higher_commands = {0.1, 0.1};
    SpinalCord *low_con = new SpinalCord(muscle_5, muscle_7);
    SpinalCord::Reflex_feedback reflex_commands_agonist = {0, 0, false};
    SpinalCord::Reflex_feedback reflex_commands_antagonist = {0, 0, false};
    SpinalCord::GTO_feedback reflex_gto_agonist = {0, 0, false};
    SpinalCord::GTO_feedback reflex_gto_antagonist = {0,0, false};
    SpinalCord::Sensor_info sensor_info_agonist;
    SpinalCord::Sensor_info sensor_info_antagonist;

    /*
    static SpinalCord::MaxTracker MaxTracker_;
    MaxTracker_.a_Ia = 1;
    */



    //double curr_tension_ = 0.0, ave_tension_ = 0.0;


    
    //store the change of sensors
    uint16_t *tension_data_left = new uint16_t[100000];
    uint16_t *tension_data_right = new uint16_t[100000];
    double *pressure_data_left = new double[100000];
    double *pressure_data_right = new double[100000];
    double *ms_data_left = new double[100000];
    double *ms_data_right = new double[100000];
    double *ms_data_left_speed = new double[100000];
    double *ms_data_right_speed = new double[100000];
    double *g_pressure_1 = new double[100000];
    double *g_pressure_2 = new double[100000];
    double *ago_SR = new double[100000];
    double *ago_RI = new double[100000];
    double *ago_AI = new double[100000];
    double *ago_RE = new double[100000];
    double *antago_SR = new double[100000];
    double *antago_RI = new double[100000];
    double *antago_AI = new double[100000];
    double *antago_RE = new double[100000];

    int *marker = new int[100000];
    int *angle = new int[100000];

    int sample_count = 0;
    double alpha_m1, alpha_m2;

    double pre_alpha_m1, pre_alpha_m2;

    int muscle_spindle_reflex_signal = 0;
		
	RateLoop *initiator = new RateLoop(100, 2.0);
	while (initiator->Sleep())
	{
		control_board.update_inputs();
    muscle_5->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.25, .goal_activation = 0.0});
    muscle_7->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.25, .goal_activation = 0.0});
	}
    //muscle_7->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.40, .goal_activation = 0.0});
    //std::cout << "start" << std:: endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //usleep(10000);
    //std::cout << "end" << std:: endl;
    
    static int para_intervene_a = 0;
	static int para_intervene_b = 0; 
    
    RateLoop *rater = new RateLoop(100, 14.0);
    //frequency of while loop will be 100 HZ, total experiment time is 12 seconds

    while(true)
    {
        control_board.update_inputs();
        low_con->update_sensor_info(para_intervene_a,para_intervene_b);

				std::cout << "******para_intervene_a = " << para_intervene_a <<"*****para_intervene_b =  " << para_intervene_b << " ***********" << std::endl;


//change mode: 2
		switch(2)
        {
            case 0: 
            {
                higher_commands = top_con->get_pattern(REACHING);
                break;
            }
            case 1:
            {
                if(para_intervene_a == 0)
                    higher_commands = top_con->get_pattern(STATIC);
                else
                    higher_commands = top_con->get_pattern(STATIC, para_intervene_a);
                break;
            }
            case 2:
            {
                if(para_intervene_b == 0)
                    higher_commands = top_con->get_pattern(REACHING);
                else
                    higher_commands = top_con->get_pattern(REACHING, para_intervene_b, alpha_m1, alpha_m2);
                break;
            }
        }	
		//higher_commands = top_con->get_pattern(RHYTHMIC, para_intervene, alpha_m1, alpha_m2);
        
        reflex_commands_agonist = low_con->agonist_Ia_innervation();
        reflex_commands_antagonist = low_con->antagonist_Ia_innervation();
        reflex_gto_agonist = low_con->agonist_Ib_innervation();
        reflex_gto_antagonist = low_con->antagonist_Ib_innervation();
        
        /*
        std::cout << "Muscle 5 reflex command" << std::endl;
        std::cout <<"Ia homo exci =" << reflex_commands_agonist.res_SR  << "  Ia anta inhi =" << reflex_commands_antagonist.res_RI 
                  <<"   Ib homo inhi =" << reflex_gto_agonist.res_AI    << "  Ib anta exci =" << reflex_gto_antagonist.res_RE <<std::endl;
        std::cout << "Muscle 7 reflex command" << std::endl;
        std::cout <<"Ia homo exci =" << reflex_commands_antagonist.res_SR  << "  Ia anta inhi =" << reflex_commands_agonist.res_RI 
                  <<"   Ib homo inhi =" << reflex_gto_antagonist.res_AI    << "  Ib anta exci =" << reflex_gto_agonist.res_RE <<std::endl;
        */
        switch(3)
        {
            case 0: 
            {
                alpha_m1 = higher_commands.goal_pressure_m1;
                alpha_m2 = higher_commands.goal_pressure_m2;
                break;
            }
            case 1:
            {
                alpha_m1 = higher_commands.goal_pressure_m1 + reflex_commands_agonist.res_SR - reflex_commands_antagonist.res_RI;
                alpha_m2 = higher_commands.goal_pressure_m2 + reflex_commands_antagonist.res_SR - reflex_commands_agonist.res_RI;
                break;
            }
            case 2:
            {
                alpha_m1 = higher_commands.goal_pressure_m1 - reflex_gto_agonist.res_AI + reflex_gto_antagonist.res_RE;
                alpha_m2 = higher_commands.goal_pressure_m2 - reflex_gto_antagonist.res_AI + reflex_gto_agonist.res_RE;
                break;
            }
            case 3:
            {
                alpha_m1 = higher_commands.goal_pressure_m1 + reflex_commands_agonist.res_SR - reflex_commands_antagonist.res_RI - reflex_gto_agonist.res_AI + reflex_gto_antagonist.res_RE;
                alpha_m2 = higher_commands.goal_pressure_m2 + reflex_commands_antagonist.res_SR - reflex_commands_agonist.res_RI - reflex_gto_antagonist.res_AI + reflex_gto_agonist.res_RE;
                break;
            }
        }
        
		if (alpha_m1 > 0.5) {
            alpha_m1 = 0.5;
        } else if (alpha_m1 < 0.02) {
            alpha_m1 = 0.02;
        }
       
		if (alpha_m2 > 0.5) {
            alpha_m2 = 0.5;
        } else if (alpha_m2 < 0.02) {
            alpha_m2 = 0.02;
        }
       // std::cout << "left:\t" << higher_commands.goal_pressure_m1 << "\tright:\t" << higher_commands.goal_pressure_m2 << std::endl;

        Muscle::muscle_cmd_t m_cmd_5 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = alpha_m1,
                .goal_activation = 0.0};
        Muscle::muscle_state_t s_5 = muscle_5->updateMuscle(m_cmd_5);
        Muscle::muscle_state_t p_5 = muscle_5->printMuscle(m_cmd_5);
        Muscle::muscle_cmd_t m_cmd_7 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = alpha_m2,
                .goal_activation = 0.0};
        Muscle::muscle_state_t s_7 = muscle_7->updateMuscle(m_cmd_7);
        Muscle::muscle_state_t p_7 = muscle_7->printMuscle(m_cmd_7);

        g_pressure_1[sample_count] = alpha_m1;
        g_pressure_2[sample_count] = alpha_m2;
        
// change mode: 3
        marker[sample_count] = top_con->isStart(REACHING);
        //marker[sample_count] = top_con->isStart();
        sensor_info_agonist = low_con->get_agonist_sensor_info();
        sensor_info_antagonist = low_con->get_antagonist_sensor_info();
        //记录数据：
        tension_data_left[sample_count] = sensor_info_agonist.muscle_tension;
        ms_data_left[sample_count] = sensor_info_agonist.muscle_len;
        ms_data_left_speed[sample_count] = sensor_info_agonist.muscle_filtered_v;
        pressure_data_left[sample_count] = s_5.current_pressure;

        tension_data_right[sample_count] = sensor_info_antagonist.muscle_tension;
        ms_data_right[sample_count] = sensor_info_antagonist.muscle_len;
        ms_data_right_speed[sample_count] = sensor_info_antagonist.muscle_filtered_v;
        pressure_data_right[sample_count] = s_7.current_pressure;

        angle[sample_count] = control_board.getPotentiometerData(potentiometer_idx);

        ago_SR[sample_count] = reflex_commands_agonist.res_SR;
        ago_RI[sample_count] = reflex_commands_agonist.res_RI;
        ago_AI[sample_count] = reflex_gto_agonist.res_AI;
        ago_RE[sample_count] = reflex_gto_agonist.res_RE;
        antago_SR[sample_count] = reflex_commands_antagonist.res_SR;
        antago_RI[sample_count] = reflex_commands_antagonist.res_RI;
        antago_AI[sample_count] = reflex_gto_antagonist.res_AI;
        antago_RE[sample_count] = reflex_gto_antagonist.res_RE;

        sample_count++;
        if(!rater->Sleep()) {
            break;
        }
//        usleep(10000); //10000 us = 10 ms -> 100 Hz
    }

    muscle_5->updateMuscle({.control_mode = Muscle::ControlMode::pressure,
                            .goal_pressure = 0.2, .goal_activation = 0.0});
    muscle_7->updateMuscle({.control_mode = Muscle::ControlMode::pressure,
                            .goal_pressure = 0.2, .goal_activation = 0.0});

        control_board.update_inputs();
    char filename[1000];
    sprintf(filename, "../data/feedback_test/20221114/DualReflex_%lld.txt", rater->TimeStamp());
    std::cout << "prepare to write data to file" << std::endl;
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);
    file << "count\t"
         << "tension_left\t" << "ms_left\t" << "ms_speed_left\t" << "pressure_left\t" << "g_pressure_1\t"
         << "tension_right\t" << "ms_right\t" << "ms_speed_right\t" << "pressure_right\t" <<"g_pressure_2\t"
         << "start of cycle\t" << "angle\t" 
         << "m5_Ia_homo+\t" << "m5_Ia_anta-\t" << "m5_Ib_homo-\t" << "m5_Ib_anta+\t" 
         << "m7_Ia_homo+\t" << "m7_Ia_anta-\t" << "m7_Ib_homo-\t" << "m7_Ib_anta+"   << std::endl;
    for (int i = 0; i <= sample_count; i++)
    {
        file << i << "\t" << tension_data_left[i] << "\t" << ms_data_left[i] << "\t" << ms_data_left_speed[i] << "\t"
             << pressure_data_left[i] << "\t" << g_pressure_1[i] << "\t"
             << tension_data_right[i] << "\t" << ms_data_right[i] << "\t" << ms_data_right_speed[i] << "\t"
             << pressure_data_right[i] << "\t" << g_pressure_2[i] << "\t" << marker[i] << "\t" << angle[i] << "\t" 
             << ago_SR[i] << "\t" << ago_RI[i] << "\t" << ago_AI[i] << "\t" << ago_RE[i] << "\t" 
             << antago_SR[i] << "\t" << antago_RI[i] << "\t" << antago_AI[i] << "\t" << antago_RE[i]
             << std::endl;
    }
    file.close();
    std::cout << "write finished"<< std::endl;




    delete[] tension_data_left;
    delete[] tension_data_right;
    delete[] pressure_data_left;
    delete[] pressure_data_right;
    delete[] ms_data_left;
    delete[] ms_data_right;
    delete[] ms_data_left_speed;
    delete[] ms_data_right_speed;
    delete[] g_pressure_1;
    delete[] g_pressure_2;
    delete[] marker;
    delete[] angle;
    delete[] ago_AI;
    delete[] ago_RE;
    delete[] ago_RI;
    delete[] ago_SR;
    delete[] antago_AI;
    delete[] antago_RE;
    delete[] antago_RI;
    delete[] antago_SR;

    //delete[] MT_a_Ib;
    //delete[] MT_anta_Ib;

    delete rater;
    delete top_con;
    delete low_con;
		delete initiator;
    pthread_exit(NULL);
}



//The Main boilerplate code to set the controlLoop up to be executed with a high priority.
//Noting should be needed to be changed here.
int main(int argc, char **argv)
{

    // Keep the kernel from swapping us out
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        printf("mlockall failed\n");
        exit(EXIT_FAILURE);
    }

    //Start thread

    // Set real-time scheduler parameters
    pthread_t controlThread;
    pthread_attr_t controlThreadAttr;
    struct sched_param thread_param;
    int ret;

    ret = pthread_attr_init(&controlThreadAttr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        exit(EXIT_FAILURE);
    }

    ret = pthread_attr_setschedpolicy(&controlThreadAttr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        exit(EXIT_FAILURE);
    }

    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    ret = pthread_attr_setschedparam(&controlThreadAttr, &thread_param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        exit(EXIT_FAILURE);
    }

    ret = pthread_attr_setinheritsched(&controlThreadAttr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        exit(EXIT_FAILURE);
    }

    // Start control thread
    ret = pthread_create(&controlThread, &controlThreadAttr, controlLoop, NULL);
    if (ret)
    {
        printf("Unable to create control thread: rv = %d", ret);
        exit(EXIT_FAILURE);
    }

    ret = pthread_join(controlThread, NULL);
    if (ret)
    {
        printf("join pthread failed\n");
    }


}
