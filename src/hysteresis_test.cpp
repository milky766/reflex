//
// Created by Sherry Wang on 2021/5/10.
//

#include <cstdio>
#include <csignal>
#include <ctime>
#include <sys/mman.h>
#include <sys/types.h>
#include <pthread.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <fstream>

#include <control_board.h>
#include <muscle.h>

double getAverage(uint16_t *_inputArray[])
{

    uint16_t sum = 0;
    int num = 0;

    for (int i = 50; _inputArray[i]!= 0 ; i++)
    {
       // sum += _inputArray[i];
        num ++;
    }
    return double(sum/num);
}

static ControlBoard control_board;

//Function for control code
[[noreturn]] void *controlLoop(void *)
{
    //Neutral Position of Valve is at 5V. So clamps are set to -4.5V and 4.5V for effective control voltages of 0.5 to 9.5V
    Muscle::pid_cfg_t pid_conf = {.p = 40.0, .i = 30.0, .d = 0.1, .lower_clamp = -4.5, .upper_clamp = 4.5};

    //Muscles are numbered by their order of labels in the setup. Indices are based on the hardware's channel numbers
    Muscle::muscle_cfg_t muscle_conf_0 = {.adc_index = 0, .dac_index = 8, .tension_sensor_index = 0, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_1 = {.adc_index = 1, .dac_index = 9, .tension_sensor_index = 1,.muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_2 = {.adc_index = 2, .dac_index = 10, .tension_sensor_index = 2,.muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_3 = {.adc_index = 3, .dac_index = 11, .tension_sensor_index = 3,.muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_4 = {.adc_index = 4, .dac_index = 12, .tension_sensor_index = 4,.muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_5 = {.adc_index = 5, .dac_index = 13, .tension_sensor_index = 0, .muscle_spindle_index = 8,
            .pid_cfg = pid_conf, .board = &control_board,.mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_6 = {.adc_index = 6, .dac_index = 14, .tension_sensor_index = 6,.muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_7 = {.adc_index = 7, .dac_index = 15, .tension_sensor_index = 2, .muscle_spindle_index = 9,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

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

    //存放拉力的数据
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
    int *marker = new int[100000];
    char filename[100];
    double goal_pressure = 0;
    int t_ = 0, sample_count = 0, temp = -1;
    std::chrono::steady_clock::time_point t_init = std::chrono::steady_clock::now();
    std::chrono::nanoseconds ts = t_init.time_since_epoch();

    while(true)
    {
        control_board.update_inputs();
        std::chrono::steady_clock::time_point t_curr = std::chrono::steady_clock::now();
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t_curr - t_init);
        //update goal pressure every second.
        t_ = t.count() / 1000;
        /*if (t_ < 2) {
            goal_pressure = 0;
        } else if ( t_ < 12) {
            goal_pressure = t_ * 0.03 + 0.14;
        } else if (t_ < 22) {
            goal_pressure = t_ * (-0.03) + 0.86;
        } else if (t_ <= 24) {
            goal_pressure = 0.2;
        } else {
            break;
        }*/
       goal_pressure = 0.2;

        Muscle::muscle_cmd_t m_cmd_left = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = goal_pressure, .goal_activation = 0.0};
        Muscle::muscle_state_t s_left = muscle_5->updateMuscle(m_cmd_left);

        Muscle::muscle_cmd_t m_cmd_right = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = goal_pressure, .goal_activation = 0.0};
        Muscle::muscle_state_t s_right = muscle_7->updateMuscle(m_cmd_right);

        sample_count = t.count() / 20; //Sampling Frequency = 50Hz, 每20ms采样一次

        if (sample_count != temp) {
            temp = sample_count;
            tension_data_left[sample_count] = muscle_5->getMuscleState().current_tension_sensor_feedback;
            ms_data_left[sample_count] = muscle_5->getMuscleState().current_ms_resistance;
            ms_data_right_speed[sample_count] = 0;
            pressure_data_left[sample_count] = muscle_5->getMuscleState().current_pressure;
            g_pressure_1[sample_count] = goal_pressure;

            tension_data_right[sample_count] = muscle_7->getMuscleState().current_tension_sensor_feedback;
            ms_data_right[sample_count] = muscle_7->getMuscleState().current_ms_resistance;
            ms_data_right_speed[sample_count] = 0;
            pressure_data_right[sample_count] = muscle_7->getMuscleState().current_pressure;
            g_pressure_2[sample_count] = goal_pressure;

            marker[sample_count] = 0;
        }

        if(t.count() / 1000 >= 10) break;
    }
    //std::cout << "Muscle 5 loadcell average is " << getAverage(tension_data_left) << std::endl;
   // std::cout << "Muscle 7 loadcell average is " << getAverage(tension_data_right) << std::endl;
    sprintf(filename, "../data/hysteresis_test/jiang_test/0629_tension_comp_%ld.txt", ts.count());
    std::cout << "prepare to write data to file" << std::endl;
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);
    file << "count\t"
         << "tension_left\t" << "ms_left\t" << "ms_speed_left\t" << "pressure_left\t" << "g_pressure_1\t"
         << "tension_right\t" << "ms_right\t" << "ms_speed_right\t" << "pressure_right\t" <<"g_pressure_2\t"
         << "start of cycle" << std::endl;
    for (int i = 0; i <= sample_count; i++)
    {
        file << i << "\t" << tension_data_left[i] << "\t" << ms_data_left[i] << "\t" << ms_data_left_speed[i] << "\t"
             << pressure_data_left[i] << "\t" << g_pressure_1[i] << "\t"
             << tension_data_right[i] << "\t" << ms_data_right[i] << "\t" << ms_data_right_speed[i] << "\t"
             << pressure_data_right[i] << "\t" << g_pressure_2[i] << "\t" << marker[i]  << std::endl;
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
