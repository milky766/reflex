//
// Created by Sherry Wang on 2021/7/15.
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
#include <math.h>

#include <control_board.h>
#include <muscle.h>
#include <interneuron.h>
#include <top_controller.h>
#include <comparator.h>

#define PI acos(-1)
#define STATIC 0
#define RHYTHMIC 1
#define REACHING 2

static ControlBoard control_board;

[[noreturn]] void *controlLoop(void *)
{
    //Neutral Position of Valve is at 5V. So clamps are set to -4.5V and 4.5V for effective control voltages of 0.5 to 9.5V
    Muscle::pid_cfg_t pid_conf = {.p = 30.0, .i = 20.0, .d = 0.01, .lower_clamp = -4.5, .upper_clamp = 4.5};

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
            .pid_cfg = pid_conf, .board = &control_board,.mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_6 = {.adc_index = 6, .dac_index = 14, .tension_sensor_index = 6, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_7 = {.adc_index = 7, .dac_index = 15, .tension_sensor_index = 2, .muscle_spindle_index = 9,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    uint8_t potentiometer_idx = 10;

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

    auto *IN_1 = new Interneuron(muscle_5);
    auto *IN_2 = new Interneuron(muscle_7);
    auto *cmp = new Comparator(IN_1, IN_2);
    auto *top_con = new topController(0.15, 0.40, 3, HZ);
    // topController top_con(0.40, 0.15, 3, HZ);
    //topController top_con(0.3, 0.3, 3, HZ);
    topController::goal_pressure commands = {0.1, 0.1};
    double curr_tension_ = 0.0, ave_tension_ = 0.0;
    //detect the change of sensors
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
    int *angle = new int[100000];
//    double *left_v = new double[1500];
//    double *left_a = new double[1500];
//    double *right_v = new double[1500];
//    double *right_a = new double[1500];

    int sample_count = -1;// pre_time = -1;
    double alpha_m1, alpha_m2;
    int muscle_spindle_reflex_signal = 0;
    std::chrono::steady_clock::time_point t_init = std::chrono::steady_clock::now();
    std::chrono::nanoseconds ts = t_init.time_since_epoch();
    char filename[1000];
    while(true)
    {
        std::chrono::steady_clock::time_point t_curr = std::chrono::steady_clock::now();
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t_curr - t_init);
        if (t.count()/ 10 == sample_count) continue;
        sample_count = t.count() / 10;
//        std :: cout << sample_count << std::endl;
//        if (t.count() / 10 == pre_time) continue;
//        pre_time = t.count() / 10;

        control_board.update_inputs();
        IN_1->update_sensor_info();
        IN_2->update_sensor_info();
        muscle_spindle_reflex_signal = Interneuron::get_mutex();
        cmp->update(muscle_spindle_reflex_signal);
//->where the key is! this is for get motor commands. 
        //commands = top_con.get_pattern(RHYTHMIC);

        if (t.count() < 1000) {
            alpha_m1 = commands.goal_pressure_m1;
            alpha_m2 = commands.goal_pressure_m2;
        } else if (t.count() < 12000) {
                if (cmp->get_GTO_status() == 0) {
                    commands = top_con->get_pattern(RHYTHMIC);
                }
                //Comparator::tension_feedback_command tensionFeedbackCommand = cmp->tension_feedback();
                //cout << cmp->get_GTO_status() << endl;
                alpha_m1 = commands.goal_pressure_m1 + IN_1->stretch_reflex() - IN_2->reciprocal_inhibition();
                // - tensionFeedbackCommand.feedback_m1;

               if (alpha_m1 > 0.5) {
                   alpha_m1 = 0.5;
               } else if (alpha_m1 < 0) {
                   alpha_m1 = 0;
               }

                alpha_m2 = commands.goal_pressure_m2 + IN_2->stretch_reflex() - IN_1->reciprocal_inhibition();
                // - tensionFeedbackCommand.feedback_m2;

               if (alpha_m2 > 0.5) {
                   alpha_m2 = 0.5;
               } else if (alpha_m2 < 0) {
                   alpha_m2 = 0;
               }
           //this if judge influence a lot on the response of the muscle after triggered. 
           //there is still some work need to be done here. IN_1 and IN_2 need a lock system. which means only one of them can work. (the one who reaches the threshold at first)
           //the one which is triggered, has its stretch speed overpass the threshold around 20ms earlier than the other side.
        } else {
            Muscle::muscle_cmd_t m_cmd_5 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.1,
                                            .goal_activation = 0.0};
            Muscle::muscle_state_t s_5 = muscle_5->updateMuscle(m_cmd_5);
            Muscle::muscle_cmd_t m_cmd_7 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.1,
                                            .goal_activation = 0.0};
            Muscle::muscle_state_t s_7 = muscle_7->updateMuscle(m_cmd_7);

            break;
        }
        //std::cout << "left:\t" << alpha_m1 << "\tright:\t" << alpha_m2 << std::endl;

        Muscle::muscle_cmd_t m_cmd_5 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = alpha_m1,
                                        .goal_activation = 0.0};
        Muscle::muscle_state_t s_5 = muscle_5->updateMuscle(m_cmd_5);
        Muscle::muscle_cmd_t m_cmd_7 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = alpha_m2,
                                        .goal_activation = 0.0};
        Muscle::muscle_state_t s_7 = muscle_7->updateMuscle(m_cmd_7);

        g_pressure_1[sample_count] = alpha_m1;
        g_pressure_2[sample_count] = alpha_m2;

        marker[sample_count] = top_con->isStart();

        //记录数据：
        tension_data_left[sample_count] = IN_1->get_tension();
        ms_data_left[sample_count] = IN_1->get_stretch();
        ms_data_left_speed[sample_count] = IN_1->get_stretch_speed();
        pressure_data_left[sample_count] = muscle_5->getMuscleState().current_pressure;

        tension_data_right[sample_count] = IN_2->get_tension();
        ms_data_right[sample_count] = IN_2->get_stretch();
        ms_data_right_speed[sample_count] = IN_2->get_stretch_speed();
        pressure_data_right[sample_count] = muscle_7->getMuscleState().current_pressure;

        angle[sample_count] = control_board.getPotentiometerData(potentiometer_idx);
//        left_v[sample_count] = IN_1->rate_len;
//        left_a[sample_count] = IN_1->rate_v;
//        right_v[sample_count] = IN_2->rate_len;
//        right_a[sample_count] = IN_2->rate_v;

        //IN_2->print_timer();
        //cout << alpha_m2 << "\t" << IN_2->stretch_reflex() << endl;

        //sample_count++;
//        usleep(10000); //10000 us = 10 ms -> 100 Hz
        // 睡眠10ms，可以按照t计算。每10ms走一次循环。
        //如果ms 用这个100Hz的频率算变化速度，那么是不是可以把tension sensor的频率也提高到100Hz。
    }
    sprintf(filename, "../data/hierarchical_test/1102/1102_reflex_method_test_rotation_%lld.txt", ts.count());
    std::cout << "prepare to write data to file" << std::endl;
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);
    file << "count\t"
    << "tension_left\t" << "ms_left\t" << "ms_speed_left\t" << "pressure_left\t" << "g_pressure_1\t"
    << "tension_right\t" << "ms_right\t" << "ms_speed_right\t" << "pressure_right\t" <<"g_pressure_2\t"
    << "start of cycle\t" << "angle" << std::endl;
    for (int i = 0; i <= sample_count; i++)
    {
        file << i << "\t" << tension_data_left[i] << "\t" << ms_data_left[i] << "\t" << ms_data_left_speed[i] << "\t"
             << pressure_data_left[i] << "\t" << g_pressure_1[i] << "\t"
                << tension_data_right[i] << "\t" << ms_data_right[i] << "\t" << ms_data_right_speed[i] << "\t"
             << pressure_data_right[i] << "\t" << g_pressure_2[i] << "\t" << marker[i] << "\t" << angle[i] <<
             std::endl;
    }
    file.close();
    std::cout << "write finished"<< std::endl;

//    sprintf(filename, "../data/hierarchical_test/1010/1010_buffer_%ld.txt", ts.count());
//    std::cout << "prepare to write data to file" << std::endl;
//    std::ofstream file1;
//    file1.open(filename, std::ios::out | std::ios::app);
//    file1 << "count\t"
//          << "speed_left\t" << "acceleration_left\t" << "speed_right\t" << "acceleration_right\t"
//          << "start of cycle" << std::endl;
//    for (int i = 0; i <= sample_count; i++)
//    {
//         file1 << i << "\t" << left_v[i] << "\t" << left_a[i] << "\t"
//         << right_v[i] << "\t" << right_a[i] << "\t" << marker[i]  << std::endl;
//    }
//    file1.close();
//    std::cout << "write finished"<< std::endl;

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
//    delete[] left_v;
//    delete[] left_a;
//    delete[] right_v;
//    delete[] right_a;
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
//
// Created by Sherry Wang on 2021/7/15.
//
