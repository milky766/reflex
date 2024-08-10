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


#include <iostream>  // std::cout, std::cin
#include <fstream>
#include <ctime>
#include <iomanip>  // std::setw, std::setfill
#include <sstream>  // std::stringstream


#define PI acos(-1)
#define STATIC 0
#define RHYTHMIC 1
#define REACHING 2
#define NOREFLEX 0
#define IA 1
#define IB 2
#define IAIB 3



static ControlBoard control_board; //ControlBoardクラスのインスタンスcontrol_boardが作成されます。このインスタンスは、プログラム内で唯一のインスタンスとなり、複数の関数やスレッドから共有されます。
 
[[noreturn]] void *controlLoop(void *) //[[noreturn]]を使うことで、コンパイラに対して「この関数は戻らない」という情報を提供し、最適化や警告の抑制に役立ちます。→つまり、無限に続く関数 // void * は「何でも入るポインタ」です。スレッド関数に渡したいデータの種類が決まっていないときに使います。この関数にデータを渡すときは、好きなデータ型に変換（キャスト）して使います。


{
    std::string userInput;
    std::cout << "Please enter the filename suffix: ";
    std::cin >> userInput;

    //Neutral Position of Valve is at 5V. So clamps are set to -4.5V and 4.5V for effective control voltages of 0.5 to 9.5V
    Muscle::pid_cfg_t pid_conf = {.p = 30.0, .i = 20.0, .d = 0.01, .lower_clamp = -4.5, .upper_clamp = 4.5}; //pid_cfg_tはPID値やセンサのインデックスなどを設定する構造体
    Muscle::pid_cfg_t pid_conf2 = {.p = 30.0, .i = 5.0, .d = 0.01, .lower_clamp = -4.5, .upper_clamp = 4.5};

    //Muscles are numbered by their order of labels in the setup. Indices are based on the hardware's channel numbers
    Muscle::muscle_cfg_t muscle_conf_0 = {.adc_index = 0, .dac_index = 8, .tension_sensor_index = 0, .muscle_spindle_index = 16,//muscle_cfg_tはPID値やセンサのインデックスなどを設定する構造体
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_1 = {.adc_index = 1, .dac_index = 9, .tension_sensor_index = 1, .muscle_spindle_index = 16, //制御弁8個あるから、muscle_0からmuscle_7までの8個のmuscleを設定.ワンさんが使っているのはmuscle_5とmuscle_7のみ
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_2 = {.adc_index = 2, .dac_index = 10, .tension_sensor_index = 2, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"}; //mslo_mshiはmuscle_spindleがhiかloか?

    Muscle::muscle_cfg_t muscle_conf_3 = {.adc_index = 3, .dac_index = 11, .tension_sensor_index = 4, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_4 = {.adc_index = 4, .dac_index = 12, .tension_sensor_index = 3, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_5 = {.adc_index = 5, .dac_index = 13, .tension_sensor_index = 6, .muscle_spindle_index = 10,
            .pid_cfg = pid_conf2, .board = &control_board,.mslo_mshi = "mslo"};

    Muscle::muscle_cfg_t muscle_conf_6 = {.adc_index = 6, .dac_index = 14, .tension_sensor_index = 5, .muscle_spindle_index = 16,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    Muscle::muscle_cfg_t muscle_conf_7 = {.adc_index = 7, .dac_index = 15, .tension_sensor_index = 7, .muscle_spindle_index = 11,
            .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mslo"}; //int *p = &x;  pはxのアドレスを指すポインタ

    //    Muscle::muscle_cfg_t muscle_conf_7 = {.adc_index = 7, .dac_index = 15, .tension_sensor_index = 2, .muscle_spindle_index = 9,
    //        .pid_cfg = pid_conf, .board = &control_board, .mslo_mshi = "mshi"};

    uint8_t potentiometer_idx = 12;

    // Using auto to avoid repeating Classname
    auto *muscle_0 = new Muscle(muscle_conf_0); //newにより動的メモリ割り当てを使用することで、必要なときに必要なだけのメモリを確保でき、柔軟性が向上します。オブジェクトの寿命を制御できる。
    auto *muscle_1 = new Muscle(muscle_conf_1); //Muscle *muscle_0 = new Muscle(muscle_conf_0); これをautoを使って簡潔にする.autoはコンパイラに型を推論させるキーワードで、変化に強くなる。
    auto *muscle_2 = new Muscle(muscle_conf_2);// 各Muscleオブジェクトを初期化し、それぞれの筋肉が異なる設定（muscle_cfg_t）を持つようにする。
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
    Muscle::muscle_state_t s_0 = muscle_0->updateMuscle(m_cmd_0); //->は、ポインタを使ってメンバ関数にアクセスするための演算子

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
    auto *top_con = new topController(0.6);//new演算子を使用すると、指定されたコンストラクタが呼び出され、動的メモリに新しいオブジェクトが作成されます。
    //auto *top_con = new topController(0.25, 0.25, 3, 100); //rotation mode 
    //auto *top_con = new topController(0.40, 0.15, 3, 100);
    // auto *top_con = new topController(0, 0, 0.40, 0.15, 10.0, 100);
    //topController top_con(0.40, 0.15, 3, HZ);
    //topController top_con(0.3, 0.3, 3, HZ);
    topController::goal_pressure higher_commands = {0.1, 0.1}; //goal_pressure_m1とgoal_pressure_m2
    SpinalCord *low_con = new SpinalCord(muscle_5, muscle_7); //5がagonist, 7がantagonist
    SpinalCord::Reflex_feedback reflex_commands_agonist = {0, 0, false};
    SpinalCord::Reflex_feedback reflex_commands_agonist_model = {0, 0, false};
    SpinalCord::Reflex_feedback reflex_commands_antagonist = {0, 0, false};
    SpinalCord::Reflex_feedback reflex_commands_antagonist_model = {0, 0, false};
    SpinalCord::GTO_feedback reflex_gto_agonist = {0, 0, false};
    SpinalCord::GTO_feedback reflex_gto_antagonist = {0,0, false};
    SpinalCord::Sensor_info sensor_info_agonist;
    SpinalCord::Sensor_info sensor_info_antagonist;
    SpinalCord::Sensor_info sensor_info_agonist_model;
    SpinalCord::Sensor_info sensor_info_antagonist_model;
    // SpinalCord::Length_model length_model_agonist;
    // SpinalCord::Length_model length_model_antagonist;

    double General_Harding_command;


    /*
    static SpinalCord::MaxTracker MaxTracker_;
    MaxTracker_.a_Ia = 1;
    */



    //double curr_tension_ = 0.0, ave_tension_ = 0.0;


    
    //store the change of sensors
    //追加
    double *tension_data_left = new double[100000];
    double *tension_data_right = new double[100000];
    double *pressure_data_left = new double[100000];
    double *pressure_data_right = new double[100000];
    double *ms_data_left = new double[100000];
    double *ms_data_right = new double[100000];
    double *ms_data_left_speed = new double[100000];
    double *ms_data_right_speed = new double[100000];

    //追加
    double *ms_data_left_model = new double[100000];
    double *ms_data_right_model = new double[100000];
    double *ms_data_left_speed_model = new double[100000];
    double *ms_data_right_speed_model = new double[100000];

    double *g_pressure_1 = new double[100000];
    double *g_pressure_2 = new double[100000];
    double *ago_SR = new double[100000]; //Stretch reflex
    double *ago_SR_model = new double[100000];
    double *ago_RI = new double[100000]; //Reciprocal inhibition
    double *ago_RI_model = new double[100000];
    double *ago_AI = new double[100000]; //Autogenic inhibition
    double *ago_RE = new double[100000]; //reciprocal excitation
    double *antago_SR = new double[100000];
    double *antago_SR_model = new double[100000];
    double *antago_RI = new double[100000];
    double *antago_RI_model = new double[100000];
    double *antago_AI = new double[100000];
    double *antago_RE = new double[100000];

    double base_a_len, base_anta_len, base_a_tension, base_anta_tension;
    //追加
    double base_a_len_model, base_anta_len_model;

    int *marker = new int[100000];
    int *angle = new int[100000];

    int sample_count = 0;
    double alpha_m1, alpha_m2;

    double PreAlpha_m1, PreAlpha_m2;
    int EntryFlag_Ia, ExitFlag_Ia;
    int EntryFlag_Ib, ExitFlag_Ib;

    int temp_FeedbackFlag_Ia = 0;
    int temp_FeedbackFlag_Ib = 0;

    bool result;

    int muscle_spindle_reflex_signal = 0;
    
	RateLoop *initiator = new RateLoop(100, 2.0); //100Hzのwhile loopを2秒間実行する
	while (initiator->Sleep()) //2秒間このループを実行する
	{ 

		control_board.update_inputs(); //この ControlBoard::update_inputs メソッドは、ControlBoard クラスのメンバー関数であり、ADC (アナログ-デジタルコンバータ) とロードセル (荷重センサ) からデータを取得し、内部データストアに更新しています。

        static bool FirstRun = true;
        if(FirstRun){
            muscle_5->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.0, .goal_activation = 0.0});
            muscle_7->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.0, .goal_activation = 0.0});
            
            low_con->update_base_sensor_info();
            low_con->update_base_sensor_info_model();

            base_a_len = low_con->getBaseSensorInfo().base_agonist_len;
            base_anta_len = low_con->getBaseSensorInfo().base_antagonist_len;
            base_a_len_model = low_con->getBaseSensorInfo_model().base_agonist_len;
            base_anta_len_model = low_con->getBaseSensorInfo_model().base_antagonist_len;
            base_a_tension = low_con->getBaseSensorInfo().base_agonist_tension;
            base_anta_tension = low_con->getBaseSensorInfo().base_antagonist_tension;

            FirstRun = false;
        }
        muscle_5->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.0, .goal_activation = 0.0});
        muscle_7->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.0, .goal_activation = 0.0});
	}
    //muscle_7->updateMuscle({.control_mode = Muscle::ControlMode::pressure, .goal_pressure = 0.40, .goal_activation = 0.0});
    //std::cout << "start" << std:: endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //usleep(10000);
    //std::cout << "end" << std:: endl;
    
    static int para_intervene_a = 0; //、static キーワードがローカル変数の前に使われると、その変数は関数が終了しても存在し続け、再度その関数が呼び出されたときに前回の値を保持します。
	static int para_intervene_b = 0; //para_interveneはreflexの回数を表している
    
    RateLoop *rater = new RateLoop(100, 14.0);
    //frequency of while loop will be 100 HZ, total experiment time is 16 seconds

    while(true)
    {
        // static bool FirstRun = true;
        // if(FirstRun){ 
        //     low_con->update_base_sensor_info();
        //     low_con->update_base_sensor_info_model();
        //     FirstRun = false;
        // }
        
        EntryFlag_Ia = 0;
        ExitFlag_Ia = 0;
        
        EntryFlag_Ib = 0;
        ExitFlag_Ib = 0;
        
        temp_FeedbackFlag_Ia = low_con->FeedbackFlag_Ia; //SpinalCord *low_con = new SpinalCord(muscle_5, muscle_7)
        temp_FeedbackFlag_Ib = low_con->FeedbackFlag_Ib;

        control_board.update_inputs();        

        low_con->update_sensor_info(para_intervene_a,para_intervene_b);
        
        if(temp_FeedbackFlag_Ia != low_con->FeedbackFlag_Ia && low_con->FeedbackFlag_Ia == 1){
            EntryFlag_Ia = 1;
        }else if(temp_FeedbackFlag_Ia != low_con->FeedbackFlag_Ia && low_con->FeedbackFlag_Ia == 0){
            ExitFlag_Ia = 1;
        }

        if(temp_FeedbackFlag_Ib != low_con->FeedbackFlag_Ib && low_con->FeedbackFlag_Ib == 1){
            EntryFlag_Ib = 1;
        }else if(temp_FeedbackFlag_Ib != low_con->FeedbackFlag_Ib && low_con->FeedbackFlag_Ib == 0){
            ExitFlag_Ib = 1;
        }

        if(EntryFlag_Ia == 1){
            PreAlpha_m1 = alpha_m1;
            PreAlpha_m2 = alpha_m2;
        }

        if(ExitFlag_Ib == 1){
            PreAlpha_m1 = alpha_m1;
            PreAlpha_m2 = alpha_m2;
        }

        //result = temp_FeedbackFlag != low_con->FeedbackFlag_;

                std::cout << "******EntryFlag_Ia = " << EntryFlag_Ia << "*****ExitFlag_Ia = " << ExitFlag_Ia << "*****temp_FeedbackFlag_Ia =" << temp_FeedbackFlag_Ia << std::endl;
                std::cout << "******EntryFlag_Ib = " << EntryFlag_Ib << "*****ExitFlag_Ib = " << ExitFlag_Ib << "*****temp_FeedbackFlag_Ib =" << temp_FeedbackFlag_Ib << std::endl;

                

                //std::cout << "******result = " << result << std::endl;
                
                std::cout << "******PreAlpha_m1 = " << PreAlpha_m1 << "*****PreAlpha_m2 = " << PreAlpha_m2 << "***********" << std::endl;

				std::cout << "******para_intervene_a = " << para_intervene_a <<"*****para_intervene_b =  " << para_intervene_b << " ***********" << std::endl;


//change mode: 2 //タスク
		switch(1) //switch文は、複数の条件によって処理を分岐するための制御構造です。今回は必ずcase1が実行される
        { 
            case 0: 
            {
                higher_commands = top_con->get_pattern(REACHING); //reflex無しのreaching
                break;
            }
            case 1:
            {
                if (ExitFlag_Ib == 1)
                    higher_commands = top_con->get_pattern(STATIC, PreAlpha_m1, PreAlpha_m2); // #define STATIC 0
                else
                    higher_commands = top_con->get_pattern(STATIC);
                break;
            }
            case 2:
            {
                if(ExitFlag_Ia == 1 || ExitFlag_Ib == 1)
                    higher_commands = top_con->get_pattern(REACHING, PreAlpha_m1, PreAlpha_m2);
                else
                    higher_commands = top_con->get_pattern(REACHING);//reflexありのreaching
                break;
            }
        }	 
		//higher_commands = top_con->get_pattern(RHYTHMIC, para_intervene, alpha_m1, alpha_m2);
        
        reflex_commands_agonist = low_con->agonist_Ia_innervation(); //maxtrackerの値を更新したり、CoConrinatorの値を更新したりする
        reflex_commands_antagonist = low_con->antagonist_Ia_innervation();
        //reflex_commands_agonist_model = low_con->agonist_Ia_innervation_model(); 
        // reflex_commands_antagonist_model = low_con->antagonist_Ia_innervation_model();
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
        switch(0)  //1は伸張反射のみ。作動圧力範囲は0.02~0.5MPa
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
            // case 1:
            // {
            //     alpha_m1 = higher_commands.goal_pressure_m1 + reflex_commands_agonist_model.res_SR - reflex_commands_antagonist_model.res_RI;
            //     alpha_m2 = higher_commands.goal_pressure_m2 + reflex_commands_antagonist_model.res_SR - reflex_commands_agonist_model.res_RI;
            //     break;
            // }
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
        
		if (alpha_m1 > 0.6) {
            alpha_m1 = 0.6;
        } else if (alpha_m1 < 0.02) {
            alpha_m1 = 0.02;
        }
       
		if (alpha_m2 > 0.6) {
            alpha_m2 = 0.6;
        } else if (alpha_m2 < 0.02) {
            alpha_m2 = 0.02;
        }
       // std::cout << "left:\t" << higher_commands.goal_pressure_m1 << "\tright:\t" << higher_commands.goal_pressure_m2 << std::endl;

        Muscle::muscle_cmd_t m_cmd_5 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = alpha_m1, .goal_activation = 0.0};
        Muscle::muscle_state_t s_5 = muscle_5->updateMuscle(m_cmd_5);
        Muscle::muscle_state_t p_5 = muscle_5->printMuscle(m_cmd_5); 
        Muscle::muscle_cmd_t m_cmd_7 = {.control_mode = Muscle::ControlMode::pressure, .goal_pressure = alpha_m2, .goal_activation = 0.0};
        Muscle::muscle_state_t s_7 = muscle_7->updateMuscle(m_cmd_7);
        Muscle::muscle_state_t p_7 = muscle_7->printMuscle(m_cmd_7);

        g_pressure_1[sample_count] = alpha_m1; 
        g_pressure_2[sample_count] = alpha_m2; 

        marker[sample_count] = top_con->isStart(STATIC);
        //marker[sample_count] = top_con->isStart();
        sensor_info_agonist = low_con->get_agonist_sensor_info(); 
        sensor_info_antagonist = low_con->get_antagonist_sensor_info();

        sensor_info_agonist_model = low_con->get_agonist_sensor_info_model(); 
        sensor_info_antagonist_model = low_con->get_antagonist_sensor_info_model();

        //记录数据：
        tension_data_left[sample_count] = sensor_info_agonist.muscle_tension; 
        ms_data_left[sample_count] = sensor_info_agonist.muscle_len;
        ms_data_left_speed[sample_count] = sensor_info_agonist.muscle_filtered_v;

        ms_data_left_model[sample_count] = sensor_info_agonist_model.muscle_len;
        ms_data_left_speed_model[sample_count] = sensor_info_agonist_model.muscle_filtered_v;

        pressure_data_left[sample_count] = s_5.current_pressure;

        tension_data_right[sample_count] = sensor_info_antagonist.muscle_tension;
        ms_data_right[sample_count] = sensor_info_antagonist.muscle_len;
        ms_data_right_speed[sample_count] = sensor_info_antagonist.muscle_filtered_v;

        ms_data_left_model[sample_count] = sensor_info_agonist_model.muscle_len;
        ms_data_left_speed_model[sample_count] = sensor_info_agonist_model.muscle_filtered_v;

        
        ms_data_right_model[sample_count] = sensor_info_antagonist_model.muscle_len;
        ms_data_right_speed_model[sample_count] = sensor_info_antagonist_model.muscle_filtered_v;


        pressure_data_right[sample_count] = s_7.current_pressure;

        angle[sample_count] = control_board.getPotentiometerData(potentiometer_idx);

        ago_SR[sample_count] = reflex_commands_agonist.res_SR;
        ago_SR_model[sample_count] = reflex_commands_agonist_model.res_SR;
        ago_RI[sample_count] = reflex_commands_agonist.res_RI;
        ago_RI_model[sample_count] = reflex_commands_agonist_model.res_RI;
        ago_AI[sample_count] = reflex_gto_agonist.res_AI;
        ago_RE[sample_count] = reflex_gto_agonist.res_RE;
        antago_SR[sample_count] = reflex_commands_antagonist.res_SR;
        antago_SR_model[sample_count] = reflex_commands_antagonist_model.res_SR;
        antago_RI[sample_count] = reflex_commands_antagonist.res_RI;
        antago_RI_model[sample_count] = reflex_commands_antagonist_model.res_RI;
        antago_AI[sample_count] = reflex_gto_antagonist.res_AI;
        antago_RE[sample_count] = reflex_gto_antagonist.res_RE;


        sample_count++;
        if(!rater->Sleep()) {
            break;
        }
//        usleep(10000); //10000 us = 10 ms -> 100 Hz
    }

    muscle_5->updateMuscle({.control_mode = Muscle::ControlMode::pressure,
                            .goal_pressure = 0.0, .goal_activation = 0.0});
    muscle_7->updateMuscle({.control_mode = Muscle::ControlMode::pressure,
                            .goal_pressure = 0.0, .goal_activation = 0.0});

    control_board.update_inputs(); //ADC (アナログ-デジタルコンバータ) とロードセル (荷重センサ) からデータを取得し、内部データストアに更新しています。

    // base_a_len = low_con->getBaseSensorInfo().base_agonist_len;
    // base_anta_len = low_con->getBaseSensorInfo().base_antagonist_len;

    // base_a_len_model = low_con->getBaseSensorInfo_model().base_agonist_len;
    // base_anta_len_model = low_con->getBaseSensorInfo_model().base_antagonist_len;

    // base_a_tension = low_con->getBaseSensorInfo().base_agonist_tension;
    // base_anta_tension = low_con->getBaseSensorInfo().base_antagonist_tension;


    // char filename[1000];
    // sprintf(filename, "../data/feedback_test/20221114/DualReflex_%lld.txt", rater->TimeStamp());
    // std::cout << "prepare to write data to file" << std::endl;
    // std::ofstream file;
    // file.open(filename, std::ios::out | std::ios::app);

 // 現在の日付を取得してフォーマット
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::stringstream ss;
    ss << (tm.tm_year + 1900) << std::setw(2) << std::setfill('0') << (tm.tm_mon + 1) << std::setw(2) << std::setfill('0') << tm.tm_mday;

    // ファイル名を設定
    // std::string filename = "../data/feedback_test/" + ss.str() + "_" + userInput + ".txt";
    std::string filename = "../data/feedback_test/" + ss.str() + "_" + userInput + ".csv";
    std::cout << "prepare to write data to file" << std::endl;
    std::ofstream file;
    file.open(filename.c_str(), std::ios::out | std::ios::app);
    file 
        //  << "count\t"
        //  << "tension_left\t" << "ms_left\t" 
        //  << "ms_left_model\t"
        //  <<"ms_speed_left\t" 
        //  <<"ms_speed_left_model\t" 
        //  << "pressure_left\t" << "g_pressure_1\t"<< "tension_right\t"<< "ms_right\t" 
        //  << "ms_right_model\t"
        //  <<"ms_speed_right\t" 
        //  <<"ms_speed_right_model\t"
        //  << "pressure_right\t" <<"g_pressure_2\t"<< "start of cycle\t" << "angle\t" 
        //  << "m5_Ia_+\t"<< "m5_Ia_model_+\t" << "m5_Ia_-\t"<< "m5_Ia_model_-\t" << "m5_Ib_-\t" << "m5_Ib_+\t" 
        //  << "m7_Ia_+\t" << "m7_Ia_model_+\t" << "m7_Ia_-\t"<< "m7_Ia_model_-\t" << "m7_Ib_-\t" << "m7_Ib_+\t"
        //  << base_a_len << "\t" 
        //  << base_anta_len << "\t"
        //  //追加
        //  << base_a_len_model << "\t" 
        //  << base_anta_len_model << "\t"

        //  << base_a_tension << "\t"
        //  << base_anta_tension << "\t"
         << "count,"
         << "tension_left," << "ms_left," 
         << "ms_left_model,"
         <<"ms_speed_left," 
         <<"ms_speed_left_model," 
         << "pressure_left," << "g_pressure_1,"<< "tension_right,"<< "ms_right," 
         << "ms_right_model,"
         <<"ms_speed_right," 
         <<"ms_speed_right_model," 
         << "pressure_right," <<"g_pressure_2,"<< "start of cycle," << "angle," 
         << "m5_Ia_+,"<< "m5_Ia_model_+," << "m5_Ia_-,"<< "m5_Ia_model_-," << "m5_Ib_-," << "m5_Ib_+," 
         << "m7_Ia_+," << "m7_Ia_model_+," << "m7_Ia_-,"<< "m7_Ia_model_-," << "m7_Ib_-," << "m7_Ib_+," 
         << base_a_len << "," 
         << base_anta_len << ","
         // 追加
         << base_a_len_model << "," 
         << base_anta_len_model << ","

         << base_a_tension << ","
         << base_anta_tension << ","
         << std::endl;
         
    for (int i = 0; i <= sample_count; i++)
    {
        file 
            //  << i << "\t" << tension_data_left[i] << "\t" << ms_data_left[i] << "\t" 
            //  <<ms_data_left_model[i] << "\t"
            //  << ms_data_left_speed[i] << "\t"
            //  << ms_data_left_speed_model[i] << "\t"
            //  << pressure_data_left[i] << "\t" << g_pressure_1[i] << "\t"
            //  << tension_data_right[i] << "\t" << ms_data_right[i] << "\t"
            //  << ms_data_right_model[i] << "\t"
            //  << ms_data_right_speed[i] << "\t"
            //  <<ms_data_right_speed_model[i] << "\t"
            //  << pressure_data_right[i] << "\t" << g_pressure_2[i] << "\t" << marker[i] << "\t" << angle[i] << "\t" 
            //  << ago_SR[i] << "\t"<< ago_SR_model[i] << "\t" << ago_RI[i] << "\t"<< ago_RI_model[i] << "\t" << ago_AI[i] << "\t" << ago_RE[i] << "\t" 
            //  << antago_SR[i] << "\t"<< antago_SR_model[i] << "\t" << antago_RI[i] << "\t" << antago_RI_model[i] << "\t" << antago_AI[i] << "\t" << antago_RE[i]
             << i << "," << tension_data_left[i] << "," << ms_data_left[i] << "," 
             << ms_data_left_model[i] << ","
             << ms_data_left_speed[i] << ","
             << ms_data_left_speed_model[i] << ","
             << pressure_data_left[i] << "," << g_pressure_1[i] << ","
             << tension_data_right[i] << "," << ms_data_right[i] << ","
             << ms_data_right_model[i] << ","
             << ms_data_right_speed[i] << ","
             << ms_data_right_speed_model[i] << ","
             << pressure_data_right[i] << "," << g_pressure_2[i] << "," << marker[i] << "," << angle[i] << "," 
             << ago_SR[i] << "," << ago_SR_model[i] << "," << ago_RI[i] << "," << ago_RI_model[i] << "," << ago_AI[i] << "," << ago_RE[i] << "," 
             << antago_SR[i] << "," << antago_SR_model[i] << "," << antago_RI[i] << "," << antago_RI_model[i] << "," << antago_AI[i] << "," << antago_RE[i]
             << std::endl;
    }
    file.close();
    std::cout << "write finished"<< std::endl;
    std::cout << "the base number is:\n" 
        << base_a_len << "\n" 
        << base_anta_len << "\n"
        << base_a_len_model << "\n" 
        << base_anta_len_model << "\n"
        << base_a_tension << "\n"
        << base_anta_tension << "\n"
        << std::endl;




    delete[] tension_data_left;
    delete[] tension_data_right;
    delete[] pressure_data_left;
    delete[] pressure_data_right;
    delete[] ms_data_left;
    delete[] ms_data_right;
    delete[] ms_data_left_speed;
    delete[] ms_data_right_speed;
    delete[] ms_data_left_model;
    delete[] ms_data_right_model;
    delete[] ms_data_left_speed_model;
    delete[] ms_data_right_speed_model;
    delete[] g_pressure_1;
    delete[] g_pressure_2;
    delete[] marker;
    delete[] angle;
    delete[] ago_AI;
    delete[] ago_RE;
    delete[] ago_RI;
    delete[] ago_RI_model;
    delete[] ago_SR;
    delete[] ago_SR_model;
    delete[] antago_AI;
    delete[] antago_RE;
    delete[] antago_RI;
    delete[] antago_RI_model;
    delete[] antago_SR;
    delete[] antago_SR_model;

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