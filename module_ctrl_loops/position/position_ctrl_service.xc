/**
 * @file  position_ctrl_server.xc
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/
#include <xs1.h>
#include <xscope.h>
#include <print.h>
#include <stdlib.h>

#include <position_ctrl_service.h>
#include <a4935.h>
#include <mc_internal_constants.h>
#include <hall_service.h>
#include <qei_service.h>
#include <filters_lib.h>
#include <stdio.h>



void init_position_control(interface PositionControlInterface client i_position_control)
{
    int ctrl_state;

    while (1) {
        ctrl_state = i_position_control.check_busy();
        if (ctrl_state == INIT_BUSY) {
            i_position_control.enable_position_ctrl();
        }

        if (ctrl_state == INIT) {
#ifdef debug_print
            printstrln("position_ctrl_service: position control initialized");
#endif
            break;
        }
    }
}

int position_limit(int position, int max_position_limit, int min_position_limit)
{
    if (position > max_position_limit) {
        position = max_position_limit;
    } else if (position < min_position_limit) {
        position = min_position_limit;
    }
    return position;
}

void position_control_service(ControlConfig &position_control_config,
                              interface HallInterface client ?i_hall,
                              interface QEIInterface client ?i_qei,
                              interface BISSInterface client ?i_biss,
                              interface AMSInterface client ?i_ams,
                              interface MotorcontrolInterface client i_motorcontrol,
                              interface PositionControlInterface server i_position_control[3])
{
    int actual_position = 0;
    int target_position = 0;
    int error_position = 0;
    int error_position_D = 0;
    int error_position_I = 0;
    int previous_error = 0;
    int position_control_out = 0;

//    int position_control_out_limit = 0;
//    int error_position_I_limit = 0;

    /* for cascaded control */
//    int adc_a, adc_b;
//    int velocity = 0;
    //General Control Variables
//    int first_loop_counter = 0;
//    unsigned T_k = 0, T_k_1n = 0;
    unsigned T_s_desired = 1000; //us
//    unsigned T_s = T_s_desired;

    //Joint Torque Control
//    int i1_torque_j_ref = 0;
    int f1_torque_j_lim = 50000;
//    float f1_torque_j_sens_measured = 0;
    int i1_torque_j_sens_offset = 0;
    int i1_torque_j_sens_offset_accumulator = 0;
//    float f1_torque_j_sens = 0;
//    int i1_torque_j_sens = 0;
//    int i1_torque_j_sens_1n = 0;
//    float f1_torque_j_sens_1n = 0;
//    float f1_torque_j_sens_2n = 0;
//    SecondOrderLPfilterParam torque_sensor_SO_LP_filter_param;
//    int torque_j_error = 0;
//    int torque_j_derivative = 0;
//    int torque_j_Kp = 0;
//    int torque_j_Ki = 0;
//    int torque_j_Kd = 0;
//    int torque_j_ctrl_cmd = 0;
//    int torque_j_error_integral = 0;
//    int torque_j_error_integral_limit = 0;
//    int torque_j_ctrl_cmd_lim = 200;

    // Position Control
//    int position_sens = 0;
//    float f1_velocity_sens = 0;
//    float f1_velocity_sens_1n = 0;
//    float f1_velocity_sens_2n = 0;
//    float f1_velocity_sens_measured = 0;
//    SecondOrderLPfilterParam velocity_sensor_SO_LP_filter_param;
//    int i1_velocity_sens = 0;
//    int position_error = 0;
//    int position_error_integral = 0;
//    int position_error_limited = 0;
//    int position_Kp = 0;
//    int position_Ki = 0;
//    int position_Kd = 0;
//    int position_ctrl_cmd = 0;
//    int position_ctrl_cmd_lim = 0;
//    int position_error_integral_limit = 0;

//    int torque_ref=100;

    // A2
//    second_order_LP_filter_init(/*f_c=*//*18*/120, /*T_s=*/T_s_desired, torque_sensor_SO_LP_filter_param);
//    second_order_LP_filter_init(/*f_c=*//*80*/140, /*T_s=*/T_s_desired, velocity_sensor_SO_LP_filter_param);

    // A6
//    second_order_LP_filter_init(/*f_c=*/18, /*T_s=*/T_s_desired, torque_sensor_SO_LP_filter_param);
//    second_order_LP_filter_init(/*f_c=*/80, /*T_s=*/T_s_desired, velocity_sensor_SO_LP_filter_param);
    /* end cascaded */


    int16_t int16_velocity_k = 0;
    int16_t int16_position_k = 0;
    int16_t int16_velocity_ref_k = 0;
    int16_t int16_position_ref_k = 0;

    timer t;
    unsigned int ts;

    int activate = 0;

    int config_update_flag = 1;

//    int i=0;
//    int offset=0;

//    MotorcontrolConfig motorcontrol_config;

    printstr(">>   SOMANET POSITION CONTROL SERVICE STARTING...\n");


    if (position_control_config.cascade_with_torque == 1) {
        i_motorcontrol.set_torque(0);
        delay_milliseconds(10);
        for (int i=0 ; i<2000; i++) {
            delay_milliseconds(1);
            i1_torque_j_sens_offset_accumulator += (i_motorcontrol.get_torque_actual());
        }
        i1_torque_j_sens_offset = i1_torque_j_sens_offset_accumulator / 2000;
        i_motorcontrol.set_voltage(0);
        printstrln(">>   POSITION CONTROL CASCADED WITH TORQUE");
    }

    t :> ts;
//    while(1)
//    {
//
//        printf("\n\n\n\n\nsending offset_detection command ...\n");
//        i_motorcontrol.set_offset_detection_enabled();
//        delay_milliseconds(10000);
//
//
//        offset=i_motorcontrol.set_calib(0);
//        printf("detected offset is: %i\n", offset);
//        delay_milliseconds(2000);
//
//
//        printf("setting offset to (detected_offset+10) ...\n");
//        i_motorcontrol.set_offset_value(offset+10);
//        delay_milliseconds(2000);
//
//
//        offset=i_motorcontrol.set_calib(0);
//        printf("set offset is: %i\n", offset);
//        delay_milliseconds(2000);
//
//
//        printf("Enabling torque controller ...\n");
//        i_motorcontrol.set_torque_control_enabled();
//        delay_milliseconds(5000);
//
//
//        for(int i=0;i<=10;i++)
//        {
//            torque_ref = 100;
//            printf("sending positive torque command ...\n");
//            i_motorcontrol.set_torque(torque_ref);
//            delay_milliseconds(200);
//
//            torque_ref = -100;
//            printf("sending negative torque command ...\n");
//            i_motorcontrol.set_torque(torque_ref);
//            delay_milliseconds(200);
//        }
//
//
//        printf("sending zero torque command ...\n");
//        torque_ref = 0;
//        i_motorcontrol.set_torque(torque_ref);
//        delay_milliseconds(5000);
//
//
//        printf("Disabling torque controller ...\n");
//        i_motorcontrol.set_torque_control_disabled();
//        delay_milliseconds(5000);
//
//        delay_milliseconds(20000);
//
//
//    }


    i_motorcontrol.set_offset_value(2440);
    delay_milliseconds(2000);
    i_motorcontrol.set_torque_control_enabled();
    delay_milliseconds(1000);

    while(1) {
#pragma ordered
        select {
            case t when timerafter(ts + USEC_STD * position_control_config.control_loop_period) :> ts:

//                    printf("not activated\n");
                if (activate == 1) {
                        /* PID Controller */

                    i_motorcontrol.set_torque(int16_position_k);
//                    delay_microseconds(5);
//                    int16_position_k = i_motorcontrol.get_velocity_actual();
//                    delay_microseconds(5);
//                    int16_position_k = i_motorcontrol.get_position_actual();


                } // end control activated

//#ifdef USE_XSCOPE
                        xscope_int(TARGET_VELOCITY, int16_position_k);
                        xscope_int(REF_VELOCITY, int16_position_k);
//#endif

                break;

            case !isnull(i_hall) => i_hall.notification():

                switch (i_hall.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case !isnull(i_qei) => i_qei.notification():

                switch (i_qei.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case i_motorcontrol.notification():

                switch (i_motorcontrol.get_notification()) {
                    case MOTCTRL_NTF_CONFIG_CHANGED:
                        config_update_flag = 1;
                        break;
                    default:
                        break;
                }
                break;

            case i_position_control[int i].set_position(int in_target_position):

                    int16_position_k = in_target_position;

                break;

            case i_position_control[int i].set_torque_limit(int in_torque_limit):

                f1_torque_j_lim = in_torque_limit;

                break;

            case i_position_control[int i].get_position() -> int out_position:

                out_position = actual_position;
                break;

            case i_position_control[int i].get_target_position() -> int out_target_position:

                out_target_position = int16_position_k;
                break;

            case i_position_control[int i].set_position_control_config(ControlConfig in_params):

                position_control_config = in_params;
                config_update_flag = 1;
                break;

            case i_position_control[int i].get_position_control_config() ->  ControlConfig out_config:

                out_config = position_control_config;
                break;

            case i_position_control[int i].set_position_sensor(int in_sensor_used):

                position_control_config.feedback_sensor = in_sensor_used;
                int16_position_k = actual_position;
                config_update_flag = 1;

                break;

            case i_position_control[int i].check_busy() -> int out_activate:

                out_activate = activate;
                break;

            case i_position_control[int i].enable_position_ctrl():

//                if (position_control_config.feedback_sensor == QEI_SENSOR && !isnull(i_qei)) {
//                    actual_position = i_qei.get_qei_position_absolute();
//                } else {
//                    actual_position = i_motorcontrol.get_position_actual();
//                }
//                target_position = actual_position;
//                while (1) {
//                    if (i_motorcontrol.check_busy() == INIT) { //__check_commutation_init(c_commutation);
//#ifdef debug_print
//                        printstrln("position_ctrl_service: commutation initialized");
//#endif
//                        if (i_motorcontrol.get_fets_state() == 0) { // check_fet_state(c_commutation);
//                            i_motorcontrol.set_fets_state(1);
//                            delay_milliseconds(2);
//                        } else {
//                            i_motorcontrol.set_fets_state(1);
//                        }
//
//                        break;
//                    }
//                }
//                if (activate == 0) {
//                    t :> ts;
//                    T_k_1n = ts - 500*USEC_STD;// fake older T_k_1n to avoid error in filters
//                    ts -= USEC_STD * position_control_config.control_loop_period; //run the control loop right affter
//                }
                activate = 1;
#ifdef debug_print
                printstrln("position_ctrl_service: position control activated");
#endif
                break;

            case i_position_control[int i].disable_position_ctrl():

                activate = 0;
                i_motorcontrol.set_voltage(0); //set_commutation_sinusoidal(c_commutation, 0);
                error_position = 0;
                error_position_D = 0;
                error_position_I = 0;
                previous_error = 0;
                position_control_out = 0;
                i_motorcontrol.set_fets_state(0); // disable_motor(c_commutation);
                delay_milliseconds(30); //wait_ms(30, 1, ts); //
#ifdef debug_print
                printstrln("position_ctrl_service: position control disabled");
#endif
                break;
        }
    }
}
