/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
//#include <CORE_BOARD_REQUIRED>
//#include <IFM_BOARD_REQUIRED>
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC1K-rev-c3.bsp>


/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */

//#include <pwm_service.h>
#include <pwm_server.h>
#include <hall_service.h>
#include <adc_service.h>
#include <user_config.h>
#include <tuning.h>
#include <torque_control.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
BISSPorts biss_ports = SOMANET_IFM_BISS_PORTS;
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
AMSPorts ams_ports = SOMANET_IFM_AMS_PORTS;
#else
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
#endif

#define POSITION_LIMIT 0 //+/- 4095

void velocity_client(interface BISSInterface client i_biss)
{
    while(1) {
        int velocity = i_biss.get_biss_velocity();
        xscope_int(VELOCITY, velocity);

        delay_milliseconds(1);
    }
}

int main(void) {

    // Motor control interfaces
    chan c_pwm_ctrl, c_adctrig; // pwm channels

    interface WatchdogInterface i_watchdog[2];
    interface update_pwm i_update_pwm;
    interface ADCInterface i_adc[2];
    interface MotorcontrolInterface i_motorcontrol[4];
    interface PositionControlInterface i_position_control[3];
    interface TuningInterface i_tuning;
    interface BrakeInterface i_brake;
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
    interface BISSInterface i_biss[5];
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
    interface AMSInterface i_ams[5];
#else
    interface HallInterface i_hall[5];
#endif

    par
    {
        /* WARNING: only one blocking task is possible per tile. */
        /* Waiting for a user input blocks other tasks on the same tile from execution. */
        on tile[APP_TILE]: run_offset_tuning(POSITION_LIMIT, i_motorcontrol[0], i_tuning, i_position_control[1]);

        /* Display phases currents */
//        on tile[IFM_TILE]: adc_client(i_adc[1]);
//        on tile[IFM_TILE]: velocity_client(i_biss[1]);
//        on tile[IFM_TILE]: tuning_service(i_motorcontrol[1], i_adc[1], i_biss[1]);
        on tile[APP_TILE_2]: tuning_service(i_tuning, i_motorcontrol[1], i_adc[1], i_position_control[0], null, i_biss[1], null);
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
//        on tile[APP_TILE_2]: tuning_service(i_tuning, i_motorcontrol[1], i_adc[1], i_position_control[0], null, i_biss[1], null);
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
//        on tile[APP_TILE_2]: tuning_service(i_tuning, i_motorcontrol[1], i_adc[1], i_position_control[0], null, null, i_ams[1]);
#else
//        on tile[APP_TILE_2]: tuning_service(i_tuning, i_motorcontrol[1], i_adc[1], i_position_control[0], i_hall[1], null, null);
#endif

        on tile[APP_TILE_2]:
        /* Position Control Loop */
        {
            ControlConfig position_control_config;
            position_control_config.feedback_sensor = MOTOR_FEEDBACK_SENSOR;
            position_control_config.Kp_n = POSITION_Kp;    // Divided by 10000
            position_control_config.Ki_n = POSITION_Ki;    // Divided by 10000
            position_control_config.Kd_n = POSITION_Kd;    // Divided by 10000
            position_control_config.control_loop_period = CONTROL_LOOP_PERIOD; //us
            /* Control Loop */
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
            position_control_service(position_control_config, null, null, i_biss[2], null, i_motorcontrol[3],
                    i_position_control);
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
//            position_control_service(position_control_config, null, null, null, i_ams[2], i_motorcontrol[3],
//                    i_position_control);
#else
//            position_control_service(position_control_config, i_hall[2], null, null, null, i_motorcontrol[3],
//                    i_position_control);
#endif
        }


        on tile[IFM_TILE]:
        {
            par
            {
                {
                    /* Watchdog Service */
                    delay_milliseconds(500);
                    watchdog_service(wd_ports,i_watchdog);
                }

                {
                    pwm_config(pwm_ports);

//                    pwm_check(pwm_ports);//checks if pulses can be generated on pwm ports or not
                    delay_milliseconds(1000);
                    pwm_service_task(_MOTOR_ID, pwm_ports, i_update_pwm);
                }

                /* ADC Service */
                {
                    delay_milliseconds(1500);
                    adc_service(adc_ports, null/*c_trigger*/, i_adc /*ADCInterface*/, i_watchdog[1]);

                }


#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
                /* BiSS service */
                {
                    BISSConfig biss_config;
                    biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                    biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                    biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                    biss_config.status_length = BISS_STATUS_LENGTH;
                    biss_config.crc_poly = BISS_CRC_POLY;
                    biss_config.pole_pairs = POLE_PAIRS;
                    biss_config.polarity = BISS_POLARITY;
                    biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                    biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                    biss_config.timeout = BISS_TIMEOUT;
                    biss_config.max_ticks = BISS_MAX_TICKS;
                    biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                    biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                    biss_service(biss_ports, biss_config, null, i_biss);
                }
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
                /* AMS Rotary Sensor Service */
                {
                    AMSConfig ams_config;
                    ams_config.factory_settings = 1;
                    ams_config.polarity = AMS_POLARITY;
                    ams_config.hysteresis = 1;
                    ams_config.noise_setting = AMS_NOISE_NORMAL;
                    ams_config.uvw_abi = 0;
                    ams_config.dyn_angle_comp = 0;
                    ams_config.data_select = 0;
                    ams_config.pwm_on = AMS_PWM_OFF;
                    ams_config.abi_resolution = 0;
                    ams_config.resolution_bits = AMS_RESOLUTION;
                    ams_config.offset = AMS_OFFSET;
                    ams_config.pole_pairs = POLE_PAIRS;
                    ams_config.max_ticks = 0x7fffffff;
                    ams_config.cache_time = AMS_CACHE_TIME;
                    ams_config.velocity_loop = AMS_VELOCITY_LOOP;
                    ams_service(ams_ports, ams_config, null, i_ams);
                }
#else
                /* Hall sensor Service */
                {
                    HallConfig hall_config;
                    hall_config.pole_pairs = POLE_PAIRS;
                    hall_service(hall_ports, hall_config, null, i_hall);
                }
#endif

                /* Motor Control Service */
                {
                    delay_milliseconds(2000);

                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.polarity_type = NORMAL_POLARITY;
                    motorcontrol_config.commutation_method = FOC;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] = COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;
#if(MOTOR_COMMUTATION_SENSOR == BISS_SENSOR)
                    Motor_Control_Service( fet_driver_ports, motorcontrol_config, c_pwm_ctrl, i_adc[0],
                            null, null, i_biss[0], null,
                            i_watchdog[0], null, i_motorcontrol, i_update_pwm);

//                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
//                                         c_pwm_ctrl, i_adc[0], null, null, i_biss[0], null, i_watchdog[0], null, i_motorcontrol);
#elif(MOTOR_COMMUTATION_SENSOR == AMS_SENSOR)
//                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
//                                         c_pwm_ctrl, i_adc[0], null, null, null, i_ams[0], i_watchdog[0], null, i_motorcontrol);
#else
//                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
//                                         c_pwm_ctrl, i_adc[0], i_hall[0], null, null, null, i_watchdog[0], null, i_motorcontrol);
#endif
                }
            }
        }
    }

    return 0;
}
