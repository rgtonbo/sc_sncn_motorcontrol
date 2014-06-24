/**
 * \file bldc_motor_config.h
 * \brief Motor Control config file (define your the motor specifications here)
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 */

#include <internal_config.h>

#pragma once

/**
 * Define Motor Specific Constants (found in motor specification sheet)
 * Mandatory constants to be set
 */
#define POLE_PAIRS                  3               // Number of pole pairs
#define MAX_NOMINAL_SPEED           4000            // rpm
#define MAX_NOMINAL_CURRENT         2               // A
#define MOTOR_TORQUE_CONSTANT       72              // mNm/A

/**
 * If you have any gears added, specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (Mandatory)
 */
#define GEAR_RATIO                  1                    // if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION          16384               // 4 x Max count of Incremental Encoder (4X decoding - quadrature mode)

/* Position Sensor Types (select your sensor type here)
 * (HALL/ QEI) */
#define SENSOR_USED                 HALL

/* Define your Incremental Encoder type (QEI_INDEX/ QEI_WITH_NO_INDEX) */
#define QEI_SENSOR_TYPE             QEI_WITH_INDEX

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (NORMAL/INVERTED) */
#define QEI_SENSOR_POLARITY         NORMAL

/* Somanet IFM Internal Config:  Specifies the current sensor resolution per Ampere
 *  (DC300_RESOLUTION / DC100_RESOLUTION / OLD_DC300_RESOLUTION) */
#define IFM_RESOLUTION              DC100_RESOLUTION

/* Commutation offset (range 0-4095) (HALL sensor based commutation) */
#define COMMUTATION_OFFSET_CLK      683
#define COMMUTATION_OFFSET_CCLK     2731

/* Motor Winding type (STAR_WINDING/DELTA_WINDING) */
#define WINDING_TYPE                DELTA_WINDING

/* Changes direction of the motor drive  (1 /-1) */
#define POLARITY                    1

/* Profile defines (Mandatory for profile modes) */
#define MAX_PROFILE_VELOCITY        MAX_NOMINAL_SPEED
#define PROFILE_VELOCITY            1000                // rpm
#define MAX_ACCELERATION            4000                // rpm/s
#define PROFILE_ACCELERATION        2000                // rpm/s
#define PROFILE_DECELERATION        2000                // rpm/s
#define QUICK_STOP_DECELERATION     2000                // rpm/s
#define PROFILE_TORQUE_SLOPE        66                  // (desired torque_slope/torque_constant)  * IFM resolution


/* Control specific constants/variables */

/* Position Control (Mandatory if Position control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define POSITION_Kp_NUMERATOR       100
#define POSITION_Kp_DENOMINATOR     1000
#define POSITION_Ki_NUMERATOR       1
#define POSITION_Ki_DENOMINATOR     1200
#define POSITION_Kd_NUMERATOR       0
#define POSITION_Kd_DENOMINATOR     1000

#if(SENSOR_USED == HALL)
    #define MAX_POSITION_LIMIT      POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO       // ticks (max range: 2^30, limited for safe operation) qei/hall/any position sensor
    #define MIN_POSITION_LIMIT      -POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO      // ticks (min range: -2^30, limited for safe operation) qei/hall/any position sensor
#endif
#if(SENSOR_USED == QEI)
    #define MAX_POSITION_LIMIT      GEAR_RATIO*ENCODER_RESOLUTION    // ticks (max range: 2^30, limited for safe operation)
    #define MIN_POSITION_LIMIT      -GEAR_RATIO*ENCODER_RESOLUTION   // ticks (min range: -2^30, limited for safe operation)
#endif

/* Torque Control (Mandatory if Torque control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define TORQUE_Kp_NUMERATOR         2
#define TORQUE_Kp_DENOMINATOR       20
#define TORQUE_Ki_NUMERATOR         1
#define TORQUE_Ki_DENOMINATOR       110
#define TORQUE_Kd_NUMERATOR         0
#define TORQUE_Kd_DENOMINATOR       10

/* Velocity Control (Mandatory if Velocity control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define VELOCITY_Kp_NUMERATOR       1
#define VELOCITY_Kp_DENOMINATOR     15
#define VELOCITY_Ki_NUMERATOR       2
#define VELOCITY_Ki_DENOMINATOR     100
#define VELOCITY_Kd_NUMERATOR       0
#define VELOCITY_Kd_DENOMINATOR     1

#define VELOCITY_FILTER_SIZE        8   //default (could be changed upto 16)

/* Define Homing method (HOMING_POSITIVE_SWITCH/HOMING_NEGATIVE_SWITCH)
 * this specifies direction for the node to find the home switch */
#define HOMING_METHOD               HOMING_POSITIVE_SWITCH





/*  DO NOT EDIT FROM HERE !!!*/





/**
 * \brief initialize Velocity sensor filter
 *
 * \param sensor_filter_par struct defines the velocity filter params
 */
void init_sensor_filter_param(filter_par &sensor_filter_par) ;

/**
 * \brief initialize cyclic synchronous velocity params
 *
 * \param csv_params struct defines cyclic synchronous velocity params
 */
void init_csv_param(csv_par &csv_params);

/**
 * \brief initialize cyclic synchronous position params
 *
 * \param csp_params struct defines cyclic synchronous position params
 */
void init_csp_param(csp_par &csp_params);

/**
 * \brief initialize cyclic synchronous torque params
 *
 * \param cst_params struct defines cyclic synchronous torque params
 */
void init_cst_param(cst_par &cst_params);

/**
 * \brief initialize profile position params
 *
 * \param pp_params struct defines profileposition params
 */
void init_pp_params(pp_par &pp_params);

/**
 * \brief initialize profile velocity params
 *
 * \param pv_params struct defines profile velocity params
 */
void init_pv_params(pv_par &pv_params);

/**
 * \brief initialize profile torque params
 *
 * \param pt_params struct defines profile torque params
 */
void init_pt_params(pt_par &pt_params);

/**
 * \brief initialize torque control PID params
 *
 * \param torque_ctrl_params struct defines torque control PID params
 */
void init_torque_control_param(ctrl_par &torque_ctrl_params);

/**
 * \brief initialize velocity control PID params
 *
 * \param velocity_ctrl_params struct defines velocity control PID params
 */
void init_velocity_control_param(ctrl_par &velocity_ctrl_params);

/**
 * \brief initialize position control PID params
 *
 * \param position_ctrl_params struct defines position control PID params
 */
void init_position_control_param(ctrl_par &position_ctrl_params);
