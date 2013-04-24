/**************************************************************************
 * \file dc_motor_config.h
 *	Motor Control config file
 *
 * Please define your the motor specifications here
 *
 * All these initialisation functions :init_params_struct_all, init_hall and init_qei
 * need to be called to set up the variables for control module, hall sensor and quadrature
 * sensor modules "else operation is not guaranteed"
 *
 * You still need to tune the PI torque control params for your motor individually
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Pavan Kanajar <pkanajar@synapticon.com> & Martin Schwarz <mschwarz@synapticon.com>
 *
 * All code contained in this package under Synapticon copyright must be
 * licensing for any use from Synapticon. Please contact support@synapticon.com for
 * details of licensing.
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **************************************************************************/

#ifndef __DC_MOTOR_CONFIG__H__test
#define __DC_MOTOR_CONFIG__H__test

#define new

#pragma once

/*
 * define Motor Specific Constants
 */
#define POLE_PAIRS	8
#define GEAR_RATIO	156
#define MAX_NOMINAL_SPEED  4000   // in 1/min
#define MAX_NOMINAL_CURRENT  5    // in A
#define QEI_COUNT_MAX (1024 * 4)  // Max count of Quadrature Encoder


/*
 * define control parameters for PI TORQUE controller
 */
#define Torque_Kp_n 40                   // Kp = Kp_n/Kp_d
#define Torque_Kp_d 10
#define Torque_Ki_n 4					 // Ki = Ki_n/Ki_d
#define Torque_Ki_d 120
#define Torque_Integral_limit 10000
#define Max_torque_out 1200              // Max_Torque_out = Max Continuous torque * 264 / torque constant

/*
 * define control closing time the controller
 */
#define loop_timing 88 					//in USEC_FAST

/*
 * define optional PI controller parameters for field control
 */
#define Field_Kp_n 25                    // Kp = Kp_n/Kp_d
#define Field_Kp_d 10
#define Field_Ki_n 2					 // Ki = Ki_n/Ki_d
#define Field_Ki_d 100
#define Field_Integral_limit 10000

/**
 * \brief Struct definitions for the torque controller
 */
typedef struct S_Torque {
	int Kp_n, Kp_d;    					//Kp = Kp_n/Kp_d
	int Ki_n, Ki_d;						//Ki = Ki_n/Ki_d
	int Integral_limit;
	int Max_torque;
} torq_par;

/**
 * \brief struct definition for control loop time
 */
typedef struct S_Loop_time {
	int delay;
} loop_par;

/**
 * \brief Struct definitions for the field controller
 */
typedef struct S_Field{
	int Kp_n, Kp_d;						//Kp = Kp_n/Kp_d
	int Ki_n, Ki_d;						//Ki = Ki_n/Ki_d
	int Integral_limit;
} field_par;

/**
 * \brief struct definition for quadrature sensor
 */
typedef struct S_QEI {
	unsigned max_count;
} qei_par;

/**
 * \brief struct definition for hall sensor
 *
 */
typedef struct S_Hall {
	int pole_pairs;
	int gear_ratio;
} hall_par;

/**
 * \brief initialize QEI sensor
 *
 * \param q_max struct defines the max count for quadrature encoder (QEI)
 */
void init_qei(qei_par &q_max);

/**
 * \brief initialize hall sensor
 *
 * \param h_pole struct defines the pole-pairs and gear ratio
 */
void init_hall(hall_par &h_pole);

/**
 * \brief Initialize Struct for Controller parameter
 *
 * \param tor initializes the torque PI controller parameters
 * \param field initializes the field PI controller parameters
 * \param loop initializes the control loop time
 *
 */
void init_params_struct_all(torq_par &tor, field_par &field, loop_par &loop);

#endif
