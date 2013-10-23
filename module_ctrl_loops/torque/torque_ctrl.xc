/**
 * \file torque_ctrl.xc
 *
 *	Torque control rountine based on field oriented Torque control method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include "torque_ctrl.h"
#include <refclk.h>
#include <xscope.h>
#include <print.h>
#include "hall_qei.h"
#include <drive_config.h>
#include <internal_config.h>
#include <dc_motor_config.h>
#include "adc_client_ad7949.h"
#include "hall_client.h"
#include "qei_client.h"
#include "hall_qei.h"

//#define ENABLE_xscope_torq


int root_function(int arg);
int get_torque(chanend c_torque)
{
	int torque;
	c_torque <: 3;
	c_torque :> torque;
	return torque;
}

void set_torque(chanend c_torque, int torque)
{
	c_torque <: 2;
	c_torque <: torque;
	return;
}

void init_buffer(int buffer[], int length)
{
	int i;
	for(i = 0; i < length; i++)
	{
		buffer[i] = 0;
	}
	return;
}

void current_filter(chanend c_adc, chanend c_current, chanend c_speed)
{
	#define filter_length 80
	int phase_a_raw = 0;
	int phase_b_raw = 0;
	int actual_speed = 0;
	int command;
	int buffer_phase_a[filter_length];
	int	buffer_phase_b[filter_length];
	timer ts, tc;
	unsigned int time;
	unsigned int time1;
	unsigned int time2;
	int fil_cnt = 0;
	int phase_a_filtered = 0;
	int phase_b_filtered = 0;
	int i = 0;
	int fl = filter_length;
	int j = 0;
	int flc = 3;
	int mod = 0;
	int mod_speed = 0;
	int filter_count = 0;


	int speed = 0 ;
	int sensor_select = HALL;

	int tim1, tim2, tim3;
	int adc_calib_start = 0;
	hall_par hall_params;
	init_hall_param(hall_params);

	while(1)
	{
		select
		{
			case c_current :> command:
				//printstrln("start adc calibration");
				adc_calib_start = 1;
				break;
		}
		if(adc_calib_start == 1)
			break;
	}

	do_adc_calibration_ad7949(c_adc);

	c_current<:1; // adc calib done
	init_buffer(buffer_phase_a, filter_length);
	init_buffer(buffer_phase_b, filter_length);
	ts :> time;
	tc :> time1;

	while(1)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time+5556) :> time: // .05 ms
			{phase_a_raw , phase_b_raw}= get_adc_calibrated_current_ad7949(c_adc);
		//	xscope_probe_data(0, phase_a_raw);
			buffer_phase_a[fil_cnt] = phase_a_raw;
			buffer_phase_b[fil_cnt] = phase_b_raw;

			fil_cnt = (fil_cnt+1)%fl;
			phase_a_filtered = 0;
			phase_b_filtered = 0;
			j=0;
			for(i=0; i<flc; i++)
			{
				mod = (fil_cnt - 1 - j)%fl;
				if(mod<0)
				mod = fl + mod;
				phase_a_filtered += buffer_phase_a[mod];
				phase_b_filtered += buffer_phase_b[mod];
				j++;
			}
			phase_a_filtered /= flc;
			phase_b_filtered /= flc;
		//	xscope_probe_data(0, phase_a_filtered);
		//	xscope_probe_data(1, phase_b_filtered);

			filter_count++;
			if(filter_count == 10)
			{
				filter_count = 0;
				//actual_speed = get_hall_velocity(c_hall_p3, hall_params);
			//	xscope_probe_data(0, actual_speed);
				mod_speed = actual_speed;
				if(actual_speed < 0)
					mod_speed = 0 - actual_speed;

				if(mod_speed <= 100)
				{
					flc = 50;
				}
				else if(mod_speed > 100 && mod_speed <= 800)
				{
					flc = 20;
				}
				else if(mod_speed >= 800)
				{
					flc = 3;
				}
			}//
			break;

			case tc when timerafter(time1+MSEC_STD+250) :> time1: // .05 ms
				if(sensor_select == HALL)
				{
					c_speed <: 2;
					master{	c_speed :> actual_speed; }
					//actual_speed = get_hall_velocity(c_hall, hall_params);
				}
				else if(sensor_select == QEI)
				{
					c_speed <: 2;
					master{	c_speed :> actual_speed; }
				}
				break;

			case c_current :> command:
				c_current <: phase_a_filtered;
				c_current <: phase_b_filtered;
				break;
		}
	}
}

void send_torque_init_state(chanend c_torque_ctrl, int init_state)
{
	int command;
	select
	{
		case c_torque_ctrl:> command:
			if(command == CHECK_BUSY)
			{
				c_torque_ctrl <: init_state;
			}
			break;
	}
}

void _torque_ctrl(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		chanend c_current, chanend c_speed, chanend sync_output, chanend c_commutation, \
		chanend c_hall, chanend c_qei, chanend c_torque_ctrl)
{
	#define filter_dc 80 //80 27
	int actual_speed = 0;
	int command;

	timer tc;
	unsigned int time;
	unsigned int time1;
	unsigned int time2;
	int phase_a_filtered = 0;
	int phase_b_filtered = 0;

	// Torque control variables
	int angle = 0;
	int sin = 0;
	int cos = 0;
	int alpha = 0;
	int beta = 0;
	int Id = 0;
	int Iq = 0;
	int phase_1 = 0;
	int phase_2 = 0;
	int buffer_Id[filter_dc];
	int buffer_Iq[filter_dc];

	int i1 = 0;
	int j1 = 0;
	int mod1 = 0;

	int iq_filtered = 0;
	int id_filtered = 0;
	int fdc = filter_dc;
	int fil_cnt_dc = 0;
	int fldc = filter_dc/hall_params.pole_pairs;
	int speed = 0 ;

	int actual_torque = 0;
	int target_torque = 0;  // (5 * DC900_RESOLUTION)/2
	int absolute_torque = 0;

	int error_torque = 0;
	int error_torque_integral = 0;
	int error_torque_derivative = 0;
	int error_torque_previous = 0;
	int torque_control_output = 0;

	unsigned int received_command = 0;
	int init_state = INIT_BUSY;
	int commutation_init = INIT_BUSY;

	int sensor_select = HALL;

	int qei_counts_per_hall ;
	qei_velocity_par qei_velocity_params;
	int qei_velocity = 0;
	int start_flag = 0;
	int offset_fw_flag = 0;
	int offset_bw_flag = 0;
	init_qei_velocity_params(qei_velocity_params);


	qei_counts_per_hall= qei_params.real_counts/ hall_params.pole_pairs;
	init_buffer(buffer_Id, filter_dc);
	init_buffer(buffer_Iq, filter_dc);




	while(1)
	{
		if(commutation_init == INIT_BUSY)
		{
//		 printstrln("initialized commutation check");
			 commutation_init = __check_commutation_init(c_commutation);
			 if(commutation_init == INIT)
			 {
				 c_current <: 1;
				 break;
			 }
		}
		//send_torque_init_state( c_torque_ctrl,  init_state);
	}
	while(1)
	{
		#pragma ordered
		select
		{
			case c_current :> command:
				//printstrln("adc calibrated");
				start_flag = 1;
				break;
			case c_torque_ctrl:> command:
				if(command == CHECK_BUSY)
				{
					c_torque_ctrl <: init_state;
				}
				break;
		}
		//send_torque_init_state( c_torque_ctrl,  init_state);
		if(start_flag == 1)
			break;
	}

	init_state = INIT;
	tc :> time1;
	while(1)
	{
		#pragma ordered
		select
		{
			case c_speed :> command:
				slave{c_speed <: actual_speed;}
				break;

			case tc when timerafter(time1 + MSEC_STD) :> time1:
				c_current <: 2;
				c_current :> phase_a_filtered;
				c_current :> phase_b_filtered;

				if(sensor_select == HALL)
				{
					angle = get_hall_position(c_hall) >> 2; //  << 10 ) >> 12
					actual_speed = get_hall_velocity(c_hall, hall_params);
					select
					{
						case c_speed :> command:
							slave{c_speed <: actual_speed;}
							break;
					}
				}
				else if(sensor_select == QEI)
				{
					//angle = (get_sync_position ( sync_output ) <<10)/qei_counts_per_hall; //synced input old
					{angle, offset_fw_flag, offset_bw_flag} = get_qei_sync_position(c_qei);
					angle = (angle <<10)/qei_counts_per_hall;
					actual_speed = get_qei_velocity( c_qei, qei_params, qei_velocity_params);//

					select
					{
						case c_speed :> command:
							slave{c_speed <: actual_speed;}
							break;
					}

				}

				phase_1 = 0 - phase_a_filtered;
				phase_2 = 0 - phase_b_filtered;


				xscope_probe_data(0, phase_a_filtered);
				//				xscope_probe_data(1, phase_b_filtered);
				alpha = phase_1;
				beta = (phase_1 + 2*phase_2); 			// beta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
				beta *= 37838;
				beta /= 65536;
				beta = -beta;

				// ==== Park transform ====

				sin = sine_table_expanded(angle);
				cos = sine_table_expanded((256 - angle)&1023);

				Id = ( alpha * cos + beta * sin ) /16384;
				Iq = ( beta * cos  - alpha * sin ) /16384;

				buffer_Id[fil_cnt_dc] = Id;
				buffer_Iq[fil_cnt_dc] = Iq;
				fil_cnt_dc = (fil_cnt_dc+1)%fdc;

				id_filtered = 0;
				iq_filtered = 0;

				j1=0;
				for(i1=0; i1<fldc; i1++)
				{
					mod1 = (fil_cnt_dc - 1 - j1)%fdc;
					if(mod1<0)
					mod1 = fdc + mod1;
					id_filtered += buffer_Id[mod1];
					iq_filtered += buffer_Iq[mod1];
					j1++;
				}
				id_filtered /= fldc;
				iq_filtered /= fldc;

				actual_torque = root_function(iq_filtered * iq_filtered + id_filtered * id_filtered);
				xscope_probe_data(1, actual_torque);
				#ifdef ENABLE_xscope_torq
				xscope_probe_data(0, actual_torque);
				xscope_probe_data(1, target_torque);
				#endif


				absolute_torque = target_torque;
				if(target_torque < 0)
					absolute_torque = 0 - target_torque;
				error_torque = absolute_torque - actual_torque; //350
				error_torque_integral = error_torque_integral + error_torque;
				error_torque_derivative = error_torque - error_torque_previous;

				#ifdef ENABLE_xscope_torq
				//xscope_probe_data(2, error_torque);
				#endif

				if(error_torque_integral > torque_ctrl_params.Integral_limit)
				{
					error_torque_integral = torque_ctrl_params.Integral_limit;
				}
				else if(error_torque_integral < -torque_ctrl_params.Integral_limit)
				{
					error_torque_integral = 0 - torque_ctrl_params.Integral_limit;
				}


				torque_control_output = (torque_ctrl_params.Kp_n * error_torque)/torque_ctrl_params.Kp_d\
						+ (torque_ctrl_params.Ki_n * error_torque_integral)/torque_ctrl_params.Ki_d\
						+ (torque_ctrl_params.Kd_n  * error_torque_derivative)/torque_ctrl_params.Kd_d;

#ifdef ENABLE_xscope_torq
				//xscope_probe_data(3, integral_member);
				//xscope_probe_data(4, torque_control_output);
#endif
				error_torque_previous = error_torque;


				if(target_torque >=0)
				{
					if(torque_control_output >= torque_ctrl_params.Control_limit) {
						torque_control_output = torque_ctrl_params.Control_limit;
					}
					else if(torque_control_output < 0){
						torque_control_output = 0;
					}
				}
				else
				{


					torque_control_output = 0 - torque_control_output;
					if(torque_control_output > 0)
						torque_control_output = 0;
					if(torque_control_output <= -torque_ctrl_params.Control_limit) {
						torque_control_output = 0 - torque_ctrl_params.Control_limit;
					}
					//else
				}
//printstrln("printloop");

				set_commutation_sinusoidal(c_commutation, torque_control_output);
				break;

			case c_torque_ctrl:> command:
				if(command == 2)
				{
					c_torque_ctrl :> target_torque;
					if(target_torque > (5 * DC900_RESOLUTION)/2) //adc range // (5 * DC900_RESOLUTION)/2
					{
						target_torque = (5 * DC900_RESOLUTION)/2;
					}
					else if(target_torque < 0- (5 * DC900_RESOLUTION)/2)
					{
						target_torque = 0-(5 * DC900_RESOLUTION)/2;
					}
					xscope_probe_data(2, target_torque);
				}
				else if(command == 3)
				{
					c_torque_ctrl <: actual_torque;
				}
				else if(command == CHECK_BUSY)
				{
					c_torque_ctrl <: init_state;
				}
				break;
		}
	}
}

void torque_ctrl(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		chanend c_adc, chanend synced_out, chanend c_commutation, chanend c_hall, chanend c_qei, chanend c_torque_ctrl)
{
	chan c_current, c_speed;
	par
	{
		current_filter(c_adc, c_current, c_speed);
		_torque_ctrl(torque_ctrl_params, hall_params, qei_params, c_current, c_speed, synced_out, c_commutation, c_hall, c_qei, c_torque_ctrl);
	}
}

