
/**
 *
 * \file ecat_motor_drive.xc
 *
 *	Ethercat Motor Drive Server
 *
 * Copyright (c) 2013, Synapticon GmbH
 * All rights reserved.
 * Author: Martin Schwarz <mschwarz@synapticon.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include <ecat_motor_drive.h>
#include <xscope.h>
#include <print.h>

extern int position_factor(int gear_ratio, int qei_max_real, int pole_pairs, int sensor_used);

void xscope_initialise()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_velocity", XSCOPE_INT,	"n",
							XSCOPE_CONTINUOUS, "1 target_velocity", XSCOPE_INT, "n");

		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}
/*core 0/1/2 only*/
//#define ENABLE_xscope_main
void ecat_motor_drive(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall,\
		chanend c_qei, chanend c_home, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl)
{
	int i = 0;
	int mode=40;
	int core_id = 0;
	int steps = 0;
	chan c_dummy;

	int target_torque = 0;
	int actual_torque = 0;
	int target_velocity = 0;
	int actual_velocity = 0;
	int target_position = 0;
	int actual_position = 0;

	int position_ramp = 0;
	int prev_position = 0;

	int velocity_ramp = 0;
	int prev_velocity = 0;

	int torque_ramp = 0;
	int prev_torque = 0;

	int nominal_speed;
	timer t;

	int init = 0;
	int op_set_flag = 0;
	int op_mode = 0;

	cst_par cst_params;
	ctrl_par torque_ctrl_params;
	csv_par csv_params;
	ctrl_par velocity_ctrl_params;
	qei_par qei_params;
	hall_par hall_params;
	ctrl_par position_ctrl_params;
	csp_par csp_params;
	pp_par pp_params;
	pv_par pv_params;
	pt_par pt_params;
	commutation_par commutation_params;
	ctrl_proto_values_t InOut;

	int setup_loop_flag = 0;
	int sense;

	int ack = 0;
	int quick_active = 0;
	int mode_quick_flag = 0;
	int shutdown_ack = 0;
	int sensor_select;

	int direction;

	int comm_active = 0;
	unsigned int comm_inactive_time_stamp;
	unsigned int c_time;
	unsigned int inactive_delay = 100*MSEC_STD;
	int comm_inactive_flag = 0;
	int inactive_timeout_flag = 0;

	unsigned int time;
	int state;
	int statusword;
	int controlword;

	int status=0;
	int tmp=0;
	int precision;
	int precision_factor;

	int torque_offstate = 0;
	int mode_selected = 0;
	check_list checklist;

	int home_velocity = 0;
	int home_acceleration = 0;
	int h_active = 0;

	int limit_switch = -1; // positive negative limit switches
	int reset_counter = 0;

	unsigned int h_time;
	int home_state = 0;
	int safety_state = 0;
	int capture_position = 0;
	int current_position = 0;
	int home_offset = 0;
	int end_state = 0;

	int ctrl_state;
	int limit_switch_type;
	int homing_method;
	int polarity;
	int homing_done = 0;
	state 		= init_state(); 			//init state
	checklist 	= init_checklist();
	InOut 		= init_ctrl_proto();

	init_cst_param(cst_params);
	init_csv_param(csv_params);
	init_csp_param(csp_params);
	init_hall_param(hall_params);
	init_pp_params(pp_params);
	init_pv_params(pv_params);
	init_pt_params(pt_params);
	init_qei_param(qei_params);
	init_velocity_control_param(velocity_ctrl_params);

#ifdef ENABLE_xscope_main
	xscope_initialise();
#endif
	t:>time;
	while(1)
	{
		comm_active = ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);

		if(comm_active == 0)
		{
			if(comm_inactive_flag == 0)
			{
				comm_inactive_flag = 1;
				t :> c_time;
			}
			else if(comm_inactive_flag == 1)
			{
				t :> comm_inactive_time_stamp;
				if(comm_inactive_time_stamp - c_time> 1*SEC_STD)
				{
					//printstrln("comm inactive timeout");
					t :> time;
					t when timerafter(time+2*SEC_STD) :> time;
					inactive_timeout_flag = 1;
				}
			}
		}
		else if(comm_active >= 1)
		{
			comm_inactive_flag = 0;
			inactive_timeout_flag = 0;
		}

		if(inactive_timeout_flag == 1)
		{

			if(op_mode == CST || op_mode == TQ)
			{
				actual_torque = get_torque(cst_params, c_torque_ctrl);
				steps = init_linear_profile(0, actual_torque, pt_params.profile_slope, pt_params.profile_slope, cst_params.max_torque);
				i = 0;
				while(i < steps)
				{
					target_torque = linear_profile_generate(i);
					set_torque(target_torque, cst_params, c_torque_ctrl);
					actual_torque = get_torque(cst_params, c_torque_ctrl);
					send_actual_torque(actual_torque*cst_params.polarity, InOut);

					t when timerafter(time + MSEC_STD) :> time;
					i++;
				}
				if(i == steps|| steps <=0)
				{
					//		printstrln("stop");
					//op_mode = 256;
					//controlword = 0;
					//mode_selected = 0;
					//while(1);
				}
			}

			if(op_mode == CSV || op_mode == PV)
			{
				actual_velocity = get_velocity(c_velocity_ctrl);
				if(op_mode == CSV)
					steps = init_quick_stop_velocity_profile(actual_velocity, csv_params.max_acceleration);
				else if(op_mode == PV)
					steps = init_quick_stop_velocity_profile(actual_velocity, pv_params.quick_stop_deceleration);
				//printintln(steps);
				i = 0;
				while(i < steps)
				{
					target_velocity = quick_stop_velocity_profile_generate(i);
					if(op_mode == CSV)
					{
						set_velocity( max_speed_limit(target_velocity, csv_params.max_motor_speed), c_velocity_ctrl );
						actual_velocity = get_velocity(c_velocity_ctrl);
						send_actual_velocity(actual_velocity * csv_params.polarity, InOut);
					}
					else if(op_mode == PV)
					{
						set_velocity( max_speed_limit(target_velocity, pv_params.max_profile_velocity), c_velocity_ctrl );
						actual_velocity = get_velocity(c_velocity_ctrl);
						send_actual_velocity(actual_velocity * pv_params.polarity, InOut);
					}
					t when timerafter(time + MSEC_STD) :> time;
					i++;
				}
				if(i == steps || steps <=0)
				{
					//printstrln("stop");
					//op_mode = 256;
					//controlword = 0;
					//mode_selected = 0;
					//while(1);
				}

			}
			else if(op_mode == CSP || op_mode == PP)
			{
				actual_velocity = get_hall_velocity(c_hall, hall_params);
				actual_position = get_position(c_position_ctrl);

				if(!(actual_velocity<100 && actual_velocity>-100))
				{
					if(actual_velocity < 0)
					{
						actual_velocity = 0-actual_velocity;
						sense = -1;
					}
					if(op_mode == CSP)
						steps = init_quick_stop_position_profile( (actual_velocity*360)/(60*hall_params.gear_ratio), actual_position, csp_params.base.max_acceleration);
					else if(op_mode == PP)
						steps = init_quick_stop_position_profile( (actual_velocity*360)/(60*hall_params.gear_ratio), actual_position, pp_params.base.quick_stop_deceleration);
					i = 0;
					mode_selected = 3;// non interruptible mode
					mode_quick_flag = 0;
				}
				if(sensor_select == HALL)
				{
					{actual_position, sense} = get_hall_position_absolute(c_hall);
				}
				else if(sensor_select == QEI)
				{
					{actual_position, sense} = get_qei_position_absolute(c_qei);
				}
				while(i < steps)
				{
					target_position   =   quick_stop_position_profile_generate(i, sense);
					if(op_mode == CSP)
					{
						set_position( position_limit( target_position ,				\
								csp_params.max_position_limit * 10000  , 			\
								csp_params.min_position_limit * 10000) , c_position_ctrl);
						actual_position = get_position(c_position_ctrl);
						send_actual_position(actual_position * csp_params.base.polarity, InOut);
					}
					else if(op_mode == PP)
					{
						set_position( position_limit( target_position ,						\
								pp_params.software_position_limit_max * 10000  , 			\
								pp_params.software_position_limit_min * 10000) , c_position_ctrl);
						actual_position = get_position(c_position_ctrl);
						send_actual_position(actual_position * pp_params.base.polarity, InOut);
					}
					t when timerafter(time + MSEC_STD) :> time;
					i++;
				}
				if(i == steps || steps <=0)
				{
					//printstrln("stop");
					//op_mode = 256;
				//	controlword = 0;
				//	mode_selected = 0;
					//while(1);
				}
			}
			mode_selected = 0;
			setup_loop_flag = 0;
			op_set_flag = 0;
			op_mode = 256;
		}

		if(comm_inactive_flag == 0)
		{
			controlword = InOut.control_word;
			update_checklist(checklist, mode, c_signal, c_hall, c_qei, c_dummy, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);

			state = get_next_state(state, checklist, controlword);
			statusword = update_statusword(statusword, state, ack, quick_active, shutdown_ack);
			InOut.status_word = statusword;
			//printintln(controlword);

			if(setup_loop_flag == 0)
			{
				if(controlword == 6)
				{
					coe_out <: CAN_GET_OBJECT;
					coe_out <: CAN_OBJ_ADR(0x60b0, 0);
					coe_out :> tmp;
					status= (unsigned char)(tmp&0xff);
					if (status == 0) {
						coe_out <: CAN_SET_OBJECT;
						coe_out <: CAN_OBJ_ADR(0x60b0, 0);
						status = 0xaf;
						coe_out <: (unsigned)status;
						coe_out :> tmp;
						if (tmp == status) {
							t:>time;
							t when timerafter(time + 500*MSEC_STD) :> time;
							//printstr("successfully set status\n");
							InOut.operation_mode_display = 105;

						}
					}
					else if(status == 0xaf)
					{
						InOut.operation_mode_display = 105;
					}
				}
				if(controlword == 5)
				{
					update_hall_param_ecat(hall_params, coe_out);
					update_qei_param_ecat(qei_params, coe_out);
					update_commutation_param_ecat(commutation_params, coe_out);
					sensor_select = sensor_select_sdo(coe_out);
					nominal_speed = speed_sdo_update(coe_out);
					update_pp_param_ecat(pp_params, coe_out);
					polarity = pp_params.base.polarity;
					qei_params.poles = hall_params.pole_pairs;
				/*	printintln(qei_params.gear_ratio);
					printintln(qei_params.index);
					printintln(qei_params.max_count);
					printintln(qei_params.real_counts);
					printintln(hall_params.gear_ratio);
					printintln(hall_params.pole_pairs);
					printintln(commutation_params.hall_offset_clk);
					printintln(commutation_params.hall_offset_cclk);
					printintln(commutation_params.winding_type);*/
					//config_sdo_handler( coe_out);
					{homing_method, limit_switch_type} = homing_sdo_update(coe_out);
					if(homing_method == HOMING_NEGATIVE_SWITCH)
						limit_switch = -1;
					else if(homing_method == HOMING_POSITIVE_SWITCH)
						limit_switch = 1;



					//printintln(homing_method); printintln(limit_switch);
					//printintln(limit_switch_type);

					if(sensor_select == HALL)
					{
						precision_factor = position_factor(hall_params.gear_ratio, 1, hall_params.pole_pairs, sensor_select);
						precision = HALL_PRECISION;
					}
					else if(sensor_select == QEI)
					{
						precision_factor = position_factor(qei_params.gear_ratio, qei_params.real_counts, 1, sensor_select);
						precision = QEI_PRECISION;
					}
					set_hall_param_ecat(c_hall, hall_params);
					if(homing_done == 0)
						set_qei_param_ecat(c_qei, qei_params);
					set_commutation_param_ecat(c_signal, hall_params, qei_params, commutation_params, nominal_speed);

					setup_loop_flag = 1;
					op_set_flag = 0;
				}
			}
			if(mode_selected == 0)
			{
				switch(InOut.operation_mode)
				{
					case HM:
						//printstrln("Homing mode op");
						if(op_set_flag == 0)
						{
							ctrl_state = check_torque_ctrl_state(c_torque_ctrl);
							if(ctrl_state == 1)
								shutdown_torque_ctrl(c_torque_ctrl);
							//ctrl_state = check_position_ctrl_state(c_position_ctrl);
							//if(ctrl_state == 1)
							//	shutdown_position_ctrl(c_position_ctrl);
							init = init_velocity_control(c_velocity_ctrl);
						}
						if(init == INIT)
						{
							op_set_flag = 1;
							//enable_velocity_ctrl(c_velocity_ctrl);
							mode_selected = 1;
							op_mode = HM;
							steps = 0;
							mode_quick_flag = 10;
							ack = 0;
							shutdown_ack = 0;

						//	update_velocity_ctrl_param_ecat(velocity_ctrl_params, coe_out);
						//	sensor_select = sensor_select_sdo(coe_out);
						/*	if(sensor_select == HALL)
							{
								set_velocity_ctrl_hall_param(hall_params, c_velocity_ctrl);
							}
							else if(sensor_select == QEI)
							{
								set_velocity_ctrl_qei_param(qei_params, c_velocity_ctrl);
							}
							set_velocity_ctrl_param(velocity_ctrl_params, c_velocity_ctrl);*/
							set_velocity_sensor(QEI, c_velocity_ctrl); //QEI
							InOut.operation_mode_display = HM;
						}
						break;
					case PP:
						if(op_set_flag == 0)
						{
							ctrl_state = check_position_ctrl_state(c_position_ctrl);
							if(ctrl_state == 0)
								shutdown_position_ctrl(c_position_ctrl);
							ctrl_state = check_torque_ctrl_state(c_torque_ctrl);
							if(ctrl_state == 1)
								shutdown_torque_ctrl(c_torque_ctrl);
							ctrl_state = check_velocity_ctrl_state(c_velocity_ctrl);
							if(ctrl_state == 1)
								shutdown_velocity_ctrl(c_velocity_ctrl);

							init = init_position_control(c_position_ctrl);
						}
						if(init == INIT)
						{
							op_set_flag = 1;
							//enable_position_ctrl(c_position_ctrl);
							mode_selected = 1;
							op_mode = PP;
							steps = 0;
							mode_quick_flag = 10;
							ack = 0;
							shutdown_ack = 0;

							update_position_ctrl_param_ecat(position_ctrl_params, coe_out);
							sensor_select = sensor_select_sdo(coe_out);
							update_pp_param_ecat(pp_params, coe_out);
							/*
							printintln(position_ctrl_params.Control_limit);
							printintln(position_ctrl_params.Integral_limit);
							printintln(position_ctrl_params.Kd_d);
							printintln(position_ctrl_params.Ki_d);
							printintln(position_ctrl_params.Kp_d);
							printintln(position_ctrl_params.Kp_n);
							printintln(position_ctrl_params.Ki_n);
							printintln(position_ctrl_params.Kd_n);
							printintln(pp_params.base.max_profile_velocity);
							printintln(pp_params.profile_velocity);
							printintln(pp_params.base.profile_acceleration);
							printintln(pp_params.base.profile_deceleration);
							printintln(pp_params.base.quick_stop_deceleration);
							printintln(pp_params.software_position_limit_max);
							printintln(pp_params.software_position_limit_min);
							printintln(pp_params.base.polarity);
							printintln(pp_params.max_acceleration);
							*/
							if(sensor_select == HALL)
							{
								set_position_ctrl_hall_param(hall_params, c_position_ctrl);
							}
							else if(sensor_select == QEI)
							{
								set_position_ctrl_qei_param(qei_params, c_position_ctrl);
							}

							set_position_ctrl_param(position_ctrl_params, c_position_ctrl);
							set_position_sensor(sensor_select, c_position_ctrl);

							init_position_profile_limits(qei_params.gear_ratio, pp_params.max_acceleration, pp_params.base.max_profile_velocity);
							InOut.operation_mode_display = PP;
						}
						break;

					case TQ:
						//printstrln("TQ");
						if(op_set_flag == 0)
						{
							ctrl_state = check_velocity_ctrl_state(c_velocity_ctrl);
							if(ctrl_state == 1)
								shutdown_velocity_ctrl(c_velocity_ctrl);
							ctrl_state = check_position_ctrl_state(c_position_ctrl);
							if(ctrl_state == 1)
								shutdown_position_ctrl(c_position_ctrl);
							init = init_torque_control(c_torque_ctrl);
						}
						if(init == INIT)
						{
							op_set_flag = 1;
							//enable_torque_ctrl(c_torque_ctrl);
							mode_selected = 1;
							op_mode = TQ;
							steps = 0;
							mode_quick_flag = 10;
							ack = 0;
							shutdown_ack = 0;

							update_torque_ctrl_param_ecat(torque_ctrl_params, coe_out);  //after checking init go to set display mode
							sensor_select = sensor_select_sdo(coe_out);
							update_cst_param_ecat(cst_params, coe_out);
							update_pt_param_ecat(pt_params, coe_out);
							torque_offstate = (cst_params.max_torque * 15) / (cst_params.nominal_current * 100 * cst_params.motor_torque_constant);
							//printintln(pt_params.profile_slope);
						/*	printintln(torque_ctrl_params.Control_limit);
							printintln(torque_ctrl_params.Integral_limit);
							printintln(torque_ctrl_params.Kd_d);
							printintln(torque_ctrl_params.Ki_d);
							printintln(torque_ctrl_params.Kp_d);
							printintln(torque_ctrl_params.Kp_n);
							printintln(torque_ctrl_params.Ki_n);
							printintln(torque_ctrl_params.Kd_n);
							printintln(sensor_select);
							printintln(cst_params.max_torque);
							printintln(cst_params.motor_torque_constant);
							printintln(cst_params.nominal_current);
							printintln(cst_params.nominal_motor_speed);
							printintln(cst_params.polarity);*/
							if(sensor_select == HALL)
							{
								set_torque_ctrl_hall_param(hall_params, c_torque_ctrl);
							}
							else if(sensor_select == QEI)
							{
								set_torque_ctrl_qei_param(qei_params, c_torque_ctrl);
							}

							set_torque_ctrl_param(torque_ctrl_params, c_torque_ctrl);
							set_torque_sensor(sensor_select, c_torque_ctrl);

							InOut.operation_mode_display = TQ;
						}
						break;

					case PV:
						//printstrln("pv");
						if(op_set_flag == 0)
						{
							ctrl_state = check_torque_ctrl_state(c_torque_ctrl);
							if(ctrl_state == 1)
								shutdown_torque_ctrl(c_torque_ctrl);
							ctrl_state = check_position_ctrl_state(c_position_ctrl);
							if(ctrl_state == 1)
								shutdown_position_ctrl(c_position_ctrl);
							init = init_velocity_control(c_velocity_ctrl);
						}
						if(init == INIT)
						{
							op_set_flag = 1;
							//enable_velocity_ctrl(c_velocity_ctrl);
							mode_selected = 1;
							op_mode = PV;
							steps = 0;
							mode_quick_flag = 10;
							ack = 0;
							shutdown_ack = 0;

							update_velocity_ctrl_param_ecat(velocity_ctrl_params, coe_out);  //after checking init go to set display mode
							sensor_select = sensor_select_sdo(coe_out);
							update_pv_param_ecat(pv_params, coe_out);
							/*printintln(velocity_ctrl_params.Kp_n);
							printintln(velocity_ctrl_params.Ki_n);
							printintln(velocity_ctrl_params.Kd_n);
							printintln(sensor_select);
							printintln(pv_params.max_profile_velocity);
							printintln(pv_params.profile_acceleration);
							printintln(pv_params.profile_deceleration);
							printintln(pv_params.quick_stop_deceleration);
							printintln(pv_params.polarity);
							 */
							if(sensor_select == HALL)
							{
								set_velocity_ctrl_hall_param(hall_params, c_velocity_ctrl);
							}
							else if(sensor_select == QEI)
							{
								set_velocity_ctrl_qei_param(qei_params, c_velocity_ctrl);
							}

							set_velocity_ctrl_param(velocity_ctrl_params, c_velocity_ctrl);
							set_velocity_sensor(sensor_select, c_velocity_ctrl);

							InOut.operation_mode_display = PV;
						}
						break;

					case CSP:
						if(op_set_flag == 0)
						{
							//ctrl_state = check_position_ctrl_state(c_position_ctrl);
							//if(ctrl_state == 0)
							//	shutdown_position_ctrl(c_position_ctrl);

							update_position_ctrl_param_ecat(position_ctrl_params, coe_out);
							sensor_select = sensor_select_sdo(coe_out);

							if(sensor_select == HALL)
							{
								set_position_ctrl_hall_param(hall_params, c_position_ctrl);
							}
							else if(sensor_select == QEI)
							{
								set_position_ctrl_qei_param(qei_params, c_position_ctrl);
							}
							set_position_ctrl_param(position_ctrl_params, c_position_ctrl);
							set_position_sensor(sensor_select, c_position_ctrl);

							ctrl_state = check_torque_ctrl_state(c_torque_ctrl);
							if(ctrl_state == 1)
								shutdown_torque_ctrl(c_torque_ctrl);
							ctrl_state = check_velocity_ctrl_state(c_velocity_ctrl);
							if(ctrl_state == 1)
								shutdown_velocity_ctrl(c_velocity_ctrl);
							init = init_position_control(c_position_ctrl);
						}
						if(init == INIT)
						{
							op_set_flag = 1;
							//enable_position_ctrl(c_position_ctrl);
							mode_selected = 1;
							mode_quick_flag = 10;
							op_mode = CSP;
							ack = 0;
							shutdown_ack = 0;

							update_csp_param_ecat(csp_params, coe_out);
							/*printintln(position_ctrl_params.Control_limit);
							printintln(position_ctrl_params.Integral_limit);
							printintln(position_ctrl_params.Kd_d);
							printintln(position_ctrl_params.Ki_d);
							printintln(position_ctrl_params.Kp_d);
							printintln(position_ctrl_params.Kp_n);
							printintln(position_ctrl_params.Ki_n);
							printintln(position_ctrl_params.Kd_n);
							printintln(sensor_select);
							printintln(csp_params.base.max_acceleration);
							printintln(csp_params.base.max_motor_speed);
							printintln(csp_params.base.motor_torque_constant);
							printintln(csp_params.base.nominal_current);
							printintln(csp_params.base.polarity);
							printintln(csp_params.max_position_limit);
							printintln(csp_params.min_position_limit);
							*/


							InOut.operation_mode_display = CSP;
						}
						break;

					case CSV: 	//csv mode index
						if(op_set_flag == 0)
						{
							ctrl_state = check_torque_ctrl_state(c_torque_ctrl);
							if(ctrl_state == 1)
								shutdown_torque_ctrl(c_torque_ctrl);
							ctrl_state = check_position_ctrl_state(c_position_ctrl);
							if(ctrl_state == 1)
								shutdown_position_ctrl(c_position_ctrl);
							init = init_velocity_control(c_velocity_ctrl);
						}
						if(init == INIT)
						{
							op_set_flag = 1;
							//enable_velocity_ctrl(c_velocity_ctrl);
							mode_selected = 1;
							mode_quick_flag = 10;
							op_mode = CSV;
							ack = 0;
							shutdown_ack = 0;
							update_velocity_ctrl_param_ecat(velocity_ctrl_params, coe_out);  //after checking init go to set display mode
							sensor_select = sensor_select_sdo(coe_out);
							update_csv_param_ecat(csv_params, coe_out);

							/*printintln(velocity_ctrl_params.Kp_n);
							printintln(velocity_ctrl_params.Ki_n);
							printintln(velocity_ctrl_params.Kd_n);
							printintln(csv_params.max_acceleration);
							printintln(csv_params.max_motor_speed);
							printintln(csv_params.motor_torque_constant);
							printintln(csv_params.nominal_current);
							printintln(csv_params.polarity);
							*/

							if(sensor_select == HALL)
							{
								set_velocity_ctrl_hall_param(hall_params, c_velocity_ctrl);
							}
							else if(sensor_select == QEI)
							{
								set_velocity_ctrl_qei_param(qei_params, c_velocity_ctrl);
							}

							set_velocity_ctrl_param(velocity_ctrl_params, c_velocity_ctrl);
							set_velocity_sensor(sensor_select, c_velocity_ctrl);

							InOut.operation_mode_display = CSV;
						}
						break;

					case CST:
						//printstrln("op mode enabled on slave");
						if(op_set_flag == 0)
						{
							ctrl_state = check_velocity_ctrl_state(c_velocity_ctrl);
							if(ctrl_state == 1)
								shutdown_velocity_ctrl(c_velocity_ctrl);
							ctrl_state = check_position_ctrl_state(c_position_ctrl);
							if(ctrl_state == 1)
								shutdown_position_ctrl(c_position_ctrl);
							init = init_torque_control(c_torque_ctrl);
						}
						if(init == INIT)
						{
							op_set_flag = 1;
							//enable_torque_ctrl(c_torque_ctrl);
							mode_selected = 1;
							mode_quick_flag = 10;
							op_mode = CST;
							ack = 0;
							shutdown_ack = 0;

							update_torque_ctrl_param_ecat(torque_ctrl_params, coe_out);  //after checking init go to set display mode

							sensor_select = sensor_select_sdo(coe_out);
							update_cst_param_ecat(cst_params, coe_out);
							update_pt_param_ecat(pt_params, coe_out);
							torque_offstate = (cst_params.max_torque * 15) / (cst_params.nominal_current * 100 * cst_params.motor_torque_constant);
							qei_params.poles = hall_params.pole_pairs;
							/*printintln(qei_params.gear_ratio);
							printintln(qei_params.index);
							printintln(qei_params.max_count);
							printintln(qei_params.real_counts);
							printintln(hall_params.gear_ratio);
							printintln(hall_params.pole_pairs);
							printintln(torque_ctrl_params.Control_limit);
							printintln(torque_ctrl_params.Integral_limit);
							printintln(torque_ctrl_params.Kd_d);
							printintln(torque_ctrl_params.Ki_d);
							printintln(torque_ctrl_params.Kp_d);
							printintln(torque_ctrl_params.Kp_n);
							printintln(torque_ctrl_params.Ki_n);
							printintln(torque_ctrl_params.Kd_n);
							printintln(sensor_select);
							printintln(cst_params.max_torque);
							printintln(cst_params.motor_torque_constant);
							printintln(cst_params.nominal_current);
							printintln(cst_params.nominal_motor_speed);
							printintln(cst_params.polarity);*/

							if(sensor_select == HALL)
							{
								set_torque_ctrl_hall_param(hall_params, c_torque_ctrl);
							}
							else if(sensor_select == QEI)
							{
								set_torque_ctrl_qei_param(qei_params, c_torque_ctrl);
							}


							set_torque_ctrl_param(torque_ctrl_params, c_torque_ctrl);
							set_torque_sensor(sensor_select, c_torque_ctrl);


							InOut.operation_mode_display = CST;
						}
						break;

				}
			}
		//	printstr("mode ");
		//	printhexln(mode_selected);
			/*printhexln(InOut.control_word);
			printstr("mode ");
			printhexln(mode_selected);
			printstr("shtudown ");
			printhexln(shutdown_ack);
			printstr("qactive ");
			printhexln(quick_active);*/
			//printhexln(statusword);
			if(mode_selected == 1)
			{
				switch(InOut.control_word)
				{
					case 0x000b: //quick stop
						if(op_mode == CST || op_mode == TQ)
						{
							actual_torque = get_torque(cst_params, c_torque_ctrl);
							steps = init_linear_profile(0, actual_torque, pt_params.profile_slope, pt_params.profile_slope, cst_params.max_torque);
							i = 0;
							mode_selected = 3;// non interruptible mode
							mode_quick_flag = 0;
						}
						else if(op_mode == CSV || op_mode == PV)
						{
							actual_velocity = get_velocity(c_velocity_ctrl);
							if(op_mode == CSV)
								steps = init_quick_stop_velocity_profile(actual_velocity, csv_params.max_acceleration);
							else if(op_mode == PV)
								steps = init_quick_stop_velocity_profile(actual_velocity, pv_params.quick_stop_deceleration);
							i = 0;
							mode_selected = 3;// non interruptible mode
							mode_quick_flag = 0;
						}
						else if(op_mode == CSP || op_mode == PP)
						{
							actual_velocity = get_hall_velocity(c_hall, hall_params);
							actual_position = get_position(c_position_ctrl);

							if(!(actual_velocity<40 && actual_velocity>-40))
							{
								if(actual_velocity < 0)
								{
									actual_velocity = 0-actual_velocity;
									sense = -1;
								}
								if(op_mode == CSP)
									steps = init_quick_stop_position_profile( (actual_velocity*360)/(60*hall_params.gear_ratio), actual_position, csp_params.base.max_acceleration);
								else if(op_mode == PP)
									steps = init_quick_stop_position_profile( (actual_velocity*360)/(60*hall_params.gear_ratio), actual_position, pp_params.base.quick_stop_deceleration);
								i = 0;
								mode_selected = 3;// non interruptible mode
								mode_quick_flag = 0;
							}
							else
							{
								mode_selected = 100;
								op_set_flag = 0; init = 0;
								mode_quick_flag = 0;
							}
						}
						break;

					case 0x000f: //switch on cyclic
						//printstrln("cyclic");
						if(op_mode == HM)
						{
							if(ack == 0 )
							{
								home_velocity = get_target_velocity(InOut);
								home_acceleration = get_target_torque(InOut);
								//h_active = 1;
								//if(home_velocity >0 || home_velocity < 0)

								if(home_velocity == 0)
								{
									home_velocity = get_target_velocity(InOut);
								}
								if(home_acceleration == 0)
								{
									home_acceleration = get_target_torque(InOut);
								}
								if(home_velocity > 0 || home_velocity < 0)
								{
									if( home_acceleration > 0 || home_acceleration < 0)
									{
										//mode_selected = 4;
										set_home_switch_type(c_home, limit_switch_type);
										i = 1;
										actual_velocity = get_velocity(c_velocity_ctrl);
										steps = init_velocity_profile(home_velocity * limit_switch, actual_velocity, home_acceleration,\
												home_acceleration, home_velocity);
										//printintln(home_velocity);
										//printintln(home_acceleration);
										ack = 1;
										reset_counter = 0;
										end_state = 0;
									}
								}
							}
							else if(ack == 1)
							{
								if(reset_counter == 0)
								{
									ack =1;//h_active = 1;
									if(i < steps)
									{
										velocity_ramp = velocity_profile_generate(i);
										set_velocity(velocity_ramp, c_velocity_ctrl);
										i = i+1;
									}

									{home_state, safety_state, capture_position, direction} = get_home_state(c_home);
//									if(i>=steps && end_state ==0)
//									{
//										home_state =1;//simulate switch
//										{capture_position, direction} = get_qei_position_absolute(c_qei);
//									}
									if( (home_state == 1 || safety_state == 1 ) && end_state == 0)
									{
										actual_velocity = get_velocity(c_velocity_ctrl);
										steps = init_velocity_profile(0, actual_velocity, home_acceleration, home_acceleration,\
												home_velocity);
										i = 1;
										end_state = 1;
									}
									if(end_state == 1 && i >= steps)
									{
										shutdown_velocity_ctrl(c_velocity_ctrl);
										if(home_state == 1)
										{
											{current_position, direction} = get_qei_position_absolute(c_qei);
											//{current_position, direction} = get_hall_position_absolute(c_hall);
											//printintln(current_position);
											home_offset = current_position - capture_position;
											printintln(home_offset);
											reset_qei_count(c_qei, home_offset); //reset_hall_count(c_hall, home_offset);//
											reset_counter = 1;
										}
									}
									if(sensor_select == HALL)
									{
										{actual_position, direction} = get_hall_position_absolute(c_hall);
										actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
									}
									else if(sensor_select == QEI)
									{
										{actual_position, direction} = get_qei_position_absolute(c_qei);
										actual_position = (actual_position * precision_factor)/precision;
									}
									send_actual_position(actual_position * polarity, InOut);
								}
								if(reset_counter == 1)
								{
									ack = 0;//h_active = 1;
									if(sensor_select == HALL)
									{
										{actual_position, direction} = get_hall_position_absolute(c_hall);
										actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
									}
									else if(sensor_select == QEI)
									{
										{actual_position, direction} = get_qei_position_absolute(c_qei);
										actual_position = (actual_position * precision_factor)/precision;
									}
									send_actual_position(actual_position * polarity, InOut);
									//mode_selected = 100;
									homing_done = 1;
									printstrln("homing_success"); //done
									InOut.operation_mode_display = 250;

									//op_set_flag = 0;
									//init = 0;
									//mode_selected = 0;  // to reenable the op selection and reset the controller
									//setup_loop_flag = 0;
								}
							}


						}
						if(op_mode == CSV)
						{
							target_velocity = get_target_velocity(InOut);
							set_velocity_csv(csv_params, target_velocity, 0, 0, c_velocity_ctrl);

							actual_velocity = get_velocity(c_velocity_ctrl) *  csv_params.polarity;
							send_actual_velocity(actual_velocity, InOut);
							if(sensor_select == HALL)
							{
								{actual_position, direction} = get_hall_position_absolute(c_hall);
								actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
							}
							else if(sensor_select == QEI)
							{
								{actual_position, direction} = get_qei_position_absolute(c_qei);
								actual_position = (actual_position * precision_factor)/precision;
							}
							send_actual_position(actual_position * csv_params.polarity, InOut);
						#ifdef ENABLE_xscope_main
						//	xscope_probe_data(0, actual_velocity);
						//	xscope_probe_data(1, target_velocity);
						#endif
						}
						else if(op_mode == CST)
						{
							//printstrln("CST");
							target_torque = get_target_torque(InOut);
							set_torque_cst(cst_params, target_torque, 0, c_torque_ctrl);
						//	xscope_probe_data(0, target_torque);
							//printintln(target_torque);

							actual_torque = get_torque(cst_params, c_torque_ctrl) *  cst_params.polarity;
						//	xscope_probe_data(1, actual_torque);
							send_actual_torque(actual_torque, InOut);
							if(sensor_select == HALL)
							{
								{actual_position, direction} = get_hall_position_absolute(c_hall);
								actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
							}
							else if(sensor_select == QEI)
							{
								{actual_position, direction} = get_qei_position_absolute(c_qei);
								actual_position = (actual_position * precision_factor)/precision;
							}
							send_actual_position(actual_position * cst_params.polarity, InOut);

						}
						else if(op_mode == CSP)
						{
							target_position = get_target_position(InOut);
							set_position_csp(csp_params, target_position, 0, 0, 0, c_position_ctrl);


							actual_position = get_position(c_position_ctrl) * csp_params.base.polarity;
							send_actual_position(actual_position, InOut);
					//#ifdef ENABLE_xscope_main
						//	xscope_probe_data(0, actual_position);
						//	xscope_probe_data(1, target_position);
					//#endif
						}
						else if(op_mode == PP)
						{
							if(ack == 1)
							{
								target_position = get_target_position(InOut);
								//printintln(target_position);
								actual_position = get_position(c_position_ctrl)*pp_params.base.polarity;
								send_actual_position(actual_position, InOut);

								if(prev_position != target_position)
								{
									ack = 0;
									steps = init_position_profile(target_position, actual_position, \
											pp_params.profile_velocity, pp_params.base.profile_acceleration,\
											pp_params.base.profile_deceleration);

									i = 1;
									prev_position = target_position;
								}
							}
							else if(ack == 0)
							{
								if(i < steps)
								{
									position_ramp = position_profile_generate(i);
									set_position( position_limit( position_ramp * pp_params.base.polarity ,	\
											pp_params.software_position_limit_max * 10000  , 			\
											pp_params.software_position_limit_min * 10000) , c_position_ctrl);
									i++;
								}
								else if(i >= steps)
								{
									t when timerafter(time + 100*MSEC_STD) :> time;
									ack = 1;
								}
								actual_position = get_position(c_position_ctrl) *pp_params.base.polarity;
								send_actual_position(actual_position, InOut);
							}
						}
						else if(op_mode == TQ)
						{
							//printstrln("cyclic TQ ");
							if(ack == 1)
							{
								target_torque = get_target_torque(InOut);
								//printintln(target_torque);
								actual_torque = get_torque(cst_params, c_torque_ctrl) *  pt_params.polarity;
								send_actual_torque(actual_torque, InOut);

								if(prev_torque != target_torque)
								{
									ack = 0;
									steps = init_linear_profile(target_torque, actual_torque, \
											pt_params.profile_slope, pt_params.profile_slope, cst_params.max_torque);

									i = 1;
									prev_torque = target_torque;
								}
							}
							else if(ack == 0)
							{
								if(i < steps)
								{
									torque_ramp = linear_profile_generate(i);
									set_torque_cst(cst_params, torque_ramp, 0, c_torque_ctrl);
									i++;
								}
								else if(i >= steps)
								{
									t when timerafter(time + 100*MSEC_STD) :> time;
									ack = 1;
								}
								actual_torque = get_torque(cst_params, c_torque_ctrl) *  pt_params.polarity;
								send_actual_torque(actual_torque, InOut);
								if(sensor_select == HALL)
								{
									{actual_position, direction} = get_hall_position_absolute(c_hall);
									actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
								}
								else if(sensor_select == QEI)
								{
									{actual_position, direction} = get_qei_position_absolute(c_qei);
									actual_position = (actual_position * precision_factor)/precision;
								}
								send_actual_position(actual_position * pt_params.polarity, InOut);
							}
						}
						else if(op_mode == PV)
						{
							//printstr("PV ");
							if(ack == 1)
							{
								target_velocity = get_target_velocity(InOut);
								//printintln(target_velocity);
								actual_velocity = get_velocity(c_velocity_ctrl) *  pv_params.polarity;
								send_actual_velocity(actual_velocity, InOut);

								if(prev_velocity != target_velocity)
								{
									ack = 0;
									steps = init_velocity_profile(target_velocity, actual_velocity, \
											pv_params.profile_acceleration, pv_params.profile_deceleration,\
											pv_params.max_profile_velocity);


									i = 1;
									prev_velocity = target_velocity;
								}
							}
							else if(ack == 0)
							{
								if(i < steps)
								{
									velocity_ramp = velocity_profile_generate(i);
									//printintln(velocity_ramp);
									set_velocity( max_speed_limit(	(velocity_ramp) * pv_params.polarity,\
											pv_params.max_profile_velocity  ), c_velocity_ctrl );
									i++;
								}
								else if(i >= steps)
								{
									t when timerafter(time + 100*MSEC_STD) :> time;
									ack = 1;
								}
								actual_velocity = get_velocity(c_velocity_ctrl) *  pv_params.polarity;
								send_actual_velocity(actual_velocity, InOut);
								if(sensor_select == HALL)
								{
									{actual_position, direction} = get_hall_position_absolute(c_hall);
									actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
								}
								else if(sensor_select == QEI)
								{
									{actual_position, direction} = get_qei_position_absolute(c_qei);
									actual_position = (actual_position * precision_factor)/precision;
								}
								send_actual_position(actual_position * pv_params.polarity, InOut);
							}
						}
						break;

					case 0x0006: //shutdown
						//deactivate
						if(op_mode == CST || op_mode == TQ)
						{
							shutdown_torque_ctrl(c_torque_ctrl);
							shutdown_ack = 1;
							op_set_flag = 0;
							init = 0;
							mode_selected = 0;  // to reenable the op selection and reset the controller
							setup_loop_flag = 0;
						}
						else if(op_mode == CSV || op_mode == PV)
						{
							shutdown_velocity_ctrl(c_velocity_ctrl);
							shutdown_ack = 1;
							op_set_flag = 0;
							init = 0;
							mode_selected = 0;  // to reenable the op selection and reset the controller
							setup_loop_flag = 0;
						}
						else if(op_mode == CSP || op_mode == PP)
						{
							shutdown_position_ctrl(c_position_ctrl);
							shutdown_ack = 1;
							op_set_flag = 0;
							init = 0;
							mode_selected = 0;  // to reenable the op selection and reset the controller
							setup_loop_flag = 0;
						}
						else if(op_mode == HM)
						{
							//shutdown_position_ctrl(c_position_ctrl);
							shutdown_ack = 1;
							op_set_flag = 0;
							init = 0;
							mode_selected = 0;  // to reenable the op selection and reset the controller
							setup_loop_flag = 0;
						}
						break;

				}
			}

			/*printstr("mode ");
			printhexln(mode_selected);
			printstr("mode q flag ");
			printhexln(mode_quick_flag);
			printstr(" i ");
			printhexln(i);
			printstr(" steps ");
			printhexln(steps);*/
			else if(mode_selected == 4)
			{
				//mode_selected = 1000; // changing the mode to stop after homing
				//reset_counter = 0;
				//printstrln("home mode");
				//printintln(home_velocity);
				if(reset_counter == 0)
				{
					ack =1;//h_active = 1;
					//i = 1;
//					home_state = 0;
//					safety_state = 0;
//					end_state = 0;


					if(i < steps)
					{
						velocity_ramp = velocity_profile_generate(i);
						set_velocity(velocity_ramp, c_velocity_ctrl);
						i = i+1;
					}

					{home_state, safety_state, capture_position, direction} = get_home_state(c_home);
					if( (home_state == 1 || safety_state == 1 ) && end_state == 0)
					{
						actual_velocity = get_velocity(c_velocity_ctrl);
						steps = init_velocity_profile(0, actual_velocity, home_acceleration, home_acceleration,\
								home_velocity);
						i = 1;
					/*	for(i = 1; i < steps; i++)
						{
							velocity_ramp = velocity_profile_generate(i);
							set_velocity(velocity_ramp, c_velocity_ctrl);
							actual_velocity = get_velocity(c_velocity_ctrl);

							t when timerafter(h_time + MSEC_STD) :> h_time;

						}*/
						end_state = 1;
						//shutdown_velocity_ctrl(c_velocity_ctrl);
						//printintln(capture_position);
					}
					//statusword = update_statusword(statusword, state, ack, quick_active, shutdown_ack);
					//	InOut.status_word = statusword;
					//	ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);


					if(end_state == 1 && i >= steps)
					{
						shutdown_velocity_ctrl(c_velocity_ctrl);
						if(home_state == 1)
						{
							{current_position, direction} = get_qei_position_absolute(c_qei);
							//printintln(current_position);
							home_offset = current_position - capture_position;
							printintln(home_offset);
							reset_qei_count(c_qei, home_offset);
							reset_counter = 1;
						}
					}
				}

				if(reset_counter == 1)
				{
					ack = 0;//h_active = 1;
					//mode_selected = 100;
					printstrln("homing_success"); //done
					InOut.operation_mode_display = 250;
					shutdown_ack = 1;
					op_set_flag = 0;
					init = 0;
					//mode_selected = 0;  // to reenable the op selection and reset the controller
					setup_loop_flag = 0;
				}
			}

			else if(mode_selected == 3) // non interrupt
			{
				if(op_mode == CST || op_mode == TQ)
				{
					while(i < steps)
					{
						target_torque = linear_profile_generate(i);
						set_torque(target_torque, cst_params, c_torque_ctrl);
						actual_torque = get_torque(cst_params, c_torque_ctrl)*cst_params.polarity;
						send_actual_torque(actual_torque, InOut);
					//	xscope_probe_data(0, target_torque);
					//	xscope_probe_data(1, actual_torque);
						t when timerafter(time + MSEC_STD) :> time;
						i++;
					}
					if(i == steps )
					{
						t when timerafter(time + 100*MSEC_STD) :> time;
						actual_torque = get_torque(cst_params, c_torque_ctrl);
						send_actual_torque(actual_torque, InOut);
					}
					if(i >= steps)
					{
						actual_torque = get_torque(cst_params, c_torque_ctrl);
						send_actual_torque(actual_torque, InOut);
						if(actual_torque < torque_offstate || actual_torque > -torque_offstate)
						{
							ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);
							mode_selected = 100;
							op_set_flag = 0; init = 0;
						}
					}
					if(steps == 0)
					{
						mode_selected = 100;
						op_set_flag = 0; init = 0;
					}
				}
				else if(op_mode == CSV || op_mode == PV)
				{

					while(i < steps)
					{
						target_velocity = quick_stop_velocity_profile_generate(i);
						if(op_mode == CSV)
						{
							set_velocity( max_speed_limit(target_velocity, csv_params.max_motor_speed), c_velocity_ctrl );
							actual_velocity = get_velocity(c_velocity_ctrl);
							send_actual_velocity(actual_velocity * csv_params.polarity, InOut);
							if(sensor_select == HALL)
							{
								{actual_position, direction} = get_hall_position_absolute(c_hall);
								actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
							}
							else if(sensor_select == QEI)
							{
								{actual_position, direction} = get_qei_position_absolute(c_qei);
								actual_position = (actual_position * precision_factor)/precision;
							}
							send_actual_position(actual_position * csv_params.polarity, InOut);
						}
						else if(op_mode == PV)
						{
							set_velocity( max_speed_limit(target_velocity, pv_params.max_profile_velocity), c_velocity_ctrl );
							actual_velocity = get_velocity(c_velocity_ctrl);
							send_actual_velocity(actual_velocity * pv_params.polarity, InOut);
						}
					#ifdef ENABLE_xscope_main
						//xscope_probe_data(0, actual_velocity);
						//xscope_probe_data(1, target_velocity);
					#endif

						t when timerafter(time + MSEC_STD) :> time;
						i++;
					}
					if(i == steps )
					{
						t when timerafter(time + 100*MSEC_STD) :> time;
					}
					if(i >= steps)
					{
						if(op_mode == CSV)
							send_actual_velocity(actual_velocity*csv_params.polarity, InOut);
						else if(op_mode == PV)
							send_actual_velocity(actual_velocity*pv_params.polarity, InOut);
						if(actual_velocity < 50 || actual_velocity > -50)
						{
							ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);
							mode_selected = 100;
							op_set_flag = 0; init = 0;
						}
					}
					if(steps == 0)
					{
						mode_selected = 100;
						op_set_flag = 0; init = 0;

					}

				}
				else if(op_mode == CSP || op_mode == PP)
				{
					if(sensor_select == HALL)
					{
						{actual_position, sense} = get_hall_position_absolute(c_hall);
					}
					else if(sensor_select == QEI)
					{
						{actual_position, sense} = get_qei_position_absolute(c_qei);
					}
					while(i < steps)
					{
						target_position   =   quick_stop_position_profile_generate(i, sense);
						if(op_mode == CSP)
						{
							set_position( position_limit( target_position ,				\
									csp_params.max_position_limit * 10000  , 			\
									csp_params.min_position_limit * 10000) , c_position_ctrl);
							actual_position = get_position(c_position_ctrl);
							send_actual_position(actual_position * csp_params.base.polarity, InOut);
						}
						else if(op_mode == PP)
						{
							set_position( position_limit( target_position ,						\
									pp_params.software_position_limit_max * 10000  , 			\
									pp_params.software_position_limit_min * 10000) , c_position_ctrl);
							actual_position = get_position(c_position_ctrl);
							send_actual_position(actual_position * pp_params.base.polarity, InOut);
						}
					#ifdef ENABLE_xscope_main
						xscope_probe_data(0, actual_position);
						xscope_probe_data(1, target_position);
					#endif
						t when timerafter(time + MSEC_STD) :> time;
						i++;
					}
					if(i == steps )
					{
						t when timerafter(time + 100*MSEC_STD) :> time;
					}
					if(i >=steps )
					{
						actual_velocity = get_hall_velocity(c_hall, hall_params);
						actual_position = get_position(c_position_ctrl);
						if(op_mode == CSP)
							send_actual_position(actual_position * csp_params.base.polarity, InOut);
						else if(op_mode == PP)
							send_actual_position(actual_position*pp_params.base.polarity, InOut);
						if(actual_velocity < 50 || actual_velocity > -50)
						{
							mode_selected = 100;
							op_set_flag = 0; init = 0;
						}
					}
				//#ifdef ENABLE_xscope_main
					//xscope_probe_data(0, actual_position);
					//xscope_probe_data(1, target_position);
				//#endif
				}


			}
			else if(mode_selected ==100)
			{
				if(mode_quick_flag == 0)
					quick_active = 1;

				if(op_mode == CST)
				{
					actual_torque = get_torque(cst_params, c_torque_ctrl)*cst_params.polarity;
					send_actual_torque(actual_torque, InOut);
					if(sensor_select == HALL)
					{
						{actual_position, direction} = get_hall_position_absolute(c_hall);
						actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
					}
					else if(sensor_select == QEI)
					{
						{actual_position, direction} = get_qei_position_absolute(c_qei);
						actual_position = (actual_position * precision_factor)/precision;
					}
					send_actual_position(actual_position * cst_params.polarity, InOut);
				//	xscope_probe_data(0, target_torque);
				//	xscope_probe_data(1, actual_torque);
				}
				else if(op_mode == TQ)
				{
					actual_torque = get_torque(cst_params, c_torque_ctrl)*cst_params.polarity;
					send_actual_torque(actual_torque, InOut);
					if(sensor_select == HALL)
					{
						{actual_position, direction} = get_hall_position_absolute(c_hall);
						actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
					}
					else if(sensor_select == QEI)
					{
						{actual_position, direction} = get_qei_position_absolute(c_qei);
						actual_position = (actual_position * precision_factor)/precision;
					}
					send_actual_position(actual_position * pt_params.polarity, InOut);
				}
				else if(op_mode == CSP)
				{
					actual_position = get_position(c_position_ctrl);
					send_actual_position(actual_position * csp_params.base.polarity, InOut);
				}
				else if(op_mode == PP)
				{
					actual_position = get_position(c_position_ctrl);
					send_actual_position(actual_position * pp_params.base.polarity, InOut);
				}
				else if(op_mode == CSV)
				{
					actual_velocity = get_velocity(c_velocity_ctrl);
					send_actual_velocity(actual_velocity*csv_params.polarity, InOut);
					if(sensor_select == HALL)
					{
						{actual_position, direction} = get_hall_position_absolute(c_hall);
						actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
					}
					else if(sensor_select == QEI)
					{
						{actual_position, direction} = get_qei_position_absolute(c_qei);
						actual_position = (actual_position * precision_factor)/precision;
					}
					send_actual_position(actual_position * csv_params.polarity, InOut);
				}
				else if(op_mode == PV)
				{
					actual_velocity = get_velocity(c_velocity_ctrl);
					send_actual_velocity(actual_velocity*pv_params.polarity, InOut);
					send_actual_velocity(actual_velocity, InOut);
					if(sensor_select == HALL)
					{
						{actual_position, direction} = get_hall_position_absolute(c_hall);
						actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
					}
					else if(sensor_select == QEI)
					{
						{actual_position, direction} = get_qei_position_absolute(c_qei);
						actual_position = (actual_position * precision_factor)/precision;
					}
					send_actual_position(actual_position * pv_params.polarity, InOut);
				}
				switch(InOut.operation_mode)
				{
					case 100:
						mode_selected = 0;
						quick_active = 0;
						mode_quick_flag = 1;
						InOut.operation_mode_display = 100;

						break;
				}
			//  xscope_probe_data(0, actual_position);
			//  xscope_probe_data(1, target_position);
			}
			t when timerafter(time + MSEC_STD) :> time;
		}

	}
}
