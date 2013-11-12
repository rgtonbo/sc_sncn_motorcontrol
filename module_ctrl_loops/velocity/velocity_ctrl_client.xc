#include "velocity_ctrl_client.h"
#include "refclk.h"
#include <internal_config.h>
#include <drive_config.h>
#include "print.h"

//#define Debug_velocity_ctrl
//#define debug_print

int init_velocity_control(chanend c_velocity_ctrl)
{
	int init_state = INIT_BUSY;

	VELOCITY_CTRL_ENABLE(); 	//activate velocity ctrl loop

	 // init check from velocity control loop
	 while(1)
	 {

		init_state = __check_velocity_init(c_velocity_ctrl);
		if(init_state == INIT)
		{
#ifdef debug_print
			printstrln("vel intialized");
#endif
			break;
		}
	 }
	 return init_state;
}

//internal
void set_velocity(int target_velocity, chanend c_velocity_ctrl) {
	VELOCITY_CTRL_WRITE(SET_VELOCITY_TOKEN);
	VELOCITY_CTRL_WRITE(target_velocity);
}

int get_velocity(chanend c_velocity_ctrl) {
	int velocity;
	VELOCITY_CTRL_WRITE(GET_VELOCITY_TOKEN);
	VELOCITY_CTRL_READ(velocity);
	return velocity;
}

int max_speed_limit(int velocity, int max_speed) {
	if (velocity > max_speed) {
		velocity = max_speed;
		return velocity;
	} else if (velocity < -max_speed) {
		velocity = -max_speed;
		return velocity;
	} else if (velocity >= -max_speed && velocity <= max_speed) {
		return velocity;
	}
}

//csv mode function
void set_velocity_csv(csv_par &csv_params, int target_velocity,
		int velocity_offset, int torque_offset, chanend c_velocity_ctrl)
{
	set_velocity( max_speed_limit(	(target_velocity + velocity_offset) * csv_params.polarity, csv_params.max_motor_speed  ), c_velocity_ctrl );
}

void set_velocity_ctrl_param(ctrl_par &velocity_ctrl_params, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SET_CTRL_PARAMETER);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kp_n);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kp_d);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Ki_n);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Ki_d);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kd_n);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kd_d);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Integral_limit);
}

void set_velocity_ctrl_hall_param(hall_par &hall_params, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SET_VELOCITY_CTRL_HALL);
	VELOCITY_CTRL_WRITE(hall_params.gear_ratio);
	VELOCITY_CTRL_WRITE(hall_params.pole_pairs);
}

void set_velocity_ctrl_qei_param(qei_par &qei_params, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SET_VELOCITY_CTRL_QEI);
	VELOCITY_CTRL_WRITE(qei_params.gear_ratio);
	VELOCITY_CTRL_WRITE(qei_params.index);
	VELOCITY_CTRL_WRITE(qei_params.real_counts);
	VELOCITY_CTRL_WRITE(qei_params.max_count);
}

void set_velocity_sensor(int sensor_used, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SENSOR_SELECT);
	VELOCITY_CTRL_WRITE(sensor_used);
}

void set_velocity_filter(chanend c_velocity_ctrl, filter_par &filter_params)
{
	VELOCITY_CTRL_WRITE(SET_VELOCITY_FILTER);
	VELOCITY_CTRL_WRITE(filter_params.filter_length);
}

void shutdown_velocity_ctrl(chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SHUTDOWN_VELOCITY);
	VELOCITY_CTRL_WRITE(1);
}

void enable_velocity_ctrl(chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(ENABLE_VELOCITY);
	VELOCITY_CTRL_WRITE(0);
}