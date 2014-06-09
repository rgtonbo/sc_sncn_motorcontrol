/**
 * \file ecat_motor_drive.h
 * \brief Ethercat Motor Drive Server
 * \author Pavan Kanajar <pkanajar@synapticon.com>
*/

#pragma once

#include <comm.h>
#include <drive_config.h>
#include <velocity_ctrl_client.h>
#include <position_ctrl_client.h>
#include <torque_ctrl_client.h>
#include <hall_client.h>
#include <qei_client.h>
#include <profile.h>

/**
 * \brief This server implementation enables motor drive functions via Ethercat communication
 *
 *  Input Channel
 * \channel pdo_in channel to receive information from ethercat
 * \channel coe_out channel to receive motor config information from ethercat
 * \channel c_signal channel to receive init ack from commutation loop
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 *
 *  Output Channel
 * \channel pdo_out channel to send out information via ethercat
 * \channel c_torque_ctrl channel to receive/send torque control information
 * \channel c_velocity_ctrl channel to receive/send velocity control information
 * \channel c_position_ctrl channel to receive/send position control information
 * \channel c_gpio channel to config/read/drive GPIO digital ports
 *
 */
void ecat_motor_drive(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall,\
        chanend c_qei, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl, chanend c_gpio);

int detect_sensor_placement(chanend c_hall, chanend c_qei, chanend c_commutation);

