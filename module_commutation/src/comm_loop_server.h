/**
 * \file comm_loop_server.h
 * \brief Commutation Loop based on sinusoidal commutation method
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 */

#pragma once

#include "comm_loop_client.h"
#include <watchdog.h>
#include <internal_config.h>

/**
 * \brief Sinusoidal based Commutation Loop
 *
 *  Input
 * \channel c_hall channel for receiving position information from hall sensor
 * \channel c_qei channel for receiving position information from the quadrature encoder interface (QEI)
 * \channel c_signal channel for signaling after initialization of commutation loop
 * \channel c_watchdog channel for controlling the watchdog
 * \channel c_commutation_p1 channel for receiving motor voltage input value - priority 1 (highest) 1 ... (lowest) 3
 * \channel c_commutation_p2 channel for receiving motor voltage input value - priority 2
 * \channel c_commutation_p3 channel for receiving motor voltage input value - priority 3
 * \param hall_params struct defines the pole-pairs and gear ratio
 * \param qei_params the struct defines sensor type and resolution parameters for QEI
 * \param commutation_params struct defines the commutation angle parameters
 *
 *  Output
 * \channel c_pwm_ctrl channel to set PWM level output to motor phases
 * \port p_ifm_esf_rstn_pwml_pwmh port to configure motor FET driver
 * \port p_ifm_coastn port to enable motor FET driver
 *
 */
void commutation_sinusoidal(chanend c_hall, chanend c_qei, chanend c_signal, chanend c_watchdog,
                            chanend c_commutation_p1, chanend c_commutation_p2, chanend c_commutation_p3, chanend c_pwm_ctrl,
                            out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn, port p_ifm_ff1, port p_ifm_ff2,
                            hall_par & hall_params, qei_par & qei_params, commutation_par & commutation_params);

/**
 * \brief Initialize commutation parameters
 *
 * \param commutation_params struct defines the commutation angle parameters
 * \param hall_params struct defines the pole-pairs and gear ratio
 * \param nominal speed is the rated speed for the motor given on specs sheet
 */
void init_commutation_param(commutation_par & commutation_params, hall_par & hall_params, int nominal_speed);
