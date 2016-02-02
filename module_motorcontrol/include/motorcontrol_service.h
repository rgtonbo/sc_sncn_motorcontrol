/**
 * @file motorcontrol_service.h
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define ERROR 0
#define SUCCESS 1

#define MOTOR_CTLR_INTRFCE_CNT  4

/**
 * @brief Type for the kind of winding of a BLDC motor.
 */
typedef enum {
    STAR_WINDING=1, /**< Star winding. */
    DELTA_WINDING   /**< Delta winding. */
} BLDCWindingType;

/**
 * @brief Type for motors.
 */
typedef enum {
    BDC_MOTOR = 10,  /**< Brushed DC Motor. */
    BLDC_MOTOR = 11  /**< Brushless DC Motor. */
} MotorType;

/**
 * Structure type for Motorcontrol Service configuration.
 */
typedef struct {
    MotorType motor_type;               /**< Type of motor to drive. */
    BLDCWindingType bldc_winding_type;  /**< Type of winding of your motor (if using a BLDC motor). */
    int commutation_sensor;             /**< Absolute position sensor used for commutation (if using a BLDC motor). For the moment just Hall sensor can be used [HALL_SENSOR]. */
    int hall_offset[2];                 /**< Feedback Hall sensor error offset for positive (hall_offset[0]) and negative (hall_offset[1]) turning [0:4095]. (Often required to optimize commutation if using a BLDC motor). */
    int commutation_loop_period;        /**< Period for the commutation loop [microseconds]. */
} MotorcontrolConfig;

#ifdef __XC__

#include <watchdog_service.h>
#include <hall_service.h>
#include <qei_service.h>
#include <biss_service.h>
#include <ams_service.h>

#include <mc_internal_constants.h>

/**
 * @brief Structure type to define the ports to manage the fet-driver in your IFM SOMANET device (if applicable).
 */
typedef struct {
    port ?p_coast;  /**< [Nullable] Port for management signals. */
    out port ?p_esf_rst_pwml_pwmh; /**< [Nullable] 4-bit Port to  enabling operation signals (if applicable in your SOMANET device). */
    port ?p_ff1; /**< [Nullable] Port to read out faults (if applicable in your SOMANET device). */
    port ?p_ff2; /**< [Nullable] Port to read out faults (if applicable in your SOMANET device). */
} FetDriverPorts;

/**
 * @brief Interface type to communicate with the Motor Control Service.
 */
interface MotorcontrolInterface{

    /**
     * @brief Notifies the interested parties that a new notification
     * is available.
     */
    [[notification]]
    slave void notification();

    /**
     * @brief Provides the type of notification currently available.
     *
     * @return type of the notification
     */
    [[clears_notification]]
    int get_notification();

    /**
     * @brief Sets an amplitude voltage on the sinusodial signals commutating the windings.
     *
     * @param voltage Voltage [-PWM_MAX_VALUE:PWM_MAX_VALUE]. By default PWM_MAX_VALUE = 13889.
     */
    void set_voltage(int voltage);

    /**
     * @brief Setter for the configuration used by the Service.
     *        Note that not all configuration parameters can be changed on runtime.
     *
     * @param in_config New Service configuration.
     */
    void set_config(MotorcontrolConfig new_config);

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current configuration.
     */
    MotorcontrolConfig get_config();

    /**
     * @brief Setter for the status of the fets
     *
     * @return 0 - Fets disabled.
     *         1 - Fets enabled.
     */
    void set_fets_state(int state);

    /**
     * @brief Getter for the status of the fets
     *
     * @return 0 - Fets disabled.
     *         1 - Fets enabled.
     */
    int get_fets_state();

    /**
     * @brief Setter for Service new configuration. Also sets new configuration for the Hall Service and Encoder Service if possible.
     *
     * @param hall_config New configuration for Hall Service.
     * @param qei_config New configuration for Encoder Service.
     * @param motorcontrol_config New configuration for Motorcontrol service.
     */
    void set_all_parameters(HallConfig hall_config, QEIConfig qei_config, MotorcontrolConfig motorcontrol_config);

    /**
     * @brief Allows you to change the commutation sensor on runtime.
     *
     * @param sensor New sensor [HALL_SENSOR]. (So far, just Hall sensor is available for commutation)
     */
    void set_sensor(int sensor);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 - not initialized.
     *         1 - initialized.
     */
    int check_busy();
};

/**
 * @brief Service to drive BLDC and Brushed DC Motors.
 *        You will need additionally to have a PWM and Watchdog Services running.
 *
 *        If you are driving BLDC motors, also a parallel Service to read the
 *        absolute position of your rotor is required, for the moment
 *        just Hall Service is suitable for commutation purposes.
 *
 * @param fet_driver_ports Ports structure defining where to access the fet-driver signals.
 * @param motorcontrol_config Configuration for the Service.
 * @param c_pwm_ctrl Communication channel to the PWM Service.
 * @param i_hall [[Nullable]] Communication interface to the Hall Sensor Service (if applicable).
 * @param i_qei [[Nullable]] Communication interface to the Encoder Service (if applicable).
 * @param i_watchdog Communication interface to the Watchdog Service.
 * @param i_motorcontrol[5] Array of communication interfaces to handle up to 5 different clients.
 */
[[combinable]]
 void motorcontrol_service(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                             chanend c_pwm_ctrl,
                             interface HallInterface client ?i_hall,
                             interface QEIInterface client ?i_qei,
                             interface BISSInterface client ?i_biss,
                             interface AMSInterface client ?i_ams,
                             interface WatchdogInterface client i_watchdog,
                             interface MotorcontrolInterface server i_motorcontrol[MOTOR_CTLR_INTRFCE_CNT]);

#endif
