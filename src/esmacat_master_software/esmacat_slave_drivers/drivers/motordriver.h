/** @file
 *  @brief Contains all the declarations required for the Esmacat Motor Driver slave
 * */
#ifndef ESMACAT_MOTORDRIVER_H
#define ESMACAT_MOTORDRIVER_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique vendor ID assigned by the EtherCAT Technology Group to Harmonic Bionics Inc.*/
#define ESMACAT_MOTOR_DRIVER_VENDOR_ID      0x0062696f

/** @brief Unique product code for Esmacat Loadcell Interface slave required for EtherCAT */
#define ESMACAT_MOTOR_DRIVER_PRODUCT_ID     0x00020101

/** @brief Minimum value for the PWM output */
#define ESMACAT_MD_MIN_PWM_OUTPUT      0

/** @brief Maximum value for the PWM output */
#define ESMACAT_MD_MAX_PWM_OUTPUT      10000

/** @brief Maximum ESCON current setpoint value */
#define ESMACAT_MD_MAX_SET_VALUE       1

/** @brief Minimum ESCON current setpoint value */
#define ESMACAT_MD_MIN_SET_VALUE       -1

/** @brief Maximum number of external analog input sources for the slave */
#define ESMACAT_MD_NUMBER_OF_EXT_ANALOG_INPUT       2

/** @brief Maximum number of ESCON analog input sources for the slave */
#define ESMACAT_MD_NUMBER_OF_ESCON_ANALOG_INPUT     2

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "base_module.h"
#include "error_list.h"
#include <iostream>
#include <math.h>
#include "master.h"
/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/** @brief Class containing all the variables and functions used by the Esmacat Motor Driver slave
 */
class esmacat_motor_driver : public esmacat_base_module
{
    /** Array containing raw readings for analog inputs from 2 external sources; IN_MD_ANLG on ESCON datasheet */
    uint16_t raw_external_analog_input_reading[ESMACAT_MD_NUMBER_OF_EXT_ANALOG_INPUT];
    /** Array containing raw readings for 2 analog inputs from ESCON; IN_ESCON_ANLG on ESCON datasheet*/
    uint16_t raw_escon_analog_input_reading[ESMACAT_MD_NUMBER_OF_ESCON_ANALOG_INPUT];
    /** Input from the LS7366 quadrature encoder reader */
    int32_t encoder_reading;
    /** Enable ESCON (1), disable ESCON (0); OUT_MD_enable on ESCON datasheet */
    bool escon_enable;
    /** Direction of the motor; OUT_MD_direction on ESCON datasheet */
    bool escon_direction;
    /** Status from ESCON;  Fault = 1; IN_MD_FAULT on ESCON datasheet */
    bool escon_fault;
    /** Proportional gain for the position control loop */
    float position_control_p_gain;
    /** Integral gain for the position control loop */
    float position_control_i_gain;
    /** Derivative gain for the position control loop */
    float position_control_d_gain;
    /** Summation of the position error over time- to be used for the control loop */
    float integ_position_error;
     /** Value of position error at the previous time instant-  to be used for the control loop */
    float prev_position_error;
    /** Max value for Summation of the position error over time- used to prevent integral windup*/
    float max_integ_position_error;
    /** Max allowable change in position in quadrature counts per interval*/
    float max_position_change_qc_per_interval;

    void ecat_data_process_motor_driver(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);

protected:
    /** Current Setpoint for ESCON: OUT_setpoint on the ESCON datasheet*/
    int32_t escon_current_setpoint;
    /** Time interval for the loop computations */
    float esmacat_app_one_cycle_time_sec;

    esmacat_err set_escon_pwm(unsigned int pwm_value);
    uint16_t get_raw_ext_analog_input(int ai_index);
    uint16_t get_raw_escon_analog_input(int ai_index);

public:
    /** Contains a product code of the Esmacat slave */
    uint32_t esmacat_slave_product_id = ESMACAT_MOTOR_DRIVER_PRODUCT_ID;
    /** Contains the vendor ID issued by the EtherCAT Technology group */
    uint32_t esmacat_slave_vendor_id = ESMACAT_MOTOR_DRIVER_VENDOR_ID;

    esmacat_motor_driver();
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
    void enable_escon(bool enable_on);
    esmacat_err set_escon_current_setpoint(float setpoint);  // set setvalue of ESCON between -1 and 1
    esmacat_err set_desired_position(int32_t desired_position);
    float get_analog_input_from_external_source_mV(int index_of_analog_input, esmacat_err& error);
    float get_analog_input_from_external_source_mV(int index_of_analog_input);
    float get_analog_input_from_escon_mV(int index_of_analog_input,  esmacat_err& error);
    float get_analog_input_from_escon_mV(int index_of_analog_input);
    int32_t get_encoder_counter();
    bool get_escon_fault();

    void configure_slave_encoder_clear();
    void set_position_control_pid_gain(float p, float i, float d);

    void set_max_allowable_integrated_error_for_position_control(uint32_t max_value);
    void set_max_velocity_in_position_control_qc_p_ms(uint32_t v);
    uint32_t get_esmacat_product_id();
    uint32_t get_esmacat_vendor_id();

};
#endif // ESMACAT_MOTORDRIVER_H
