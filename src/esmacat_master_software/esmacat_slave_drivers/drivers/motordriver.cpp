/** @file
 * @brief This file contains the definitions of the functions used in the Esmacat Motor
 * Driver slave
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "motordriver.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Private function- Data exchange process that is specific to the Esmacat Motor Driver slave
 *
 *  [***hardware dependent - do not change***] This function queues the values to be sent to the
 * firmware, and also decodes packets received. Motor Driver fault, analog input readings from both
 * the external source and the ESCON, and quadrature encoder readings are handled here.
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 **/
void esmacat_motor_driver::ecat_data_process_motor_driver(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs)
{
    //  decoding and encoding ethercat data
    uint8_t temp_digital_in_output_bundle = 0;

    // get digital data, and extract the IN_MD_FAULT or escon_fault
    bool temp_digital_array_unbundled[8];

    // this section is for reading the received data

    // a byte of data is read in
    temp_digital_in_output_bundle = *(ec_slave_inputs+1);
    for (int i=0;i<8;i++)
    {   //each bit is read into the unbundled value as a boolean
        temp_digital_array_unbundled[i]=  (( temp_digital_in_output_bundle & (0b00000001 << i)) != 0);
    }
    escon_fault = temp_digital_array_unbundled[0];

    //Skip the first 6 bytes of the input pointer [***hardware dependent - do not change***]
    //the first 4 bytes correspond to the two external source analog inputs
    for (int i=0;i<2;i++)
    {
        raw_external_analog_input_reading[i] = *(ec_slave_inputs+7 + 2*i );
        raw_external_analog_input_reading[i] = (raw_external_analog_input_reading[i] << 8) + *(ec_slave_inputs + 6 + 2*i);
    }
    //next 4 bytes correspond to the readings of the two ESCON analog inputs
    for (int i=0;i<2;i++)
    {
        raw_escon_analog_input_reading[i] = *(ec_slave_inputs+11 + 2*i );
        raw_escon_analog_input_reading[i] = (raw_escon_analog_input_reading[i] << 8) + *(ec_slave_inputs + 10 + 2*i);
    }
    //The next 4 bytes correspond to the quadrature encoder reading
    encoder_reading = *(ec_slave_inputs+17);
    encoder_reading = (encoder_reading<<8) + *(ec_slave_inputs+16);
    encoder_reading = (encoder_reading<<8) + *(ec_slave_inputs+15);
    encoder_reading = (encoder_reading<<8) + *(ec_slave_inputs+14);

    // this section is for preparing the output to be transmitted

    // encode for ESCON enable
    temp_digital_in_output_bundle = escon_enable;
    // encode for ESCON direction
    temp_digital_in_output_bundle = temp_digital_in_output_bundle  | (escon_direction<<1);
    //the first byte contains the direction and enable in bits 1 and 0 respectively
    *(ec_slave_outputs+1)  = temp_digital_in_output_bundle;
    // 4 bytes of the setpoint are sent across
    *(ec_slave_outputs+6)  = (escon_current_setpoint & 0x000000ff) >> 0;
    *(ec_slave_outputs+7)  = (escon_current_setpoint & 0x0000ff00) >> 8;
    *(ec_slave_outputs+8)  = (escon_current_setpoint & 0x00ff0000) >> 16;
    *(ec_slave_outputs+9)  = (escon_current_setpoint & 0xff000000) >> 24;
}

/** @brief Protected funciton - Sets the PWM duty cycle of the ESCON within the specified bounds
 *
 * It ensures that the provided pwm setpoint does not exceed ESMACAT_MD_MAX_PWM_OUTPUT
 * and does not go below ESMACAT_MD_MIN_PWM_OUTPUT. Also provides error handling.
 *
 * @param pwm_value Dutycycle setpoint for the ESCON; 10000 implies duty_cycle of 100%
 * @return Status of the operation- NO_ERR, ERR_MOTOR_DRIVER_MAX_PWM_REACHED,
 * ERR_MOTOR_DRIVER_MIN_PWM_REACHED
 */
esmacat_err esmacat_motor_driver::set_escon_pwm(unsigned int pwm_value)
{
    if (pwm_value > ESMACAT_MD_MAX_PWM_OUTPUT){
        pwm_value = ESMACAT_MD_MAX_PWM_OUTPUT;
        return ERR_MOTOR_DRIVER_MAX_PWM_REACHED;
    }
    else if(pwm_value < ESMACAT_MD_MIN_PWM_OUTPUT)   {
        pwm_value = ESMACAT_MD_MIN_PWM_OUTPUT;
        return ERR_MOTOR_DRIVER_MIN_PWM_REACHED;
    }
    else {
        escon_current_setpoint = pwm_value;
        return NO_ERR;
    }
}

/** @brief Protected Function- Reads the specified raw analog input from external source
 * @param Index of the analog input to be read
 * @return Reading of the specified analog input
 */
uint16_t esmacat_motor_driver::get_raw_ext_analog_input(int ai_index)
{
    return raw_external_analog_input_reading[ai_index];
}

/** @brief Protected Function- Reads the specified raw analog input from ESCON
 * @param Index of the analog input to be read
 * @return Reading of the specified analog input
 */
uint16_t esmacat_motor_driver::get_raw_escon_analog_input(int ai_index)
{
    return raw_escon_analog_input_reading[ai_index];
}


/** @brief Constructor for the class that holds the initialization
 *
 * Intializes the raw ADC readings for the analog inputs from both
 * external sources and the ESCON. Also sets the product code and vendor ID.
 * Clears the encoder, disables the ESCON and clears its inputs and fault,
 * resets all the values used in the control loop, and sets
 * the max values for the integrated position error, and the velocity.
 * It also sets the time interval for the control loop computations.
 */

esmacat_motor_driver::esmacat_motor_driver()
{
    std::cout <<"Esmacat Motor Driver Object is created" << std::endl;
    for (int i=0;i<2;i++){
        raw_external_analog_input_reading[i] = 0;
        raw_escon_analog_input_reading[i] = 0;
    }
    esmacat_slave_product_id = ESMACAT_MOTOR_DRIVER_PRODUCT_ID;
    esmacat_slave_vendor_id = ESMACAT_MOTOR_DRIVER_VENDOR_ID;
    encoder_reading = 0;
    escon_enable = 0;
    escon_direction = 0;
    escon_current_setpoint = 0;
    escon_fault = 0;
    integ_position_error = 0;
    prev_position_error = 0;
    position_control_p_gain = 0;
    position_control_i_gain = 0;
    position_control_d_gain = 0;
    max_integ_position_error = 10000.0;
    max_position_change_qc_per_interval = 1000;
    esmacat_app_one_cycle_time_sec = (float)ESMACAT_TIME_PERIOD_US/(float)1000;
}

/** @brief Enables/disables the escon
 * @param enable 1= Enable, 0 = Disable
 */
void esmacat_motor_driver::enable_escon(bool enable)
{
    escon_enable  = enable;
}

/** @brief Provides current setpoint to the ESCON
 *
 * Verifies that the setpoint is within the specified bounds. Limits the value
 * to the bounds specified. The ESCON in only driven from 10% to 90% of its
 * usable range
 * @param setpoint Current setpoint value for the ESCON
 * (Range = [ESMACAT_MD_MIN_SET_VALUE, ESMACAT_MD_MAX_SET_VALUE])
 * @return Status of the function. ERR_MOTOR_DRIVER_SETPOINT_OUT_OF_RANGE
 * if the setpoint was out of range, else NO_ERR
 */
esmacat_err esmacat_motor_driver::set_escon_current_setpoint(float setpoint)
{
    esmacat_err e = NO_ERR;
    if (setpoint > ESMACAT_MD_MAX_SET_VALUE ){
        setpoint = ESMACAT_MD_MAX_SET_VALUE;
        e= ERR_MOTOR_DRIVER_SETPOINT_OUT_OF_RANGE;
    }
    else if (setpoint < ESMACAT_MD_MIN_SET_VALUE ){
        setpoint = ESMACAT_MD_MIN_SET_VALUE ;
        e = ERR_MOTOR_DRIVER_SETPOINT_OUT_OF_RANGE;
    }

    // for the ESCON PWM, range = 0 to 10000. The range of the input
    // is mapped to 1000 to 9000 to ensure safe operation of the ESCON
    set_escon_pwm( (setpoint+1)/2.0 * 8000 + 1000);

    return e;
}

/** @brief Reads analog input value in mV and provides error handling
 *
 * @param analog_input_index Index of the external source analog input to be read
 * @param error Contains the error message to demonstrate success of data acquisition
  * (ERR_MOTOR_DRIVER_EXT_ANALOG_INPUT_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Voltage reading (mV) of the selected external source analog input
 */
float esmacat_motor_driver::get_analog_input_from_external_source_mV(int analog_input_index, esmacat_err& error)
{
    float adc_reading_mV = 0;
    if (analog_input_index > ESMACAT_MD_NUMBER_OF_EXT_ANALOG_INPUT || analog_input_index < 0)
    {
        error = ERR_MOTOR_DRIVER_EXT_ANALOG_INPUT_INDEX_OUT_OF_RANGE;
        adc_reading_mV = 0;
    }
    else
    {
        error = NO_ERR;
        adc_reading_mV = 5.0*get_raw_ext_analog_input(analog_input_index)/4096.0;
    }
    return adc_reading_mV;
}

/** @brief Reads analog input value from external source of specified index in mV.
 * No error handling
 * @param analog_input_index Index of the external source analog input to be read
 * @return Voltage reading (mV) of the selected external source analog input
 */
float esmacat_motor_driver::get_analog_input_from_external_source_mV(int analog_input_index)
{
    esmacat_err error = NO_ERR;
    return get_analog_input_from_external_source_mV(analog_input_index, error);
}

/** @brief Reads analog input value in mV and provides error handling
 *
 * @param analog_input_index Index of the ESCON analog input to be read
 * @param error Contains the error message to demonstrate success of data acquisition
  * (ERR_MOTOR_DRIVER_ESCON_ANALOG_INPUT_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Voltage reading (mV) of the selected ESCON analog input
 */
float esmacat_motor_driver::get_analog_input_from_escon_mV(int analog_input_index, esmacat_err& error)
{
    float adc_reading_mV = 0;
    if (analog_input_index > ESMACAT_MD_NUMBER_OF_ESCON_ANALOG_INPUT || analog_input_index < 0)
    {
        error = ERR_MOTOR_DRIVER_ESCON_ANALOG_INPUT_INDEX_OUT_OF_RANGE;
        adc_reading_mV = 0;
    }
    else
    {
        error = NO_ERR;
        adc_reading_mV = 5.0*get_raw_escon_analog_input(analog_input_index)/4096.0;
    }
    return adc_reading_mV;
}

/** @brief Reads analog input value from ESCON source of specified index in mV.
 * No error handling
 * @param analog_input_index Index of the ESCON analog input to be read
 * @return Voltage reading (mV) of the selected ESCON analog input
 */
float esmacat_motor_driver::get_analog_input_from_escon_mV(int analog_input_index)
{
    esmacat_err error = NO_ERR;
    return get_analog_input_from_escon_mV(analog_input_index, error);
}

/** @brief Obtains the encoder reading
 * @return Encoder reading in ticks
 */
int32_t esmacat_motor_driver::get_encoder_counter()
{
    return encoder_reading;
}

/** @brief Definition of inherited data exchange function for Esmacat slave
 *
 * Combines data exchange process of the base module and the motor driver slaves
 *
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param oloop No. of output bytes
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 * @param iloop No. of input bytes
 */
void esmacat_motor_driver::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){
    ecat_data_process_base_module(ec_slave_outputs,ec_slave_inputs);    // for base module
    ecat_data_process_motor_driver(ec_slave_outputs,ec_slave_inputs);   // for motor driver
}


/** @brief Position control PID loop that supplies the ESCON with its setpoint
 *
 * The rate of change of position is limited by using max_position_change_qc_per_interval.
 *
 * Output Current Setpoint for ESCON = pterm + iterm + dterm
 *
 * error = position setpoint - measured position
 *
 * pterm = kp * error
 *
 * iterm = ki * (integral of error over time)
 *
 * dterm = kd * (rate of change of error)
 *
 * The iterm is bounded by  [-max_integ_position_error, max_integ_position_error] to
 * prevent integral windup.
 *
 * The final current setpoint sent to the ESCON is also bounded to
 * [ESMACAT_MD_MIN_SET_VALUE, ESMACAT_MD_MAX_SET_VALUE]
 * @param desired_position Position setpoint in quadrature counts
 * @return Error status of the function (Currently only returns NO_ERR if successful)
 */
esmacat_err esmacat_motor_driver::set_desired_position(int32_t desired_position){

    int32_t filtered_desired_position = 0;       // low-pass filtered desired position
    int32_t curr_pos = get_encoder_counter();   // current angle
    float pterm = 0;
    float iterm = 0;
    float dterm = 0;

    // this converts a step input into a ramp input
    if ( (desired_position - curr_pos) > max_position_change_qc_per_interval)
    {
        filtered_desired_position = curr_pos + max_position_change_qc_per_interval;
    }
    else if ( (desired_position - curr_pos) < -max_position_change_qc_per_interval)
    {
        filtered_desired_position = curr_pos - max_position_change_qc_per_interval;
    }
    else
    {
        filtered_desired_position = desired_position;
    }

    double pos_error = curr_pos-filtered_desired_position;
    //double non_filtered_pos_error = curr_pos - desired_position;
    double deriv_error = (pos_error- prev_position_error) / esmacat_app_one_cycle_time_sec;
    prev_position_error = pos_error;
    integ_position_error += pos_error * esmacat_app_one_cycle_time_sec;

    //cap the error summation over time to the bounds [-max_integ_position_error, max_integ_position_error]
    if (integ_position_error > max_integ_position_error)
    {
        integ_position_error = max_integ_position_error;
    }
    if (integ_position_error < -max_integ_position_error )
    {
        integ_position_error = -max_integ_position_error;
    }

    pterm = position_control_p_gain*pos_error;
    dterm = position_control_d_gain*deriv_error;
    iterm = position_control_i_gain*integ_position_error;

    float current_setpoint = pterm + dterm + iterm;

    //cap the setpoint at between the bounds of [ESMACAT_MD_MIN_SET_VALUE, ESMACAT_MD_MAX_SET_VALUE]
    if (current_setpoint > ESMACAT_MD_MAX_SET_VALUE)
    {
        current_setpoint = ESMACAT_MD_MAX_SET_VALUE;
    }
    if (current_setpoint < ESMACAT_MD_MIN_SET_VALUE)
    {
        current_setpoint = ESMACAT_MD_MIN_SET_VALUE;
    }
    //send the derived current setpoint to the ESCON
    set_escon_current_setpoint( current_setpoint );

    return NO_ERR;
}

/** @brief Clears the encoder [Do not change]
 *
 * This function should not be modified since it is tied to the firmware
 * for the slave
 */
void esmacat_motor_driver::configure_slave_encoder_clear()
{
    add_system_parameters_in_queue(0x0500,0);
}

/** @brief Sets the proportional, integral and derivative gains for the control loop
 * for position
 *
 * The gains are multiplied by a factor of 0.001 prior to being applied
 * @param p Proportional gain
 * @param i Integral gain
 * @param d Derivative gain
 */
void esmacat_motor_driver::set_position_control_pid_gain(float p, float i, float d){
    position_control_p_gain = p* 0.001;
    position_control_d_gain = d* 0.001;
    position_control_i_gain = i* 0.001;
}

/** @brief Obtains the fault status of the ESCON
 * @return ESCON fault status
 */
bool esmacat_motor_driver::get_escon_fault()
{
    return escon_fault;
}

/** @brief Sets the maximum allowed integral error for position control loop
 * This may be used to prevent integral windup
 * @param max_value Maximum Allowable integrated position error
 */
void esmacat_motor_driver::set_max_allowable_integrated_error_for_position_control(uint32_t max_value)
{
    max_integ_position_error = max_value;
}
/** @brief Sets the maximum allowed velocity in quadrature counts per ms
 * for the motor
 * @param max_value maximum allowed velocity in quadrature counts per ms
 */
void esmacat_motor_driver::set_max_velocity_in_position_control_qc_p_ms(uint32_t max_value)
{
    max_position_change_qc_per_interval=max_value;
}

/** @brief Reads the product id of the slave
 * @return Product ID of the slave
 */
uint32_t esmacat_motor_driver::get_esmacat_product_id()
{
    return esmacat_slave_product_id;
}

/** @brief Returns the vendor ID of the slave
 * @return Vendor ID of the slave
 */
uint32_t esmacat_motor_driver::get_esmacat_vendor_id()
{
    return esmacat_slave_vendor_id;
}
