/** @file
 * @brief This file contains all the definitions of the functions used by the Esmacat base module class
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "base_module.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the Esmacat Base module class
 *
 * The system inputs and outputs are set to 0. All the I/Os are
 * configured to Outputs. Values that were inherited from the parent
 * class are also set to 0
 */

esmacat_base_module::esmacat_base_module()
{
    sys_digital_in = 0;
    for (int i=0;i<ESMACAT_BASE_NUMBER_OF_DIO;i++){
        digital_data_in[i] = 0;
        digital_data_out[i] = 0;
        ICAT_base_dio_direction[i] = IO_OUTPUT;
    }
    user_led_out = 0;

    IN_system_parameter_type = 0;
    IN_system_parameter_value = 0;
    OUT_system_parameter_type = 0;
    OUT_system_parameter_value = 0;
}

/** @brief Turns on/off the digital output of the system
 *
 * @param Boolean input indicating whether User LED must be turned on
 */
void esmacat_base_module::set_usr_led(bool led_on)
{
    user_led_out = led_on;
}

/** @brief Assigns the the specified digital output boolean value to the specified output signal
 *
 * Has error handling- ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE if input index is out of range,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_OUTPUT if value is being assigned to an input not output,
 * NO_ERR indicates successful completion
 *
 * @param index_of_digital_output Index of the digital output signal to be assigned
 * @param digital_output_value Boolean value to be assigned to the digital output signal
 * @return Error status of the function (ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_OUTPUT, NO_ERR)
 */
esmacat_err esmacat_base_module::set_digital_output(int index_of_digital_output, bool digital_output_value)
{
    esmacat_err error;
    if (index_of_digital_output < 0 || index_of_digital_output>ESMACAT_BASE_NUMBER_OF_DIO)
    {
        error = ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE;
    }
    else
    {
        if (ICAT_base_dio_direction[index_of_digital_output] != IO_OUTPUT )
        {
            error = ERR_BASE_MODULE_DIO_DIRECTION_NOT_OUTPUT;
        }
        else
        {
            digital_data_out[index_of_digital_output] = digital_output_value;
            error = NO_ERR;
        }
    }
    return error;
}

/** @brief Reads the value of the specified input signal
 *
 * Has error handling- ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE if input index is out of range,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_input if value is being read is not an input,
 * NO_ERR indicates successful completion
 *
 * @param index_of_digital_input Index of the digital output signal to be assigned
 * @param error Status of the function (ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_INPUT, NO_ERR)
 * @return Boolean value read at the specified index
 */
bool esmacat_base_module::get_digital_input(int index_of_digital_input, esmacat_err& error)
{
    if (index_of_digital_input < 0 || index_of_digital_input >ESMACAT_BASE_NUMBER_OF_DIO)
    {
        error = ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE;
        return 0;
    }
    else
    {
        if (ICAT_base_dio_direction[index_of_digital_input] != IO_INPUT )
        {
            error = ERR_BASE_MODULE_DIO_DIRECTION_NOT_INPUT;
            return 0;
        }
        else
        {
            return digital_data_in[index_of_digital_input];
            error = NO_ERR;
        }
    }
}


/** @brief Reads the value of the specified input signal
 *
 * No error handling
 *
 * @param index_of_digital_input Index of the digital output signal to be assigned
 * @return Boolean value read at the specified index
 */
bool esmacat_base_module::get_digital_input(int index_of_digital_input)
{
    esmacat_err error = NO_ERR;
    return get_digital_input(index_of_digital_input, error);
}

/** @brief Private function- Data exchange process that is specific to the Esmacat base slave
 *
 *  [***hardware dependent - do not change***] This function queues the values to be sent to the
 * firmware, and also decodes packets received. sys_digital_in, IN_BASE_DO, user_led_out and digital_data_out
 * are set
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 **/
void esmacat_base_module::ecat_data_process_base_module(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs)
{

    //  decoding and encoding ethercat data
    uint8_t temp_digital_in_output_bundle = 0;

    // get digital data, and extract the sys_digital_in or escon_fault
    bool temp_digital_array_unbundled[8];

    // this section is for reading the received data

    // a byte of data is read in
    temp_digital_in_output_bundle = *ec_slave_inputs;
    for (int i=0;i<8;i++)
    {//each bit is read into the unbundled value as a boolean
        temp_digital_array_unbundled[i]=  (( temp_digital_in_output_bundle & (0b00000001 << i)) != 0);
    }
    //LSB of first byte contains sys_digital_in
    sys_digital_in = temp_digital_array_unbundled[0];
    //next 7 bits contain the digital_data_in values
    for (int i=0;i<ESMACAT_BASE_NUMBER_OF_DIO;i++)
    {
        digital_data_in[i]=  temp_digital_array_unbundled[i+1];
    }

    //Skip byte at location 1
    //Locations 2 and 3 contain the system parameter type being read i.e Byte 3, Byte 2
    IN_system_parameter_type = *(ec_slave_inputs+3);
    IN_system_parameter_type = (IN_system_parameter_type << 8) +  *(ec_slave_inputs+2);

    //Locations 4 and 5 contain the system parameter value i.e Byte 5, Byte 4
    IN_system_parameter_value= *(ec_slave_inputs+5);
    IN_system_parameter_value = (IN_system_parameter_value << 8) +  *(ec_slave_inputs+4);


    // this section is for preparing the output to be transmitted
    //the first byte being transmitted contains digital_data_out concatenated with user_led_out
    temp_digital_in_output_bundle = 0;
    temp_digital_in_output_bundle = user_led_out;                        // encode for usr led
    for (int i=0;i<ESMACAT_BASE_NUMBER_OF_DIO;i++)
    {
        temp_digital_in_output_bundle = temp_digital_in_output_bundle  | (digital_data_out[i] << i+1);
    }
    //prepare data to be sent
    *(ec_slave_outputs) = temp_digital_in_output_bundle ;

    //now append the type and value of the system parameter as
    // Type [LSB], Type[MSB], Value[LSB], Value[MSB]
    *(ec_slave_outputs+2)  = OUT_system_parameter_type & 0x00ff;        // encode system param type
    *(ec_slave_outputs+3)  = OUT_system_parameter_type >> 8;
    *(ec_slave_outputs+4)  = OUT_system_parameter_value & 0x00ff;       // encode system param value
    *(ec_slave_outputs+5)  = OUT_system_parameter_value >> 8;

}

/** @brief Sets the direction of the digital I/O on the slave
 *
 * Also queues the values to be sent to the firmware [** do not change**]
 * @param dio_direction Array of ESMACAT_BASE_NUMBER_OF_DIO number of values specifying
 * which signals are inputs and which are set as outputs
 */
void esmacat_base_module::configure_slave_dio_direction(IO_Direction dio_direction[ESMACAT_BASE_NUMBER_OF_DIO])
{
    uint16_t dio_direction_bit_array = 0;
    for (int i=0;i<ESMACAT_BASE_NUMBER_OF_DIO;i++)
    {
        ICAT_base_dio_direction[i] = dio_direction[i];
        dio_direction_bit_array = dio_direction_bit_array | (dio_direction[i] << i);
    }
    add_system_parameters_in_queue(0x0011,dio_direction_bit_array);
}

/** @brief Virtual function to be overwritten by the specific slave classes */
void esmacat_base_module::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop)
{

}
