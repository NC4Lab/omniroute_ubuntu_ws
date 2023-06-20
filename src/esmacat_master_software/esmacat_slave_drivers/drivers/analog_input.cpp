/** @file
 * @brief Contains definitions of functions used for the Esmacat Analog Input slave
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "analog_input.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the class that holds the initialization
 *
 * Intializes the raw ADC readings for all 16 or ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX
 * channels to 0. And the product code and Vendor ID for the slave are set
 *
 * Sets a default full-scale-range and offset for default type of
 * single-ended input signals in the range of -12V to +12V
 */
esmacat_analog_input_slave::esmacat_analog_input_slave()
{
    esmacat_slave_product_id = ESMACAT_ANALOG_INPUT_PRODUCT_ID;
    esmacat_slave_vendor_id = ESMACAT_ANALOG_INPUT_VENDOR_ID;
    std::cout <<"Esmacat 16CH Analog Input Object is created" << std::endl;
    for(int i=0;i<ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX;i++)
    {
        raw_adc_reading_by_channel[i] = 0;
    }
    // default fsr and offset for all channels of esmacat ai board
    for(int i=0;i<ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX;i++)
    {
        ai_full_scale_range[i] = 6.0 * 4096.0;
        ai_offset[i] = -3.0 * 4096.0;
    }
}


 /** @brief Definition of inherited data exchange function for Esmacat slave
  *
  * Combines data exchange process of the base module and the analog input slaves
  *
  * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
  * @param oloop No. of output bytes
  * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
  * @param iloop No. of input bytes
  */
void esmacat_analog_input_slave::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop)
{
    ecat_data_process_base_module(ec_slave_outputs,ec_slave_inputs);    // for base module
    ecat_data_process_analog_input(ec_slave_outputs,ec_slave_inputs);   // for anlog input module
}


/** @brief Private function- Data exchange process that is specific to the Esmacat Analog Input slave
 *
 * Skip the first 6 bytes of the input pointer [***hardware dependent - do not change***]
 *
 * Following that every pair of bytes is a 16 bit ADC reading for the corresponding channel
 *
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 **/
void esmacat_analog_input_slave::ecat_data_process_analog_input(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs)
{
        for (int i=0;i<ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX;i++)
        {
            // MSB of the ADC reading of the channel
            raw_adc_reading_by_channel[i] = *(ec_slave_inputs+7+i*2);
             // Concatenate with LSB of ADC reading for the channel
            raw_adc_reading_by_channel[i] = (raw_adc_reading_by_channel[i] << 8) |  *(ec_slave_inputs+6+i*2);
        }
}


/** @brief Private function - Reads the ADC channels
 *
 * @param channel_index Index of the channel in the Esmacat Analog Input slave
 * @param error Contains the error message to demonstrate success of data acquisition
 * (ERR_ANALOG_INPUT_CHANNEL_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Raw ADC reading for the desired channel on the Esmacat Analog Input slave
 */
uint16_t esmacat_analog_input_slave::get_raw_analog_input(int channel_index, esmacat_err& error)
{
    uint16_t raw_adc_reading;
    //if index is out of range
    if (channel_index < 0 || channel_index > ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX )
    {
        error = ERR_ANALOG_INPUT_CHANNEL_INDEX_OUT_OF_RANGE;
    }
    else
    {
        raw_adc_reading = raw_adc_reading_by_channel[channel_index];
        error =  NO_ERR;
    }
    return raw_adc_reading;
}

/** @brief Private function - Configures the type and range of ADC channels with error handling
 *
 * Further information can be obtained from the MAX1300 ADC datasheet.
 * Vref = 4096 mV
 * The programmable ranges of the ADC are as follows:
 *
 * Single-ended, 0 to 3 * Vref/2 range
 *
 * Single-ended, 0 to 3 * Vref
 *
 * Single-ended, -3 * Vref/4 to 3 * Vref/4
 *
 * Single-ended, -3 * Vref/2 to 0
 *
 * Single-ended, -3 * Vref/2 to 3 * Vref/2
 *
 * Single-ended, -3 * Vref to 0
 *
 * Single-ended, -3 * Vref to 3 * Vref
 *
 * Differential, -3 * Vref/2 to 3 * Vref/2
 *
 * Differential, -3 * Vref to 3 * Vref
 *
 * Differential, -6 * Vref to 6 * Vref
 *
 * @param ai_config_ch0to3 Type and range of the signals of channels 0 - 3
 * @param ai_config_ch4to7 Type and range of the signals of channels 4 - 7
 * @param ai_config_ch8to11 Type and range of the signals of channels 8 - 11
 * @param ai_config_ch12to15 Type and range of the signals of channels 12 - 15
 * @return Error message generated during configuration (ERR_ANALOG_INPUT_NO_SUCH_CONFIG or NO_ERR)
 */
esmacat_err esmacat_analog_input_slave::configure_slave_analog_input( AdcChannelTypeandRange ai_config_ch0to3,
                                                                           AdcChannelTypeandRange ai_config_ch4to7,
                                                                           AdcChannelTypeandRange ai_config_ch8to11,
                                                                           AdcChannelTypeandRange ai_config_ch12to15)
{
    uint16_t esmacat_analog_input_config = 0;
    AdcChannelTypeandRange ai_config_array[4];
    ai_config_array[0] = ai_config_ch0to3;
    ai_config_array[1] = ai_config_ch4to7;
    ai_config_array[2] = ai_config_ch8to11;
    ai_config_array[3] = ai_config_ch12to15;
	
    for (int i=0;i<4;i++){
        if(ai_config_array[i] == SINGLE_ENDED_0V_POS6V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 1.5;
                ai_offset[i*4+j] = 0.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_0V_POS12V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = 0.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG3V_POS3V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 1.5;
                ai_offset[i*4+j] = -0.75 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG6V_0V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 1.5;
                ai_offset[i*4+j] = -1.5 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG6V_POS6V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = -1.5 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG12V_0V)
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = -3.0 * 4096.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == SINGLE_ENDED_NEG12V_POS12V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 6.0;
                ai_offset[i*4+j] = 4096.0 * -3.0;
                ai_single_ended_or_diff[i*4+j] = SINGLE_ENDED_INPUT;
            }
        }
        else if(ai_config_array[i] == DIFF_NEG6V_POS6V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 3.0;
                ai_offset[i*4+j] = 4096.0 * -1.5;
                ai_single_ended_or_diff[i*4+j] = DIFFERENTIAL_INPUT;
            }
        }
        else if(ai_config_array[i] == DIFF_NEG12V_POS12V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 6.0;
                ai_offset[i*4+j] = 4096.0 * -3.0;
                ai_single_ended_or_diff[i*4+j] = DIFFERENTIAL_INPUT;
            }
        }
        else if(ai_config_array[i] == DIFF_NEG24V_POS24V )
        {
            for(int j=0;j<4;j++){
                ai_full_scale_range[i*4+j] = 4096.0 * 12.0;
                ai_offset[i*4+j] = 4096.0 * -6.0;
                ai_single_ended_or_diff[i*4+j] = DIFFERENTIAL_INPUT;
            }
        }
        else
        {
            return  ERR_ANALOG_INPUT_INVALID_CONFIG;
        }
    }
    //stage values for transmission
    esmacat_analog_input_config = ai_config_ch12to15;
    esmacat_analog_input_config = (esmacat_analog_input_config <<4) | ai_config_ch8to11;
    esmacat_analog_input_config = (esmacat_analog_input_config <<4) | ai_config_ch4to7;
    esmacat_analog_input_config = (esmacat_analog_input_config <<4) | ai_config_ch0to3;

    add_system_parameters_in_queue(ESMACAT_ANALOG_INPUT_CONFIG_REGISTER,esmacat_analog_input_config);
    return NO_ERR;
}

/** @brief Configures the type and range of ADC channels with error handling
 *
 * @param ai_option Object containing the type and voltage range for all 16 or
 * ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX channels
 * @return Error message generated during configuration (ERR_ANALOG_INPUT_NO_SUCH_CONFIG or NO_ERR)
 */
esmacat_err esmacat_analog_input_slave::configure_slave_analog_input(esmacat_analog_input_channel_config_T channel_config)
{
	esmacat_err error;
    error = configure_slave_analog_input(channel_config.ch0to3_config, channel_config.ch4to7_config,
                                            channel_config.ch8to11_config, channel_config.ch12to15_config);
	return error;

}

/** @brief Reads analog input value in mV and provides error handling
 *
 * @param channel_index Index of the ADC channel to be read
 * @param error Contains the error message to demonstrate success of data acquisition
  * (ERR_ANALOG_INPUT_CHANNEL_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Voltage reading of input channel in mV
 */
float esmacat_analog_input_slave::get_analog_input_mV(int channel_index, esmacat_err& error)
{
    uint16_t raw_ai_value;
    esmacat_err result_of_get_raw_analog_input;
    float analog_input_reading_mV;

    // if it's a differential signal, the channel at which the reading will be received
    // is the even numbered channel. Refer to MAX1300 ADC datasheet
    if ( ai_single_ended_or_diff[channel_index] != SINGLE_ENDED_INPUT)
    {
        if ( channel_index %2 ==1)
        {
            channel_index--;
        }
    }
    //get the raw ADC reading for the channel
    raw_ai_value = get_raw_analog_input( channel_index, result_of_get_raw_analog_input);
    if ( result_of_get_raw_analog_input  == NO_ERR )
    {   //if no error, convert it to mV, and set error to NO_ERR
        analog_input_reading_mV = ai_offset[channel_index] + ai_full_scale_range[channel_index] * ( (float)raw_ai_value/(float)65535 );
        error = NO_ERR;
    }
    else
    {   //if error then set value to 0 and set the error
        analog_input_reading_mV  = 0;
        error = result_of_get_raw_analog_input;
    }

    return analog_input_reading_mV;

}

/** @brief Reads analog input value in mV. No error handling
 * @param Index of the channel to be read
 * @return Voltage reading (mV) of the input channel
 */
float esmacat_analog_input_slave::get_analog_input_mV(int channel_index)
{
    esmacat_err error = NO_ERR;
    return get_analog_input_mV(channel_index, error);

}

/** @brief Reads the product id of the slave
 * @return Product ID of the slave
 */
uint32_t esmacat_analog_input_slave::get_esmacat_product_id()
{
    return esmacat_slave_product_id;
}

 /** @brief Returns the vendor ID of the slave
  * @return Vendor ID of the slave
  */
uint32_t esmacat_analog_input_slave::get_esmacat_vendor_id()
{
    return esmacat_slave_vendor_id;
}
