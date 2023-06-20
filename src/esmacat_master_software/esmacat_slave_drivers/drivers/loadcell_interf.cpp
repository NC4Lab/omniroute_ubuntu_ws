/** @file
 *  @brief Contains definitions of functions used for the Esmacat Loadcell Interface slave
*/


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "loadcell_interf.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the class that holds the initialization
 *
 * Intializes the raw ADC readings for all 16 channels to 0
 * And the product code and Vendor ID for the slave are set
 */
esmacat_loadcell_interface::esmacat_loadcell_interface()
{
    esmacat_slave_product_id = ESMACAT_LOADCELL_INTERFACE_PRODUCT_ID;
    esmacat_slave_vendor_id = ESMACAT_LOADCELL_INTERFACE_VENDOR_ID;
    std::cout <<"esmacat Loadcell Interface Object is created" << std::endl;
    for(int i=0;i<ESMACAT_LOADCELL_INTERFACE_MAX_ANALOG_INPUT_INDEX;i++){
        raw_adc_reading_by_channel[i] = 0;
    }
}

/** @brief Definition of inherited data exchange function for Esmacat slave
 *
 * Combines data exchange process of the base module and the loadcell interface slaves
 *
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param oloop No. of output bytes
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 * @param iloop No. of input bytes
 */
void esmacat_loadcell_interface::ecat_data_process(uint8_t* ec_slave_outputs,
                                                   int oloop,uint8_t* ec_slave_inputs,int iloop)
{
    ecat_data_process_base_module(ec_slave_outputs,ec_slave_inputs);
    ecat_data_loadcell_interface(ec_slave_outputs,ec_slave_inputs);
}

/** @brief Private function- Data exchange process that is specific to the Esmacat Loadcell Interface slave
 *
 * Skip the first 6 bytes of the input pointer [***hardware dependent - do not change***]
 *
 * Following that every pair of bytes is a 16 bit ADC reading for the corresponding channel
 *
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 **/
void esmacat_loadcell_interface::ecat_data_loadcell_interface(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs)
{
    for (int i=0;i<ESMACAT_LOADCELL_INTERFACE_MAX_ANALOG_INPUT_INDEX;i++)
    {
        raw_adc_reading_by_channel[i] = *(ec_slave_inputs+7+i*2);            // decode  system_parameter_value
        raw_adc_reading_by_channel[i] = (raw_adc_reading_by_channel[i] << 8) |  *(ec_slave_inputs+6+i*2);
    }
}

/** @brief Private function - Reads the ADC channels
 *
 * @param channel_index Index of the channel in the Esmacat Loadcell Interface slave
 * @param error Contains the error message to demonstrate success of data acquisition
 * (ERR_LOADCELL_IFC_CHANNEL_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Raw ADC reading for the desired channel on the Esmacat Analog Input slave
 */
int16_t esmacat_loadcell_interface::get_raw_analog_input(int channel_index, esmacat_err& error)
{
    int16_t raw_adc_reading = 0;
    if (channel_index < 0 || channel_index > ESMACAT_LOADCELL_INTERFACE_MAX_ANALOG_INPUT_INDEX ){
        error = ERR_LOADCELL_IFC_CHANNEL_INDEX_OUT_OF_RANGE;
    }
    else{
        raw_adc_reading = raw_adc_reading_by_channel[channel_index];
        error =  NO_ERR;
    }
    return raw_adc_reading;
}

/** @brief Reads analog input value in mV and provides error handling
 *
 * @param channel_index Index of the ADC channel to be read
 * @param error Contains the error message to demonstrate success of data acquisition
  * (ERR_LOADCELL_IFC_CHANNEL_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Voltage reading of input channel in mV
 */
float esmacat_loadcell_interface::get_analog_input_mV(int channel_index, esmacat_err& error)
{
    uint16_t raw_analog_input;
    esmacat_err result_of_get_raw_analog_input;
    float analog_input_mV;

    //int16_t int_v = (int16_t)(raw_adc_reading_by_channel[channel_index]);
    raw_analog_input = get_raw_analog_input(channel_index,result_of_get_raw_analog_input);
    if (result_of_get_raw_analog_input  == NO_ERR)
    {   //if no error, convert it to mV, and set error to NO_ERR
        analog_input_mV = 5000.0 * (float)(raw_adc_reading_by_channel[channel_index])/ float(32767); 
        error = NO_ERR;
    }
    else
    {   //if error then set value to 0 and set the error
        analog_input_mV  = 0;
        error = result_of_get_raw_analog_input;
    }
    return analog_input_mV;

}

/** @brief Reads analog input value in mV. No error handling
 * @param channel_index Index of the ADC channel to be read
 * @return Voltage reading (mV) of the input channel
 */
float esmacat_loadcell_interface::get_analog_input_mV(int channel_index)
{
    esmacat_err error = NO_ERR;
    return get_analog_input_mV(channel_index, error);
}

/** @brief Configures the type, gain and buffer status of
 * all channels of the slave
 *
 * Prepares packet for transmission to the slave
 * @param loadcell_interface_channel_config Holds the details of the configuration of the loadcell interface slave channels
 * @return Status of the transmission (NO_ERR returned if successful)
 */
esmacat_err esmacat_loadcell_interface::configure_slave_loadcell_interface_adc(esmacat_loadcell_interface_channel_config_T loadcell_interface_channel_config)
{
    uint16_t configuration_param = 0;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_0_1;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_2_3<< 1;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_4_5<< 2;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_6_7 << 3;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_8_9 << 4;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_10_11 << 5;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_12_13 << 6;
    configuration_param |= loadcell_interface_channel_config.single_ended_diff_ch_14_15 << 7;
    configuration_param |= loadcell_interface_channel_config.PGA_ch_0_7 << 8;
    configuration_param |= loadcell_interface_channel_config.PGA_ch_8_15 << 11;
    configuration_param |= loadcell_interface_channel_config.buff_en_ADC_ch_0_7 << 14;
    configuration_param |= loadcell_interface_channel_config.buff_en_ADC_ch_8_15 << 15;
    add_system_parameters_in_queue(ESMACAT_LOADCELL_INTERF_CONIG_ADDR,configuration_param);
    return NO_ERR;
}

/** @brief Reads the product id of the slave
 * @return Product ID of the slave
 */
uint32_t esmacat_loadcell_interface::get_esmacat_product_id()
{
    return esmacat_slave_product_id;
}

/** @brief Returns the vendor ID of the slave
 * @return Vendor ID of the slave
 */
uint32_t esmacat_loadcell_interface::get_esmacat_vendor_id()
{
    return esmacat_slave_vendor_id;
}
