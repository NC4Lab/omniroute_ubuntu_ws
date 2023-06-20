/** @file
 * @brief Contains all the declarations required for the Esmacat Analog Input slave
 * */

#ifndef ESMACAT_ANALOG_INPUT_H
#define ESMACAT_ANALOG_INPUT_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique vendor ID assigned by the EtherCAT Technology Group to Harmonic Bionics Inc.*/
#define ESMACAT_ANALOG_INPUT_VENDOR_ID          0x0062696f

/** @brief Unique product code for Esmacat Analog Input slave required for EtherCAT */
#define ESMACAT_ANALOG_INPUT_PRODUCT_ID         0x00040101

/** @brief Total channels on the Esmacat Analog Input slave */
#define ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX 16

/** @brief ADC Configuration register defined in firmware. [***firmware dependent - do not change***]*/
#define ESMACAT_ANALOG_INPUT_CONFIG_REGISTER        0x0701

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "base_module.h"
#include "slave.h"
#include "error_list.h"
#include <iostream>
#include <math.h>
/*****************************************************************************************
 * ENUMERATIONS
 ****************************************************************************************/
/** @brief Range-Select bits of the MAX1300 ADC
 *
 * bit[3] : 1 if differential, 0 if single-ended
 *
 * bit[2-0] represent the range per the datasheet (Vref = 4096 mV)
 */
enum AdcChannelTypeandRange
{
    SINGLE_ENDED_NEG3V_POS3V = 0b0001, /**< Single-ended -3V to +3V*/
    SINGLE_ENDED_NEG6V_0V = 0b0010, /**< Single-ended -6V to 0V*/
    SINGLE_ENDED_0V_POS6V = 0b0011, /**< Single-ended 0V to 6V*/
    SINGLE_ENDED_NEG6V_POS6V = 0b0100, /**< Single-ended -6V to +6V*/
    SINGLE_ENDED_NEG12V_0V = 0b0101, /**< Single-ended -12V to 0V*/
    SINGLE_ENDED_0V_POS12V = 0b0110, /**< Single-ended -0V to 12V*/
    SINGLE_ENDED_NEG12V_POS12V = 0b0111, /**< Single-ended -12V to +12V*/
    DIFF_NEG6V_POS6V = 0b1001, /**< Differential -6V to +6V*/
    DIFF_NEG12V_POS12V = 0b1100, /**< Differential -12V to +12V*/
    DIFF_NEG24V_POS24V = 0b1111 /**< Differential -24V to +24V*/
};

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/
/** @brief Structure that holds the Analog Input slave channel configuration
 *
 * Describes the type and range of the 16 channels of the ADC
*/
struct esmacat_analog_input_channel_config_T{
    AdcChannelTypeandRange ch0to3_config; /**< Type and range  of channels 0 - 3 of the ADC */
    AdcChannelTypeandRange ch4to7_config; /**< Type and range  of channels 4 - 7 of the ADC */
    AdcChannelTypeandRange ch8to11_config; /**< Type and range  of channels 8 - 11 of the ADC */
    AdcChannelTypeandRange ch12to15_config; /**< Type and range  of channels 12 - 15 of the ADC */

    esmacat_analog_input_channel_config_T() //constructor containing initialization for the struct
    {
        ch0to3_config = SINGLE_ENDED_0V_POS6V;
        ch4to7_config = SINGLE_ENDED_0V_POS6V;
        ch8to11_config = SINGLE_ENDED_0V_POS6V;
        ch12to15_config = SINGLE_ENDED_0V_POS6V;
    }/**< All channels initialized to single-ended 0 to 6V range */
};

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/** @brief Class that holds the Analog Input slave data and
 * functions to access and communicate with it
*/
class esmacat_analog_input_slave: public esmacat_base_module
{

private:
    /** Holds ADC readings for all 16 or ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX channels */
    uint16_t raw_adc_reading_by_channel[ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX];
    /** Offset to be applied to the reading from the ADC to account for negative values */
    float ai_offset[ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX];
    /** Full scale range of the ADC readings */
    float ai_full_scale_range[ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX];
    /** Defines whether the input voltage is single-ended or differential for all 16
     * or ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX channels */
    ADC_input_single_ended_diff ai_single_ended_or_diff[ESMACAT_ANALOG_INPUT_MAX_CHANNEL_INDEX];
    void ecat_data_process_analog_input(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);
    uint16_t get_raw_analog_input(int index_of_analog_input, esmacat_err& error);
    esmacat_err configure_slave_analog_input( AdcChannelTypeandRange ai_config_ch0to3,
                                                         AdcChannelTypeandRange ai_config_ch4to7,
                                                         AdcChannelTypeandRange ai_config_ch8to11,
                                                         AdcChannelTypeandRange ai_config_ch12to15);

public:
    /** Contains a product code of the Esmacat slave */
    uint32_t esmacat_slave_product_id = ESMACAT_ANALOG_INPUT_PRODUCT_ID;
     /** Contains a unique Vendor ID assigned by the EtherCAT Technology Group*/
    uint32_t esmacat_slave_vendor_id = ESMACAT_ANALOG_INPUT_VENDOR_ID;

    esmacat_analog_input_slave();
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
    float get_analog_input_mV(int index_of_analog_input, esmacat_err& error);
    float get_analog_input_mV(int index_of_analog_input);
    esmacat_err configure_slave_analog_input(esmacat_analog_input_channel_config_T channel_config);
    uint32_t get_esmacat_product_id();
    uint32_t get_esmacat_vendor_id();
};
#endif // ESMACAT_ANALOG_INPUT_H
