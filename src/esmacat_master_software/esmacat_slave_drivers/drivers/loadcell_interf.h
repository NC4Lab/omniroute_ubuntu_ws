/** @file
 * @brief Contains all the declarations for the Esmacat Loadcell Interface slave
 */

#ifndef ESMACAT_LOADCELL_INTERF_H
#define ESMACAT_LOADCELL_INTERF_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique vendor ID assigned by the EtherCAT Technology Group to Harmonic Bionics Inc.*/
#define ESMACAT_LOADCELL_INTERFACE_VENDOR_ID    0x0062696f

/** @brief Unique product code for Esmacat Loadcell Interface slave required for EtherCAT */
#define ESMACAT_LOADCELL_INTERFACE_PRODUCT_ID   0x00030101

/** @brief Total channels on the Esmacat Loadcell Interface slave ADC*/
#define ESMACAT_LOADCELL_INTERFACE_MAX_ANALOG_INPUT_INDEX 16

/** @brief Loadcell configuration register defined in firmware. [***firmware dependent - do not change***]*/
#define ESMACAT_LOADCELL_INTERF_CONIG_ADDR 0x0601

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "base_module.h"
#include "error_list.h"
#include <iostream>
#include <math.h>
/*****************************************************************************************
 * ENUMERATIONS
 ****************************************************************************************/
/** Programmable gain values for the ADS1256. Refer to the datasheet for more details */
enum ProgrammableGain
{
    PGA1 = 0, /**< Gain of 1 */
    PGA2 = 1, /**< Gain of 2 */
    PGA4 = 2, /**< Gain of 4 */
    PGA8 = 3, /**< Gain of 8 */
    PGA16 = 4, /**< Gain of 16 */
    PGA32 = 5, /**< Gain of 32 */
    PGA64 = 6  /**< Gain of 64 */
};

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/
/** @brief Struct that holds the Esmacat Loadcell Interface ADC channel configuration
 *
 * Describes the type of all the channels (single-ended or differential), sets the gain
 * for each ADC on the slave, and enables/disables the input buffer for each ADC on the
 * slave
 */
struct esmacat_loadcell_interface_channel_config_T{
    /** Enables input buffer for channels 0 to 7; Enable = 1 */
    bool buff_en_ADC_ch_0_7;
    /** Enables input buffer for channels 8 to 15; Enable = 1 */
    bool buff_en_ADC_ch_8_15;
    /** Sets the gain for channels 0 to 7 */
    ProgrammableGain PGA_ch_0_7;
    /** Sets the gain for channels 8 to 15 */
    ProgrammableGain PGA_ch_8_15;
     /** Set type of input signal on channels 0 and 1 */
    ADC_input_single_ended_diff single_ended_diff_ch_0_1;
     /** Set type of input signal on channels 2 and 3 */
    ADC_input_single_ended_diff single_ended_diff_ch_2_3;
    /** Set type of input signal on channels 4 and 5 */
    ADC_input_single_ended_diff single_ended_diff_ch_4_5;
     /** Set type of input signal on channels 6 and 7 */
    ADC_input_single_ended_diff single_ended_diff_ch_6_7;
    /** Set type of input signal on channels 8 and 9 */
    ADC_input_single_ended_diff single_ended_diff_ch_8_9;
    /** Set type of input signal on channels 10 and 11 */
    ADC_input_single_ended_diff single_ended_diff_ch_10_11;
    /** Set type of input signal on channels 12 and 13 */
    ADC_input_single_ended_diff single_ended_diff_ch_12_13;
     /** Set type of input signal on channels 14 and 15 */
    ADC_input_single_ended_diff single_ended_diff_ch_14_15;
    /** All channels are set to receive single-ended input signals
     * with the input buffer disabled, and a gain of 1
     */
    esmacat_loadcell_interface_channel_config_T(){ //constructor for initializing the struct
        buff_en_ADC_ch_0_7 = false;
        buff_en_ADC_ch_8_15 = false;
        PGA_ch_0_7 = PGA1;
        PGA_ch_8_15 = PGA1;
        single_ended_diff_ch_0_1 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_2_3 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_4_5 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_6_7 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_8_9 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_10_11 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_12_13 = SINGLE_ENDED_INPUT;
        single_ended_diff_ch_14_15 = SINGLE_ENDED_INPUT;
    }
 };

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/** @brief Class that holds the Esmacat Loadcell Interface slave data and functions to
 * access and communicate with it
*/
class esmacat_loadcell_interface : public esmacat_base_module
{
    /** Contains the raw ADC readings for 16 channels. The readings from the ADS1526 are
      * 24 bits long. The least significant 8 bits are dropped*/
    int16_t raw_adc_reading_by_channel[ESMACAT_LOADCELL_INTERFACE_MAX_ANALOG_INPUT_INDEX];
    void ecat_data_loadcell_interface(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);
    int16_t get_raw_analog_input(int index_of_analog_input, esmacat_err& error);
public:
    /** Contains a product code of the Esmacat slave */
    uint32_t esmacat_slave_product_id = ESMACAT_LOADCELL_INTERFACE_PRODUCT_ID;
    /** Contains the vendor ID issued by the EtherCAT Technology group */
    uint32_t esmacat_slave_vendor_id = ESMACAT_LOADCELL_INTERFACE_VENDOR_ID;

    esmacat_loadcell_interface();
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
    float get_analog_input_mV(int channel_index, esmacat_err& error);
    float get_analog_input_mV(int index_of_analog_input);
    esmacat_err configure_slave_loadcell_interface_adc(esmacat_loadcell_interface_channel_config_T li_option);
    uint32_t get_esmacat_product_id();
    uint32_t get_esmacat_vendor_id();
};

#endif // ESMACAT_LOADCELL_INTERF_H
