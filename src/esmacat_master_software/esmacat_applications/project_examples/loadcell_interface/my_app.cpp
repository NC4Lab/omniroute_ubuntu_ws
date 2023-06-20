/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat Loadcell Interface slave example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "my_app.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void my_app::assign_slave_sequence()
{
    assign_esmacat_slave_index(&ecat_li,0);
}

/**
 * @brief Setup configuration parameters for your Esmacat LoadCell Interface slave
 *
 * The slave is an amplified analog input device. The board accepts
 * 8 differential inputs (or 16 single-ended inputs). The gain for each channel
 * can be programmed
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 */
void my_app::configure_slaves()
{

    // Functions starting with "configure_slave" work only in configure_slaves() function
    esmacat_loadcell_interface_channel_config_T loadcell_config;
    //configure the type of inputs that are connected to the slave
    //in our example, the loadcell provides a differential input connected to ch0-1
    loadcell_config.single_ended_diff_ch_0_1 = DIFFERENTIAL_INPUT;
    loadcell_config.single_ended_diff_ch_2_3 = DIFFERENTIAL_INPUT;
    loadcell_config.single_ended_diff_ch_4_5 = DIFFERENTIAL_INPUT;
    loadcell_config.single_ended_diff_ch_6_7 = DIFFERENTIAL_INPUT;
    loadcell_config.single_ended_diff_ch_8_9 = SINGLE_ENDED_INPUT;
    loadcell_config.single_ended_diff_ch_10_11 = SINGLE_ENDED_INPUT;
    loadcell_config.single_ended_diff_ch_12_13 = SINGLE_ENDED_INPUT;
    loadcell_config.single_ended_diff_ch_14_15 = SINGLE_ENDED_INPUT;

    //gain set to 64 for the channels to which the loadcell is connected
    loadcell_config.PGA_ch_0_7 = PGA64;
    //gain set to 1
    loadcell_config.PGA_ch_8_15 = PGA1;

    //disable voltage buffering of the loadcell input
    loadcell_config.buff_en_ADC_ch_0_7 = 0;
    loadcell_config.buff_en_ADC_ch_8_15 = 0;

    ecat_li.configure_slave_loadcell_interface_adc(loadcell_config);
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void my_app::init()
{


}

/**
 * @brief Executes functions at the defined loop rate
 */
void my_app::loop()
{
    //outputs the elapsed time of the application
    cout << elapsed_time_ms << "\t";

    //outputs readings on all 16 channels of the loadcell interface board
    for(int i=0;i<16;i++){
        cout << i << ": " << ecat_li.get_analog_input_mV(i) << "\t" ;
    }

    cout << endl;

    //terminate if the application has run for 300s
    if (loop_cnt > 1000000)
    {
        printf("\nWARNING: Real time loop rate was exceeded %d times\n", get_app_error_counter());
        stop();
    }
}
