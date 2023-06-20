/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat Analog Input slave example project */
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
    assign_esmacat_slave_index(&ecat_ai,0);
}

/**
 * @brief Set up configuration parameters for your Esmacat Analog Input slave
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 *
 */
void my_app::configure_slaves()
{

    //set up the structure to provide inputs for configuration
    // Functions starting with "configure_slave" work only in configure_slaves() function
    /** The current settings are as follows:
    * Channels 0 - 7: Program to accept single-ended inputs in range 0 to 12V
    * Channels 8 - 11: Program to accept differential inputs in range -12V to 12V
    * Channels 12 - 15: Program to accept differential inputs in range -24V to 24V
    * Details on the configuration are located in esmacat_analog_input.cpp
    * You may modify this as necessary */
    esmacat_analog_input_channel_config_T adc_channel_config;
    adc_channel_config.ch0to3_config = SINGLE_ENDED_0V_POS12V;
    adc_channel_config.ch4to7_config = SINGLE_ENDED_0V_POS12V;
    adc_channel_config.ch8to11_config = DIFF_NEG12V_POS12V;
    adc_channel_config.ch12to15_config = DIFF_NEG24V_POS24V;
    ecat_ai.configure_slave_analog_input(adc_channel_config);
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void my_app::init()
{


}

/**
 * @brief Runs the application for 3s and outputs 16 readings of the potentiometer input in mV
 */
void my_app::loop()
{
    // outputs the elapsed time of the application at the start
    cout << elapsed_time_ms << "\t";

    //each execution of the loop outputs the current potentiometer input reading in mV
    for (int i=0;i<16;i++){
        cout << i << ": " << ecat_ai.get_analog_input_mV(i) << "\t";
    }
    cout << endl;

    //allows application to run for a total of 3s
    if ( elapsed_time_ms > 3000)
    {
        stop();
    }
}
