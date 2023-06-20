/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat motor control example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "my_app.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 *
 * In this example, the communication chain is as follows:
 * Esmacat master - Analog Input slave - Motor Driver slave
 */
void my_app::assign_slave_sequence()
{
    assign_esmacat_slave_index(&ecat_ai,0);
    assign_esmacat_slave_index(&ecat_md,1);
}

/**
 * @brief Setup configuration parameters for your Esmacat slave
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 */
void my_app::configure_slaves()
{

    // Functions starting with "configure_slave" work only in configure_slaves() function
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
    float v = ecat_ai.get_analog_input_mV(0);
    //enable the ESCON motor drive
    ecat_md.enable_escon(1);
    //set normalized setpoint (-1 to +1) of current that is proportional to the potentiometer reading (which is between 0 to 5000)
    ecat_md.set_escon_current_setpoint( (v-2500.0)/2500.0 );

    //output the encoder and potentiometer readings
    cout << elapsed_time_ms << "\t"
         << ecat_md.get_encoder_counter() << "\t"
         << v << "\t"
         << endl;

    //disable ESCON and terminate after application has run for 60s
    if ( elapsed_time_ms > 60000)
    {
        ecat_md.enable_escon(0);
        stop();
    }
}
