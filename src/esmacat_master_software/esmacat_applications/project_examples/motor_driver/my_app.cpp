/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat Motor Driver slave example project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "my_app.h"
#include <math.h>

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void my_app::assign_slave_sequence()
{
    assign_esmacat_slave_index(&ecat_md,0);
}

/**
 * @brief Setup configuration paramters for your Esmacat slave
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 */
void my_app::configure_slaves()
{
    // Functions starting with "configure_slave" work only in configure_slaves() function
    //clear encoder
    ecat_md.configure_slave_encoder_clear();
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
    //enable ESCON motor drive
    ecat_md.enable_escon(1);

    //Normalized setpoint (in range -1 to +1)
    ecat_md.set_escon_current_setpoint(0.3*sin(2*3.1415*elapsed_time_ms/1000.0) );

    cout << elapsed_time_ms << "\t"
         << ecat_md.get_encoder_counter() << "\t"
         << ecat_md.get_analog_input_from_external_source_mV(0) << endl;

    //disable ESCON and terminate after application has run for 30s
    if ( elapsed_time_ms > 30000)
    {
        ecat_md.enable_escon(0);
        stop();
    }
}
