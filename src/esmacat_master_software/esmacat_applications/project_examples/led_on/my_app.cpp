/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat example project to turn on an LED*/
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
 *
 */
void my_app::loop()
{
    // LED blinks at a 1 kHz rate; on for 0.5s and off for 0.5s
    if ( fmod(elapsed_time_ms,1000) < 500 )
    {
        ecat_li.set_usr_led(1);
    }
    else
    {
        ecat_li.set_usr_led(0);
    }

    // stop the application after 5s
    if ( elapsed_time_ms > 5000)
    {
        stop();
    }
}
