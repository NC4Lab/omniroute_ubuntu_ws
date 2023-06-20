/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat Digital I/O example project */
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
 * @brief Setup the configuration parameters for your Loadcell Interface Esmacat slave
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 *
 */
void my_app::configure_slaves()
{
    // Functions starting with "configure_slave" work only in configure_slaves() function
    bool led_on_off_status = 0;
    //configure the direction of the digital i/o
    IO_Direction dio_config[7] = { IO_INPUT,IO_INPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT};
    ecat_li.configure_slave_dio_direction(dio_config);
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
    //read the input
    if( ecat_li.get_digital_input(0) == 0 )
    {
        led_on_off_status = 0;
    }
    if( ecat_li.get_digital_input(1) == 0 )
    {
        led_on_off_status = 1;
    }
    //set the output based on the input values
    if (led_on_off_status)
    {
        ecat_li.set_digital_output(2,1);
        ecat_li.set_digital_output(3,0);
    }
    else
    {
        ecat_li.set_digital_output(2,0);
        ecat_li.set_digital_output(3,1);
    }

    cout << elapsed_time_ms << "\t"
         << ecat_li.get_digital_input(0) << " " << ecat_li.get_digital_input(1)  << " " <<led_on_off_status  << endl;

    //stop after application has run for 300s
    if ( elapsed_time_ms > 300000)
    {
        stop();
    }
}
