/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for position control of a motor using Esmacat slaves */
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
 * @brief Setup configuration parameters for your Esmacat slave
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 */
void my_app::configure_slaves()
{
    // Functions starting with "configure_slave" work only in configure_slaves() function
    //clear encoder
    ecat_md.configure_slave_encoder_clear();
    //set gains for the proportional, derivative and integral gains for the control loop
    ecat_md.set_position_control_pid_gain(0.3*5,0.0, 0.01*5);
    //Set motor driver interface max velocity profile slope
    ecat_md.set_max_velocity_in_position_control_qc_p_ms(2000);
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
    //enables the ESCON motor drive
    ecat_md.enable_escon(1);

    //set position setpoint based on the encoder datasheet
    int pulses_per_revolution = 1024;
    double target = 0;

    //this target is set every 1 ms
    //motor turns between -90 degrees and +90 degrees
    //since counts per turn = 4 * pulses per revolution
    if(fmod(elapsed_time_ms, 2000) < 1000)
        target = pulses_per_revolution;
    else
        target = -pulses_per_revolution;
    //set desired target
    ecat_md.set_desired_position(target);

    cout << elapsed_time_ms << "\t"
         << target << "\t"
         << ecat_md.get_encoder_counter() << "\t"
         << endl;

    // terminate after application has run for 60s
    if ( elapsed_time_ms > 60000)
    {
        ecat_md.enable_escon(0);
        stop();
    }
}
