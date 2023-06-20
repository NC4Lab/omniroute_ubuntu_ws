/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat Sales Kit slave example project */
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
 *
 * The chain is identified as follows:
 * Esmacat master - Motor Driver - Loadcell Interface - Analog Input - Motor Driver
 */
void my_app::assign_slave_sequence()
{
    assign_esmacat_slave_index(&dcm,0);
    assign_esmacat_slave_index(&li,1);
    assign_esmacat_slave_index(&ai,2);
    assign_esmacat_slave_index(&bldcm,3);
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void my_app::init()
{


}

/**
 * @brief Setup configuration parameters for all the Esmacat slaves
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 *
 */
void my_app::configure_slaves()
{
    // Functions starting with "configure_slave" work only in configure_slaves() function
    // loadcell interface slave setup
    esmacat_loadcell_interface_channel_config_T loadcell_interface_config;
    loadcell_interface_config.single_ended_diff_ch_0_1 = DIFFERENTIAL_INPUT;
    loadcell_interface_config.PGA_ch_0_7 = PGA64;
    li.configure_slave_loadcell_interface_adc(loadcell_interface_config);
    // digital i/o setup
    IO_Direction dio_direction[7]={ IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_INPUT,IO_INPUT,IO_OUTPUT,};
    li.configure_slave_dio_direction(dio_direction);

    // analog input slave setup
    esmacat_analog_input_channel_config_T adc_channel_config;
    adc_channel_config.ch0to3_config = SINGLE_ENDED_0V_POS6V;
    adc_channel_config.ch4to7_config = SINGLE_ENDED_0V_POS6V;
    adc_channel_config.ch8to11_config = SINGLE_ENDED_0V_POS6V;
    adc_channel_config.ch12to15_config = SINGLE_ENDED_0V_POS6V;
    ai.configure_slave_analog_input(adc_channel_config);

    // dc motor - direct control (Motor Driver slave setup)
    dcm.configure_slave_encoder_clear();

    // bldc motor - position control (Motor Driver slave setup)
    bldcm.configure_slave_encoder_clear();
    bldcm.set_position_control_pid_gain(0.3, 0, 0.001);
    bldcm.set_max_velocity_in_position_control_qc_p_ms(2000);
}

/**
 * @brief Executes functions at the defined loop rate
 *
 */
void my_app::loop()
{
    cout << elapsed_time_ms << "\t";
    // digital i/o
    // if red button is pressed
    if (li.get_digital_input(4) == 0)
    {
        //turn on red LED
        li.set_digital_output(3,TRUE);
        //turn off green LED
        li.set_digital_output(0,FALSE);
    }
    // if green button is pressed
    if (li.get_digital_input(5) == 0)
    {
        //turn on green LED
        li.set_digital_output(0,TRUE);
        //turn off red LED
        li.set_digital_output(3,FALSE);
    }

    // output loadcell input
    float li_v = li.get_analog_input_mV(0);
    cout << li_v;

    // bldc motor - current control
    // desired current proportional to loadcell input
    dcm.enable_escon(1);
    dcm.set_escon_current_setpoint(-li_v/800.0);
    cout << " " << dcm.get_encoder_counter();

    //output analog input
    float ai_v = ai.get_analog_input_mV(0);
    cout << " " << ai_v ;

    // motor position control
    // desired position is proportional to the analog input
    bldcm.enable_escon(1);
    bldcm.set_desired_position(-(int)ai_v);

    //output encoder reading
    cout << " " << bldcm.get_encoder_counter();
    cout << endl;

    // stop if application has run for 500s
    if ( elapsed_time_ms > 500000)
    {
        stop();
    }
}
