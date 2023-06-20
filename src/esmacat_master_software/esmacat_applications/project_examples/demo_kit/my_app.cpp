/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat Demo Kit example project */
#include "my_app.h"
#include <math.h>

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 *
 * In this example the communication chain is as follows:
 * Esmacat master - Motor Driver slave - Loadcell Interface slave
 */
void my_app::assign_slave_sequence()
{
    assign_esmacat_slave_index(&ecat_md0,0);
    assign_esmacat_slave_index(&ecat_li0,1);
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void my_app::init()
{


}

/**
 * @brief Setup configuration parameters for your Esmacat slave
 *
 * Functions beginning with 'configure_slave' must only be executed in this function
 */
void my_app::configure_slaves()
{
    // Functions starting with "configure_slave" work only in configure_slaves() function
    //INITIALIZATION
    encoder_counts_per_turn = 512; //Encoder has 512 counts per turn

    //total time for which application is executed
    end_time_miliseconds = 60000*2;

    //clear setpoints for position and velocity control
    target_for_position_control = 0;
    target_for_current_control = 0;

    //initialize values for the potentiometer
    pot_voltage = 0;
    pot_command_normalized = 0;

    //0=position control with potentiometer, 1 = velocity control with load cell
    control_mode = 0;

    //set to be initially "off", which is logic 1
    green_button = 1;
    //set to be initially "off", which is logic 1
    red_button = 1;

    //Clear the encoder
    ecat_md0.configure_slave_encoder_clear();
    //Set gains for position control loop (proportional, derivative, integral)
    ecat_md0.set_position_control_pid_gain(0.4*10, 0.0, 0.004*10);
    //Set motor driver interface max velocity profile slope
    ecat_md0.set_max_velocity_in_position_control_qc_p_ms(2000);

    //Declare load cell interface option objects
    esmacat_loadcell_interface_channel_config_T loadcell_config;
    esmacat_loadcell_interface_channel_config_T potentiometer;

    //Set load cell interface configuration
    loadcell_config.single_ended_diff_ch_8_9 = DIFFERENTIAL_INPUT;
    potentiometer.single_ended_diff_ch_8_9 = SINGLE_ENDED_INPUT;

    //Set load cell interface programmable gain amplifier
    loadcell_config.PGA_ch_8_15 = PGA64;
    ecat_li0.configure_slave_loadcell_interface_adc(loadcell_config);

    // Set load cell interface DIO to output to LED or read button presses
    IO_Direction dig_io[7] = { IO_OUTPUT,IO_INPUT,IO_OUTPUT,IO_OUTPUT,IO_INPUT,IO_INPUT,IO_INPUT};
    ecat_li0.configure_slave_dio_direction(dig_io);

}

/**
 * @brief Executes functions at the defined loop rate
 */
void my_app::loop(){

    //Change LED based on button selected control_mode variable
    if(control_mode == 0 ){
        ecat_li0.set_digital_output(0,0);//Red light
        ecat_li0.set_digital_output(3,1);//Green light
        ecat_li0.set_digital_output(2,0);//Blue light
    }

    else if (control_mode == 1 ){
        ecat_li0.set_digital_output(0,1);//Red light
        ecat_li0.set_digital_output(3,0);//Green light
        ecat_li0.set_digital_output(2,0);//Blue light
    }

    //Obtain state of the buttons
    green_button = ecat_li0.get_digital_input(4);
    red_button = ecat_li0.get_digital_input(5);

    //Check if green button was pressed to change to control_mode=0
    if(green_button == 0){
        control_mode = 0;
    }

    //Check if red buttton was pressed to change to control_mode=1
    if(red_button == 0){
        control_mode = 1;
    }

    //Print to terminal
    std::cout << elapsed_time_ms << "\t"
              << "Pos_target: " << target_for_position_control << "\t"
              << "Pos_encoder: " << ecat_md0.get_encoder_counter() << "\t"
              << "Pot_voltage: " << pot_voltage << "\t"
              << "DIO4_Green: " << green_button << "\t"
              << "DIO5_Red: " << red_button << "\t"
              << "Control_mode: " << control_mode << "\t"
              << "Load_cell_8: " << ecat_li0.get_analog_input_mV(8) << "\t"
              << "Load_cell_9: " << ecat_li0.get_analog_input_mV(9) << "\t"
              << std::endl;

    //Enable ESCON to operate the BLDC motor
    ecat_md0.enable_escon(TRUE);

    //RUN PROGRAM BASED ON control_mode = 0. POTENTIOMETER POSITION CONTROL
    if (control_mode == 0)
    {

        //Read potentiometer voltage
        pot_voltage = ecat_md0.get_analog_input_from_external_source_mV(0);

        //Run position control from potentiometer input (normalized)
        pot_command_normalized = (pot_voltage-5.0)/ (5.0/2048.0);
        target_for_position_control = pot_command_normalized;
        ecat_md0.set_desired_position(target_for_position_control);

        //Turn off LED 50ms before the code ends
        if(elapsed_time_ms > end_time_miliseconds-50)
        {

            ecat_li0.set_digital_output(0,0);
            ecat_li0.set_digital_output(2,0);
            ecat_li0.set_digital_output(3,0);

        }

        //Turn off the system
        if ( elapsed_time_ms > end_time_miliseconds)
        {
            ecat_md0.enable_escon(FALSE);
            stop();
        }
    }

    //RUN PROGRAM BASED ON control_mode = 1. LOAD CELL VELOCITY CONTROL
    else if (control_mode == 1)
    {

        //Test velocity control from load cell input
        target_for_current_control = ecat_li0.get_analog_input_mV(8); //Observing the second voltage input
        //Negative sign to change direction of motor to be intuitive. Addition is for offset. Division to normalize
        ecat_md0.set_escon_current_setpoint(-(target_for_current_control-41.20)/50);

        //Turn off LED 50ms before the code ends
        if(elapsed_time_ms > end_time_miliseconds-50)
        {

            ecat_li0.set_digital_output(0,0);
            ecat_li0.set_digital_output(2,0);
            ecat_li0.set_digital_output(3,0);

        }

        //Turn off the system
        if ( elapsed_time_ms > end_time_miliseconds)
        {
            ecat_md0.enable_escon(FALSE);
            stop();
        }
    }
}
