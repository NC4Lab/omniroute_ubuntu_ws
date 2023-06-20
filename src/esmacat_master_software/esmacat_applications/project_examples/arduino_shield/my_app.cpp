/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */
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
void my_app::assign_slave_sequence(){
    // tell the master what type of slave is at which point in the chain
    assign_esmacat_slave_index(&ecat_as,0);
}

/**
 * @brief Configure your Esmacat slave.
 * Link Esmacat slave object with the actual Esmacat slave in the EtherCAT communication chain.
 * Functions beginning with 'configure slave' must only be executed in this function
 */
void my_app::configure_slaves(){


    // add initialization code here
    // Functions starting with "configure_slave" work only in configure_slave() function
}

/** @brief Initialization that needs to happen on the first iteration of the loop
 */
void my_app::init()
{
    curr_counter =0;
    cout << "\n" << loop_cnt << "\t Initialization complete";

}


/**
 * @brief Executes functions at the defined loop rate
 */
void my_app::loop(){
    // add functions below that are to be executed at the loop rate

    //read register 0 (testing write_reg_value by arduino)
    cout << "\nReading at elapsed time = " << elapsed_time_ms << "\t" << ecat_as.get_input_variable_0_IN_GEN_INT0();

    //write all registers (which will be read by the serial monitor in the arduino)
    cout << "\nWriting at loop_cnt = " << loop_cnt << "\t"<< curr_counter << "\t";
    ecat_as.set_output_variable_0_OUT_GEN_INT0(curr_counter++);
    cout << "\t" << curr_counter << "\t";
    ecat_as.set_output_variable_1_OUT_GEN_INT1(curr_counter++);
    cout << "\t" << curr_counter << "\t";
    ecat_as.set_output_variable_2_OUT_GEN_INT2(curr_counter++);
    cout << "\t" << curr_counter << "\t";
    ecat_as.set_output_variable_3_OUT_GEN_INT3(curr_counter++);
    cout << "\t" << curr_counter << "\t";
    ecat_as.set_output_variable_4_OUT_GEN_INT4(curr_counter++);
    cout << "\t" << curr_counter << "\t";
    ecat_as.set_output_variable_5_OUT_GEN_INT5(curr_counter++);
    cout << "\t" << curr_counter << "\t";
    ecat_as.set_output_variable_6_OUT_GEN_INT6(curr_counter++);
    cout << "\t" << curr_counter << "\t";
    ecat_as.set_output_variable_7_OUT_GEN_INT7(curr_counter++);

    //Runs the loop 1000 times at defined loop rate (Refer to ESMACAT_TIME_PERIOD_US
    // in master.h)
    if (loop_cnt > 1000)
    {
        printf("\nWARNING: Real time loop rate was exceeded %d times out of a total of %ld\n", get_app_error_counter(), loop_cnt);
        stop();
    }
}
