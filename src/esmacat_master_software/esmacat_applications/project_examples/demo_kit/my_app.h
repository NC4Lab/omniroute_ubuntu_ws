/** @file
 * @brief This file contains the declaration of the classes associated with the user-defined
 * application for the Esmacat Demo Kit example project */
#ifndef MY_APP_H
#define MY_APP_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "application.h"

#include "loadcell_interf.h"
#include "motordriver.h"
using namespace std;

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/**
 * @brief Description of your custom application class
 *
 * Your custom application class my_app inherits the class 'esmacat_application'
 * Write functions to override the parent functions of 'esmacat_application'
 * Declare an object of your slave class (e.g. ecat_ai)
 * Declare any other variables you might want to add
 * Define the constructor for initialization
 */
class my_app : public esmacat_application
{
private:
    void assign_slave_sequence(); /**< identify sequence of slaves and their types*/
    void configure_slaves(); /**< configure all slaves in communication chain*/
    void init(); /** code to be executed in the first iteration of the loop */
    void loop(); /**< control loop*/

    esmacat_loadcell_interface ecat_li0; /**< create your Esmacat Loadcell interface slave object */
    esmacat_motor_driver ecat_md0; /**< create your Esmacat Motor Driver slave object */

    double encoder_counts_per_turn;
    double end_time_miliseconds;
    double target_for_position_control;
    double target_for_current_control;
    double pot_voltage;
    double pot_command_normalized;
    int    control_mode;
    bool   green_button;
    bool   red_button;
public:
    /** A constructor- sets initial values for class members */
    my_app()
    {

    }
};
#endif // MY_APP_H
