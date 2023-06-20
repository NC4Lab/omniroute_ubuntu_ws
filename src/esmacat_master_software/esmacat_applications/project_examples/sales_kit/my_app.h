/** @file
 * @brief This file contains the declaration of classes associated with the user-defined
 * application for the Esmacat Sales Kit slave example project */

#ifndef MY_APP_H
#define MY_APP_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "application.h"
#include "loadcell_interf.h"
#include "analog_input.h"
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

    esmacat_loadcell_interface ecat_li; /**< create your Esmacat Loadcell Interface slave object */
    esmacat_loadcell_interface li; /**< create your Esmacat Loadcell Interface slave object */
    esmacat_motor_driver dcm; /**< create your Esmacat Motor Driver slave object */
    esmacat_analog_input_slave ai; /**< create your Esmacat Analog Input slave object */
    esmacat_motor_driver bldcm; /**< create your Esmacat Motor Driver slave object */
public:
    /** A constructor- sets initial values for class members */
    my_app()
    {

    }
};
#endif // MY_APP_H
