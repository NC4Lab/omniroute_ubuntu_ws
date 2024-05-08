/** @file
 * @brief This file contains the declaration of the class associated with the user-defined
 * application for the EtherCAT Arduino Shield by Esmacat slave example project */

#ifndef ROS_ESMACAT_APP_H
#define ROS_ESMACAT_APP_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "application.h"
using std::cout;
//Include the header file for the Esmacat slave you plan to use for e.g. EASE
#include "ethercat_arduino_shield_by_esmacat.h"
// Include the ROS header files for the Esmacat slave you plan to use e.g. EASE 
#include "ros_ethercat_arduino_shield_by_esmacat.h"

#include "motordriver.h"
#include "ros_ethercat_motor_driver_250.h"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"
#include "plog/Appenders/RollingFileAppender.h"
#include "plog/Appenders/ColorConsoleAppender.h"
#include "plog/Formatters/CsvFormatter.h"
#include "plog/Formatters/TxtFormatter.h"

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/**
 * @brief Description of your custom application class
 *
 * Your custom application class ros_esmacat_app inherits the class 'esmacat_application' & 'ros_ethercat_arduino_shield_by_esmacat'
 * Write functions to override the parent functions of 'esmacat_application' & 'ros_ethercat_arduino_shield_by_esmacat'
 * Declare an object of your slave class (e.g. ecat_as)
 * Declare an object of your slave class to interact with ROS Nodes (e.g. maze_ard0_ease)
 * Declare any other variables you might want to add
 * Define the constructor for initialization
 */
class ros_esmacat_app : public esmacat_application
{
public:
    /** A constructor- sets initial values for class members */
    ros_esmacat_app(): sync_ease("sync_ease"), sync_ease_ros_message(),
    maze_ard0_ease("maze_ard0_ease"), maze_ard0_ease_ros_message(),
    feeder_ease("feeder_ease"), feeder_ease_ros_message() {}

    void assign_slave_sequence(); /** identify sequence of slaves and their types */
    void configure_slaves(); /** configure all slaves in communication chain */
    void init(); /** code to be executed in the first iteration of the loop */
    void loop(); /** control loop */

private:

    // Declare slave objects for syncing arduino
    esmacat_ethercat_arduino_shield_by_esmacat sync_ease_ecat_as; // Esmacat slave object
    ros_ethercat_arduino_shield_by_esmacat sync_ease; // ROS object for Esmacat slave
    ros_ethercat_arduino_shield_by_esmacat :: write sync_ease_ros_message; // ROS message object for Esmacat slave

    // Declare slave objects for maze wall conroller arduino
    esmacat_ethercat_arduino_shield_by_esmacat maze_ard0_ease_ecat_as; 
    ros_ethercat_arduino_shield_by_esmacat maze_ard0_ease;
    ros_ethercat_arduino_shield_by_esmacat :: write maze_ard0_ease_ros_message;

    // Declare slave objects for feeder servos
    esmacat_ethercat_arduino_shield_by_esmacat feeder_ease_ecat_as; 
    ros_ethercat_arduino_shield_by_esmacat feeder_ease;
    ros_ethercat_arduino_shield_by_esmacat :: write feeder_ease_ros_message;
};

#endif // ROS_ESMACAT_APP_H
