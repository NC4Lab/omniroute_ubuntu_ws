/** @file
 * @brief This file contains the enumerations of all the error codes used by the Esmacat slaves */

#ifndef ESMACAT_ERROR_LIST_H
#define ESMACAT_ERROR_LIST_H

/*****************************************************************************************
 * ENUMERATIONS
 ****************************************************************************************/
enum esmacat_err{
    NO_ERR = 0,
    //ESMACAT slave application error
    ERR_ESMACAT_SLAVE_DOES_NOT_MATCH = 1001, //not used
    //Motor driver error list
    ERR_MOTOR_DRIVER_NO_DIRECTION_CHANGE_AT_NON_DIRECT_CONTROL = 2001,
    ERR_MOTOR_DRIVER_NO_PWM_CHANGE_AT_NON_DIRECT_CONTROL = 2002,
    ERR_MOTOR_DRIVER_MAX_PWM_REACHED = 2003,
    ERR_MOTOR_DRIVER_MIN_PWM_REACHED = 2004,
    ERR_MOTOR_DRIVER_SETPOINT_OUT_OF_RANGE = 2005,
    ERR_MOTOR_DRIVER_EXT_ANALOG_INPUT_INDEX_OUT_OF_RANGE = 2006,
    ERR_MOTOR_DRIVER_ESCON_ANALOG_INPUT_INDEX_OUT_OF_RANGE = 2007,
    ERR_MOTOR_DRIVER_POS_CONTROL_MODE_IS_DISABLED = 2008,
    ERR_MOTOR_DRIVER_TORQUE_CONTROL_MODE_IS_DISABLED = 2009,
    ERR_MOTOR_DRIVER_INSUFFICIENT_PARAMETER= 2010,
    ERR_MOTOR_DRIVER_PARAMETER_NOT_READY= 2011,
    //Loadcell interface error list
    ERR_LOADCELL_IFC_CHANNEL_INDEX_OUT_OF_RANGE = 3001,
    //Anlaog input error list
    ERR_ANALOG_INPUT_CHANNEL_INDEX_OUT_OF_RANGE = 4001,
    ERR_ANALOG_INPUT_INVALID_CONFIG = 4002,
    ERR_ANALOG_INPUT_NOT_SINGLE_ENDED = 4003,
    // Base module error list
    ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE = 5001,
    ERR_BASE_MODULE_DIO_DIRECTION_NOT_OUTPUT = 5002,
    ERR_BASE_MODULE_DIO_DIRECTION_NOT_INPUT = 5003,
    //Generic Slave error list
    ERR_SLAVE_QUEUE_FULL = 6001
};


#endif // ESMACAT_ERROR_LIST_H
