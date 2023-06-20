/** @file
 * @brief This file declares the parent Ethercat slave class
 */

#ifndef ESMACAT_SLAVE_H
#define ESMACAT_SLAVE_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Maximum number of system parameters that can be assigned to each EtherCAT slave */
#define MAX_BUFFER_OF_SYSTEM_PARAMETER  30

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include <queue>
#include <vector>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "error_list.h"
/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/** @brief Generic parent Esmacat slave class that can be used for any EtherCAT slave */
class esmacat_slave
{
protected:
public:
    /** @brief Input System parameter type*/
    uint16_t IN_system_parameter_type;
    /** @brief Input System paramter value */
    uint16_t IN_system_parameter_value;
    /** @brief Output System parameter type */
    uint16_t OUT_system_parameter_type;
    /** @brief Output System parameter value */
    uint16_t OUT_system_parameter_value;
    /** @brief Queue containing all the system parameter types */
    std::queue <uint16_t> system_parameter_type_queue;
    /** @brief Queue containing all the system parameter values */
    std::queue <uint16_t> system_parameter_value_queue;

    esmacat_slave();
    virtual ~esmacat_slave();
    virtual int flush_one_set_of_system_parameters();
    virtual esmacat_err add_system_parameters_in_queue(uint16_t system_parameter_type, uint16_t system_parameter_value);

    /** @brief Pure virtual function that has to be implemented by the child classes */
    virtual uint32_t get_esmacat_product_id()=0;
    /** @brief Pure virtual function that has to be implemented by the child classes */
    virtual uint32_t get_esmacat_vendor_id()=0;
    virtual void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
};

#endif // ESMACAT_SLAVE_H
