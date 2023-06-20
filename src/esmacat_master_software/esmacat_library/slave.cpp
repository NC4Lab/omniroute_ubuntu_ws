/** @file
 * @brief This file contains the definitions for the functions in the parent EtherCAT slave class
 */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "slave.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the EtherCAT slave class that will contain the initialization for
  * class variables*/
esmacat_slave::esmacat_slave()
{

}

/** @brief Destructor for the EtherCAT slave class */
esmacat_slave::~esmacat_slave()
{
    std::cout << "esmacat_slave object is destructed"  << std::endl;
}

/** @brief Flushes a system parameter type and value from the queue
 *
 * This function reads the value at the front of the queue (LIFO),
 * and if this parameter type is the same as the input parameter type,
 * then this value is popped and therefore flushed
 *
 * @return Size of the queue after parameter is flushed
 */

int esmacat_slave::flush_one_set_of_system_parameters()
{
    // set system parameters for multifunction
    if (system_parameter_type_queue.size() == 0 )
    {
        OUT_system_parameter_type = 0;
        OUT_system_parameter_value = 0;
        return 0;
    }
    else
    {
        OUT_system_parameter_type = system_parameter_type_queue.front();
        OUT_system_parameter_value = system_parameter_value_queue.front();
        if (IN_system_parameter_type == OUT_system_parameter_type )
        {
            std::cout << "succeed for setup, setup type: " << IN_system_parameter_type  << "  setup value: " << IN_system_parameter_value << std::endl;
            system_parameter_type_queue.pop();
            system_parameter_value_queue.pop();
        }
        return system_parameter_type_queue.size();
    }
}

/** @brief Pushes the parameter type and value onto their corresponding queues
 *
 * Provides error handling. If the queue is full, then an error message is output
 * ERR_SLAVE_QUEUE_FULL, else NO_ERR is returned
 *
 * @param system_parameter_type Type of system parameter to be queued
 * @param system_parameter_value Value of system paramter to be queued
 * @return Error status of the function ( ERR_SLAVE_QUEUE_FULL or NO_ERR)
 */
esmacat_err esmacat_slave::add_system_parameters_in_queue(uint16_t system_parameter_type, uint16_t system_parameter_value)
{
    esmacat_err error;
    if (system_parameter_type_queue.size() > MAX_BUFFER_OF_SYSTEM_PARAMETER || system_parameter_value_queue.size() > MAX_BUFFER_OF_SYSTEM_PARAMETER)
    {
        std::cout << "You cannot queue the system parameters more than " << MAX_BUFFER_OF_SYSTEM_PARAMETER << std::endl;
        error = ERR_SLAVE_QUEUE_FULL;
    }
    else
    {
        system_parameter_type_queue.push(system_parameter_type);
        system_parameter_value_queue.push(system_parameter_value);
        error = NO_ERR;
    }
    return error;
}

/** @brief Virtual function that will be overwritten by the child class */
void esmacat_slave::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop)
{

}
