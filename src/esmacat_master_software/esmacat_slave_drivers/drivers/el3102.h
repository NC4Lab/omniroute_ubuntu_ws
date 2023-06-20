/** @file
 * @brief This file contains all the declarations for the EL3102 from Beckhoff
 */

#ifndef ESMACAT_EL3102_H
#define ESMACAT_EL3102_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique vendor ID assigned by the EtherCAT Technology Group to Beckhoff*/
#define	ESMACAT_EL3102_VENDOR_ID	0x00000002
/** @brief Unique product code for EL3102 required for EtherCAT */
#define	ESMACAT_EL3102_PRODUCT_ID	0x0c1e3052

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "slave.h"

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/** @brief Generic class that allows for access of information from the EL3102 slave*/
class esmacat_el3102: public esmacat_slave
{
private:
    /** @brief Contains the status of variable 0 */
	uint8_t	input_variable_0_Status;
    /** @brief Contains the value of variable 1 */
	int16_t	input_variable_1_Value;
    /** @brief Contains the status of variable 2 */
	uint8_t	input_variable_2_Status;
    /** @brief Contains the value of variable 3 */
	int16_t	input_variable_3_Value;

public:
    /** Contains a product code of the EL3102 slave */
    uint32_t esmacat_slave_product_id = ESMACAT_EL3102_PRODUCT_ID;
    /** Contains a vendor ID assigned by the EtherCAT Technology Group*/
    uint32_t esmacat_slave_vendor_id = ESMACAT_EL3102_VENDOR_ID;

	esmacat_el3102();
	void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
	uint8_t get_input_variable_0_Status();
	int16_t get_input_variable_1_Value();
	uint8_t get_input_variable_2_Status();
	int16_t get_input_variable_3_Value();
    uint32_t get_esmacat_product_id();
    uint32_t get_esmacat_vendor_id();
}; 

#endif
