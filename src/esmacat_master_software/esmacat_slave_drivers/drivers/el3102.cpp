/** @file
 * @brief Contains the definitions for all the functions used by the EL3102 slave
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "el3102.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the class that holds the initialization
 *
 * Intializes all the variables and sets the product code and Vendor ID
 * for the slave
 *
 */

esmacat_el3102::esmacat_el3102(){
    esmacat_slave_vendor_id = ESMACAT_EL3102_VENDOR_ID;
    esmacat_slave_product_id = ESMACAT_EL3102_PRODUCT_ID;
	input_variable_0_Status=0;
	input_variable_1_Value=0;
	input_variable_2_Status=0;
	input_variable_3_Value=0;
}

/** @brief Returns the value of class variable input_variable_0_Status
 *
 * @return Byte containing Variable 0 status
 */
uint8_t esmacat_el3102::get_input_variable_0_Status()
{
	return input_variable_0_Status;  
}

/** @brief Returns the value of class variable input_variable_1_Value
 *
 * @return Byte containing Variable 1 value
 */
int16_t esmacat_el3102::get_input_variable_1_Value()
{
	return input_variable_1_Value;  
}

/** @brief Returns the value of class variable input_variable_2_Status
 *
 * @return Byte containing Variable 2 status
 */
uint8_t esmacat_el3102::get_input_variable_2_Status()
{
	return input_variable_2_Status;  
}

/** @brief Returns the value of class variable input_variable_3_Value
 *
 * @return Byte containing Variable 3 value
 */
int16_t esmacat_el3102::get_input_variable_3_Value()
{
	return input_variable_3_Value;  
}

/** @brief Data exchange process that is specific to the EL3102 slave
 *
 * Access the first 6 bytes.
 *
 * Variable 0 status = Byte 0
 *
 * Variable 1 value = Bytes 2, Byte 1
 *
 * Variable 2 status = Byte 3
 *
 * Variable 3 value = Byte 5, Byte 4
 *
  * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
  * @param oloop No. of output bytes
  * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
  * @param iloop No. of input bytes
  */
void esmacat_el3102::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){
	unsigned char input_variable[6];
	input_variable[0] = *(ec_slave_inputs+0);
	input_variable[1] = *(ec_slave_inputs+1);
	input_variable[2] = *(ec_slave_inputs+2);
	input_variable[3] = *(ec_slave_inputs+3);
	input_variable[4] = *(ec_slave_inputs+4);
	input_variable[5] = *(ec_slave_inputs+5);
	input_variable_0_Status = +(input_variable[0] << 0);
	input_variable_1_Value = +(input_variable[1] << 0)+(input_variable[2] << 8);
	input_variable_2_Status = +(input_variable[3] << 0);
	input_variable_3_Value = +(input_variable[4] << 0)+(input_variable[5] << 8);
}
/** @brief Reads the product id of the slave
 * @return Product ID of the slave
 */
uint32_t esmacat_el3102::get_esmacat_product_id()
{
    return esmacat_slave_product_id;
}
/** @brief Returns the vendor ID of the slave
 * @return Vendor ID of the slave
 */
uint32_t esmacat_el3102::get_esmacat_vendor_id()
{
    return esmacat_slave_vendor_id;
}
