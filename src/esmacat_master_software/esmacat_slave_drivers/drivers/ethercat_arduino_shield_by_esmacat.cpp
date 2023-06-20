/** @file
 * @brief Contains the definitions for all the functions used by the EtherCAT Arduino Shield by Esmacat slave
 */


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "ethercat_arduino_shield_by_esmacat.h"


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the class that holds the initialization
 *
 * Intializes all the variables to 0 and sets the product code and Vendor ID
 * for the slave
 *
 */
esmacat_ethercat_arduino_shield_by_esmacat::esmacat_ethercat_arduino_shield_by_esmacat()
{
	esmacat_slave_vendor_id = 0x0062696f;
	esmacat_slave_product_id = 0x00090101;
	input_variable_0_IN_GEN_INT0=0;
	input_variable_1_IN_GEN_INT1=0;
	input_variable_2_IN_GEN_INT2=0;
	input_variable_3_IN_GEN_INT3=0;
	input_variable_4_IN_GEN_INT4=0;
	input_variable_5_IN_GEN_INT5=0;
	input_variable_6_IN_GEN_INT6=0;
	input_variable_7_IN_GEN_INT7=0;
	output_variable_0_OUT_GEN_INT0=0;
	output_variable_1_OUT_GEN_INT1=0;
	output_variable_2_OUT_GEN_INT2=0;
	output_variable_3_OUT_GEN_INT3=0;
	output_variable_4_OUT_GEN_INT4=0;
	output_variable_5_OUT_GEN_INT5=0;
	output_variable_6_OUT_GEN_INT6=0;
	output_variable_7_OUT_GEN_INT7=0;
}
/** @brief Returns the value of input variable 0
 *
 * @return Signed integer containing Variable 0 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_0_IN_GEN_INT0()
{
	return input_variable_0_IN_GEN_INT0;  
}

/** @brief Returns the value of input variable 1
 *
 * @return Signed integer containing Variable 1 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_1_IN_GEN_INT1()
{
	return input_variable_1_IN_GEN_INT1;  
}

/** @brief Returns the value of input variable 2
 *
 * @return Signed integer containing Variable 2 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_2_IN_GEN_INT2()
{
	return input_variable_2_IN_GEN_INT2;  
}

/** @brief Returns the value of input variable 3
 *
 * @return Signed integer containing Variable 3 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_3_IN_GEN_INT3()
{
	return input_variable_3_IN_GEN_INT3;  
}

/** @brief Returns the value of input variable 4
 *
 * @return Signed integer containing Variable 4 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_4_IN_GEN_INT4()
{
	return input_variable_4_IN_GEN_INT4;  
}

/** @brief Returns the value of input variable 5
 *
 * @return Signed integer containing Variable 5 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_5_IN_GEN_INT5()
{
	return input_variable_5_IN_GEN_INT5;  
}

/** @brief Returns the value of input variable 6
 *
 * @return Signed integer containing Variable 6 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_6_IN_GEN_INT6()
{
	return input_variable_6_IN_GEN_INT6;  
}

/** @brief Returns the value of input variable 7
 *
 * @return Signed integer containing Variable 7 value
 */
int16_t esmacat_ethercat_arduino_shield_by_esmacat::get_input_variable_7_IN_GEN_INT7()
{
	return input_variable_7_IN_GEN_INT7;  
}

/** @brief Sets the value of output variable 0
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_0_OUT_GEN_INT0(int16_t value)
{
	output_variable_0_OUT_GEN_INT0 = value;  
}

/** @brief Sets the value of output variable 1
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_1_OUT_GEN_INT1(int16_t value)
{
	output_variable_1_OUT_GEN_INT1 = value;  
}

/** @brief Sets the value of output variable 2
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_2_OUT_GEN_INT2(int16_t value)
{
	output_variable_2_OUT_GEN_INT2 = value;  
}

/** @brief Sets the value of output variable 3
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_3_OUT_GEN_INT3(int16_t value)
{
	output_variable_3_OUT_GEN_INT3 = value;  
}

/** @brief Sets the value of output variable 4
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_4_OUT_GEN_INT4(int16_t value)
{
	output_variable_4_OUT_GEN_INT4 = value;  
}

/** @brief Sets the value of output variable 5
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_5_OUT_GEN_INT5(int16_t value)
{
	output_variable_5_OUT_GEN_INT5 = value;  
}

/** @brief Sets the value of output variable 6
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_6_OUT_GEN_INT6(int16_t value)
{
	output_variable_6_OUT_GEN_INT6 = value;  
}

/** @brief Sets the value of output variable 7
 */
void esmacat_ethercat_arduino_shield_by_esmacat::set_output_variable_7_OUT_GEN_INT7(int16_t value)
{
	output_variable_7_OUT_GEN_INT7 = value;  
}

/** @brief Data exchange process that is specific to the Arduino Shield slave
 * Queues the values of the received input into the input_variable and queues all
 * the values to be transmitted into output_variable which is then sent out
 * using ethercat
 *
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param oloop No. of output bytes
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 * @param iloop No. of input bytes
 */

void esmacat_ethercat_arduino_shield_by_esmacat::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop)
{
	unsigned char input_variable[16];
	unsigned char output_variable[16];
	input_variable[0] = *(ec_slave_inputs+0);
	input_variable[1] = *(ec_slave_inputs+1);
	input_variable[2] = *(ec_slave_inputs+2);
	input_variable[3] = *(ec_slave_inputs+3);
	input_variable[4] = *(ec_slave_inputs+4);
	input_variable[5] = *(ec_slave_inputs+5);
	input_variable[6] = *(ec_slave_inputs+6);
	input_variable[7] = *(ec_slave_inputs+7);
	input_variable[8] = *(ec_slave_inputs+8);
	input_variable[9] = *(ec_slave_inputs+9);
	input_variable[10] = *(ec_slave_inputs+10);
	input_variable[11] = *(ec_slave_inputs+11);
	input_variable[12] = *(ec_slave_inputs+12);
	input_variable[13] = *(ec_slave_inputs+13);
	input_variable[14] = *(ec_slave_inputs+14);
	input_variable[15] = *(ec_slave_inputs+15);
	input_variable_0_IN_GEN_INT0 = +(input_variable[0] << 0)+(input_variable[1] << 8);
	input_variable_1_IN_GEN_INT1 = +(input_variable[2] << 0)+(input_variable[3] << 8);
	input_variable_2_IN_GEN_INT2 = +(input_variable[4] << 0)+(input_variable[5] << 8);
	input_variable_3_IN_GEN_INT3 = +(input_variable[6] << 0)+(input_variable[7] << 8);
	input_variable_4_IN_GEN_INT4 = +(input_variable[8] << 0)+(input_variable[9] << 8);
	input_variable_5_IN_GEN_INT5 = +(input_variable[10] << 0)+(input_variable[11] << 8);
	input_variable_6_IN_GEN_INT6 = +(input_variable[12] << 0)+(input_variable[13] << 8);
	input_variable_7_IN_GEN_INT7 = +(input_variable[14] << 0)+(input_variable[15] << 8);
	output_variable[0] = 0;
	output_variable[1] = 0;
	output_variable[2] = 0;
	output_variable[3] = 0;
	output_variable[4] = 0;
	output_variable[5] = 0;
	output_variable[6] = 0;
	output_variable[7] = 0;
	output_variable[8] = 0;
	output_variable[9] = 0;
	output_variable[10] = 0;
	output_variable[11] = 0;
	output_variable[12] = 0;
	output_variable[13] = 0;
	output_variable[14] = 0;
	output_variable[15] = 0;
	output_variable[0] =  +((output_variable_0_OUT_GEN_INT0 >> 0) & 0x00ff);
	output_variable[1] =  +((output_variable_0_OUT_GEN_INT0 >> 8) & 0x00ff);
	output_variable[2] =  +((output_variable_1_OUT_GEN_INT1 >> 0) & 0x00ff);
	output_variable[3] =  +((output_variable_1_OUT_GEN_INT1 >> 8) & 0x00ff);
	output_variable[4] =  +((output_variable_2_OUT_GEN_INT2 >> 0) & 0x00ff);
	output_variable[5] =  +((output_variable_2_OUT_GEN_INT2 >> 8) & 0x00ff);
	output_variable[6] =  +((output_variable_3_OUT_GEN_INT3 >> 0) & 0x00ff);
	output_variable[7] =  +((output_variable_3_OUT_GEN_INT3 >> 8) & 0x00ff);
	output_variable[8] =  +((output_variable_4_OUT_GEN_INT4 >> 0) & 0x00ff);
	output_variable[9] =  +((output_variable_4_OUT_GEN_INT4 >> 8) & 0x00ff);
	output_variable[10] =  +((output_variable_5_OUT_GEN_INT5 >> 0) & 0x00ff);
	output_variable[11] =  +((output_variable_5_OUT_GEN_INT5 >> 8) & 0x00ff);
	output_variable[12] =  +((output_variable_6_OUT_GEN_INT6 >> 0) & 0x00ff);
	output_variable[13] =  +((output_variable_6_OUT_GEN_INT6 >> 8) & 0x00ff);
	output_variable[14] =  +((output_variable_7_OUT_GEN_INT7 >> 0) & 0x00ff);
	output_variable[15] =  +((output_variable_7_OUT_GEN_INT7 >> 8) & 0x00ff);
	*(ec_slave_outputs+0) = output_variable[0];
	*(ec_slave_outputs+1) = output_variable[1];
	*(ec_slave_outputs+2) = output_variable[2];
	*(ec_slave_outputs+3) = output_variable[3];
	*(ec_slave_outputs+4) = output_variable[4];
	*(ec_slave_outputs+5) = output_variable[5];
	*(ec_slave_outputs+6) = output_variable[6];
	*(ec_slave_outputs+7) = output_variable[7];
	*(ec_slave_outputs+8) = output_variable[8];
	*(ec_slave_outputs+9) = output_variable[9];
	*(ec_slave_outputs+10) = output_variable[10];
	*(ec_slave_outputs+11) = output_variable[11];
	*(ec_slave_outputs+12) = output_variable[12];
	*(ec_slave_outputs+13) = output_variable[13];
	*(ec_slave_outputs+14) = output_variable[14];
	*(ec_slave_outputs+15) = output_variable[15];
}
/** @brief Reads the product id of the slave
 * @return Product ID of the slave
 */
uint32_t esmacat_ethercat_arduino_shield_by_esmacat::get_esmacat_product_id()
{
    return esmacat_slave_product_id;
}
/** @brief Reads the Vendor id of the slave
 * @return Vendor ID of the slave
 */
uint32_t esmacat_ethercat_arduino_shield_by_esmacat::get_esmacat_vendor_id()
{
    return esmacat_slave_vendor_id;
}
