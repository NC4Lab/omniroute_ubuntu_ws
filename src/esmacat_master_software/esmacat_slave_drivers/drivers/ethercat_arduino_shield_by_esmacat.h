/** @file
 * @brief This file contains all the declarations for the EtherCAT Arduino Shield by Esmacat
 */

#ifndef ESMACAT_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT_H
#define ESMACAT_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT_H


/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique vendor ID assigned by the EtherCAT Technology Group to Beckhoff*/
#define	ESMACAT_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT_VENDOR_ID	0x0062696f
/** @brief Unique Product Code assigned to the EtherCAT Arduino Shield by Esmacat*/
#define	ESMACAT_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT_PRODUCT_ID	0x00090101


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "slave.h"


/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/** @brief Generic class that allows for access of information from the Arduino Shield slave*/
class esmacat_ethercat_arduino_shield_by_esmacat: public esmacat_slave
{
private:
    /** @brief Contains the value of input variable 0 */
	int16_t	input_variable_0_IN_GEN_INT0;
    /** @brief Contains the value of input variable 1 */
	int16_t	input_variable_1_IN_GEN_INT1;
    /** @brief Contains the value of input variable 2 */
	int16_t	input_variable_2_IN_GEN_INT2;
    /** @brief Contains the value of input variable 3 */
	int16_t	input_variable_3_IN_GEN_INT3;
    /** @brief Contains the value of input variable 4 */
	int16_t	input_variable_4_IN_GEN_INT4;
    /** @brief Contains the value of input variable 5 */
	int16_t	input_variable_5_IN_GEN_INT5;
    /** @brief Contains the value of input variable 6 */
	int16_t	input_variable_6_IN_GEN_INT6;
    /** @brief Contains the value of input variable 7 */
	int16_t	input_variable_7_IN_GEN_INT7;
    /** @brief Contains the value of output variable 0 */
	int16_t	output_variable_0_OUT_GEN_INT0;
    /** @brief Contains the value of output variable 1 */
	int16_t	output_variable_1_OUT_GEN_INT1;
    /** @brief Contains the value of output variable 2 */
	int16_t	output_variable_2_OUT_GEN_INT2;
    /** @brief Contains the value of output variable 3 */
	int16_t	output_variable_3_OUT_GEN_INT3;
    /** @brief Contains the value of output variable 4 */
	int16_t	output_variable_4_OUT_GEN_INT4;
    /** @brief Contains the value of output variable 5 */
	int16_t	output_variable_5_OUT_GEN_INT5;
    /** @brief Contains the value of output variable 6 */
	int16_t	output_variable_6_OUT_GEN_INT6;
    /** @brief Contains the value of output variable 7 */
	int16_t	output_variable_7_OUT_GEN_INT7;

public:
    /** Contains a product code of the Arduino Shield slave */
	uint32_t esmacat_slave_product_id = ESMACAT_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT_PRODUCT_ID;
    /** Contains a Vendor ID of the Arduino Shield slave */
	uint32_t esmacat_slave_vendor_id = ESMACAT_ETHERCAT_ARDUINO_SHIELD_BY_ESMACAT_VENDOR_ID;

    esmacat_ethercat_arduino_shield_by_esmacat();
    uint32_t get_esmacat_product_id();
    uint32_t get_esmacat_vendor_id();
	void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
	int16_t get_input_variable_0_IN_GEN_INT0();
	int16_t get_input_variable_1_IN_GEN_INT1();
	int16_t get_input_variable_2_IN_GEN_INT2();
	int16_t get_input_variable_3_IN_GEN_INT3();
	int16_t get_input_variable_4_IN_GEN_INT4();
	int16_t get_input_variable_5_IN_GEN_INT5();
	int16_t get_input_variable_6_IN_GEN_INT6();
	int16_t get_input_variable_7_IN_GEN_INT7();
	void set_output_variable_0_OUT_GEN_INT0(int16_t value);
	void set_output_variable_1_OUT_GEN_INT1(int16_t value);
	void set_output_variable_2_OUT_GEN_INT2(int16_t value);
	void set_output_variable_3_OUT_GEN_INT3(int16_t value);
	void set_output_variable_4_OUT_GEN_INT4(int16_t value);
	void set_output_variable_5_OUT_GEN_INT5(int16_t value);
	void set_output_variable_6_OUT_GEN_INT6(int16_t value);
	void set_output_variable_7_OUT_GEN_INT7(int16_t value);
}; 

#endif
