/** @file
 * @brief This file contains the declarations for the generic Esmacat slave
 */
#ifndef ESMACAT_BASE_MODULE_H
#define ESMACAT_BASE_MODULE_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique Vendor ID assigned to Harmonic Bionics, Inc. by the EtherCAT Technology Group */
#define ETHERCAT_VENDOR_ID_OF_HARMONIC_BIONICS 0x0062696F

/** @brief Max Number of digital inputs and outputs supported by all Esmacat slaves */
#define ESMACAT_BASE_NUMBER_OF_DIO 7

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "slave.h"
#include "error_list.h"

/*****************************************************************************************
 * ENUMERATIONS
 ****************************************************************************************/
/** @brief Direction for the input/output */
typedef enum{
    IO_INPUT = 0, /**< Configured as an input */
	IO_OUTPUT = 1 /**< Configured as an output */
}IO_Direction;


/** @brief Identifies if the analog input is single-ended or differential */
enum ADC_input_single_ended_diff {
    SINGLE_ENDED_INPUT = 0, /**< Single-ended, reference is ground; input signal V's voltage reading = V - Vground */
    DIFFERENTIAL_INPUT = 1 /**< Differential signal, two inputs Vp and Vn, voltage reading = Vp - Vn */
};

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

/** @brief This is the parent class for all Esmacat slaves */
class esmacat_base_module : public esmacat_slave{
private:
    /** @brief Private variable - Digital input of the system */
    bool sys_digital_in;
     /** @brief Private variable - Digital input of Esmacat slave */
    bool digital_data_in[ESMACAT_BASE_NUMBER_OF_DIO];
     /** @brief Private variable - Digital output of the system */
    bool user_led_out;
     /** @brief Private variable - Digital output of the Esmacat slave */
    bool digital_data_out[ESMACAT_BASE_NUMBER_OF_DIO];
     /** @brief Private variable - Direction of the 8 I/O available on the slave */
    bool ICAT_base_dio_direction[ESMACAT_BASE_NUMBER_OF_DIO];

public:
    esmacat_base_module();
    void set_usr_led(bool led_on);
    esmacat_err set_digital_output(int digital_output_index, bool digital_output_value);
    bool get_digital_input(int index_of_digital_input, esmacat_err& error);
    bool get_digital_input(int index_of_digital_input);
    void ecat_data_process_base_module(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);
    void configure_slave_dio_direction(IO_Direction dio_direction[ESMACAT_BASE_NUMBER_OF_DIO]);
    virtual void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
};

#endif // ESMACAT_BASE_MODULE_H



