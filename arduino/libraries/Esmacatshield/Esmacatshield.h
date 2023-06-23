/*
  Esmacatshield.h - Library for using EtherCAT Arduino Shield by Esmacat (EASE) with Arduino ecosystem.
  Created by Harmonic Bionics, Inc., 05/03/2021
  For any questions or comments, please contact info@esmacat.com
*/

#ifndef Esmacatshield_h
#define Esmacatshield_h

#include "Arduino.h"
#include "SPI.h"

#define READ_REG    	0B00000000
#define WRITE_REG   	0B10000000
#define SINGLE_SHOT   	0B10111111
#define LED_ON    		0B00000100
#define LED_OFF			0B11111011

class Esmacatshield
{
public:
  Esmacatshield(int pin = 10);
  void start_spi(void);
  void write_reg_value(int write_addr,int value, bool led_on=1);
  int* get_ecat_registers(int regs[8]); 
private:
  int ecat_cs;
  SPISettings ecat_spi_settings;
  int read_reg_value(int read_addr);
};

#endif
