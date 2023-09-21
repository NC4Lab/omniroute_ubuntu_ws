//######################################

//======= CypressComBase.h ==========

//######################################

/// <file>
/// A near-complete list of Cypress chip registry-related values
/// <file>

#ifndef CYPRESS_COM_BASE_H
#define CYPRESS_COM_BASE_H

#define CY8C95X0_ADDR B0100000
#define CY8C95X0_EEPROM_ADDR B1010000
#define CY8C95X0_PWM_0 0

// Device register map
#define REG_PORT_SEL 0x18 ///<port select
#define REG_INT_MASK 0x19 ///<interrupt mask
#define REG_SEL_PWM_PORT_OUT 0x1A ///<PWM select for port output
#define REG_INVERSION 0x1B ///<inversion
#define REG_PIN_DIR 0x1C ///<pin direction input/output [0:output, 1:input]
#define REG_SEL_PWM 0x28 ///<PWM Select
#define REG_CONF_PWM 0x29 ///<PWM config clock [0: 32 kHz(default), 1: 24 MHz, 2: 1.5 MHz, 3: 93.75 kHz, 4: 367.6 Hz(programmable), 5: previous PWM] 
#define REG_PERI_PWM 0x2A ///<PWM period of the PWM counter [1-255]. Devisor for hardward clock 
#define REG_PW_PWM 0x2B ///<pulse width PWM [0-REG_PERI_PWM-1]. Duty cycle = REG_PW_PWM/REG_PERI_PWM
#define REG_PROG_DIV 0x2C ///<programmable Divider
#define REG_ENABLE 0x2D ///<enable [(]EERO, EEE, WDE]
#define REG_DEV_STATUS 0x2E ///<device ID/Status
#define REG_WATCHDOG 0x2F ///<watchdog
#define REG_CMD 0x30 ///<command

// Interrupt Status 
#define REG_INT_STAT_0 0x10
#define REG_INT_STAT_1 0x11
#define REG_INT_STAT_2 0x12
#define REG_INT_STAT_3 0x13
#define REG_INT_STAT_4 0x14
#define REG_INT_STAT_5 0x15
#define REG_INT_STAT_6 0x16
#define REG_INT_STAT_7 0x17

// Command registers 
#define REG_CMD 0x30 ///<sends commands to the device
#define REG_CMD_STORE 0x01 ///<store device configuration to EEPROM POR defaults
#define REG_CMD_RESTORE 0x02 ///<restore Factory Defaults
#define REG_CMD_STORE_D 0x03 ///<in the datasheet, pg 14, probably not useful on the large scale
#define REG_CMD_READ_D 0x04 ///<this is good for dumping the chip's config
#define REG_CMD_WRITE 0x05
#define REG_CMD_READ 0x06
#define REG_CMD_RECONF 0x07

// Drive modes 
#define DRIVE_PULLUP 0x1D ///<drive mode pull up
#define DRIVE_PULLDOWN 0x1E ///<drive mode pull down
#define DRIVE_OPENHIGH 0x1F ///<drive mode open drain high
#define DRIVE_OPENLOW 0x20 ///<drive mode open drain low
#define DRIVE_STRONG 0x21 ///<drive mode strong
#define DRIVE_SLOW 0x22 ///<drive mode slow strong
#define DRIVE_HIZ 0x23 ///<drive mode High-Z

// IO register mapping
// Input
#define REG_GI0 0x00
#define REG_GI1 0x01
#define REG_GI2 0x02
#define REG_GI3 0x03
#define REG_GI4 0x04
#define REG_GI5 0x05
// Output
#define REG_GO0 0x08
#define REG_GO1 0x09
#define REG_GO2 0x0A
#define REG_GO3 0x0B
#define REG_GO4 0x0C
#define REG_GO5 0x0D 

// Misc defs
#define ALL_PINS 0xFF
#define NO_PINS 0x00
#define MAX_PIN 8
#define MAX_PORT 6

// Hex address list for Cypress chips
/*
* MATLAB CODE
% Generate 49 sequential values with 0 in the first bit
n = 49;
dec_values = zeros(1, n);
hex_values = cell(1, n);
for i = 1:n
	bin_7_str = dec2bin(i, 7); % convert decimal value to 7-bit binary string
	bin_8_str = [bin_7_str '0']; % replace first bit with 0
	dec_values(i) = bin2dec(bin_8_str); % convert binary string to decimal value
	hex_values{i} = dec2hex(dec_values(i));
end
% Print as hex to paist into Arduino code
fprintf('\r')
for i = 1:n
		fprintf('0x%s, ', hex_values{i})
		if mod(i,5)==0
			fprintf('\r')
		end
end
fprintf('\r')
*/

/*
ADDR_LIST[49] = {
0x2, 0x4, 0x6, 0x8, 0xA,
0xC, 0xE, 0x10, 0x12, 0x14,
0x16, 0x18, 0x1A, 0x1C, 0x1E,
0x20, 0x22, 0x24, 0x26, 0x28,
0x2A, 0x2C, 0x2E, 0x30, 0x32,
0x34, 0x36, 0x38, 0x3A, 0x3C,
0x3E, 0x40, 0x42, 0x44, 0x46,
0x48, 0x4A, 0x4C, 0x4E, 0x50,
0x52, 0x54, 0x56, 0x58, 0x5A,
0x5C, 0x5E, 0x60, 0x62
//};
*/

#endif