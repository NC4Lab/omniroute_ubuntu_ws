// ######################################

//========= GantryOperation.h ===========

// ######################################

/// @file Used for the GantryOperation class

//============= INCLUDE ================

// BUILT IN
#include "Arduino.h"
#include "Servo.h"

// LOCAL
#include "MazeDebug.h"
#include "EsmacatCom.h"

#ifndef _FEEDER_SERVO_h
#define _FEEDER_SERVO_h

extern bool DB_VERBOSE; ///< set this variable in your INO file to control debugging [0:silent, 1:verbose]

/// @brief Used for printing different types of information to the Serial Output Window.
///
/// @remarks This class is used in both the CypressComm and WallOperation classes.
class GantryOperation
{

	// ---------------VARIABLES---------------
public:
	// GRBL acknoledgement timeout
	const unsigned long grblAckTimeout = 1000; // Timeout for grbl acknoledgement (ms)

	// Gantry position parameters
	double value_X = 0.0;
	double value_Y = 0.0;

	// Store max feed rate
	float maxFeedRate = 50000.0;

	// Feeder port servo parameters
	Servo portServo;			 // Servo object
	const int portServoPin = 8;	 // Servo pwm pin
	const int portUpAngle = 270; // Servo up angle
	const int portDownAngle = 90; // Servo down angle

	// Pump servo parameters
	Servo pumpServo;				// Servo object
	const int pumpServoPin = 9;		// Servo pwm pin
	const int pumpRunSpeed = 500;	// Servo run speed (Forward: 500us to 1400us, Backward: 1600us to 2500us, Stop: 1500us)
	const int pumpStopSpeed = 1500; // Servo run speed (Forward: 500us to 1400us, Backward: 1600us to 2500us, Stop: 1500us)

	// Instance of EsmacatCom class using SPI chip select pin 10
	EsmacatCom EsmaCom;

private:
	// Struct to hold GRBL settings
	struct GRBLSetting
	{
		String command; // The GRBL command string, e.g., "$0"
		float value;	// The value associated with the setting, e.g., 10.0
	};

	// Fixed size array to store all GRBL settings
	GRBLSetting _grblSettings[70] = {
		{"$0", 10.0},		 // Step pulse time, microseconds [1.0 - 100.0]
		{"$1", 50.0},		 // Step idle delay, milliseconds [0.0 - 255.0]
		{"$2", 0.0},		 // Step pulse invert, mask [0.0 - 7.0]
		{"$3", 3.0},		 // Step direction invert, mask [0.0 - 7.0]
		{"$4", 0.0},		 // Step enable invert, boolean [0.0, 1.0]
		{"$5", 7.0},		 // Invert limit pins, boolean/mask [0.0 - 7.0]
		{"$6", 1.0},		 // Invert probe pin, boolean [0.0, 1.0]
		{"$8", 0.0},		 // Ganged axes direction invert as bitfield [0.0, 1.0]
		{"$9", 1.0},		 // PWM Spindle as bitfield, enable [0.0, 1.0]
		{"$10", 511.0},		 // Status report options, mask [0.0 - 511.0]
		{"$11", 0.010},		 // Junction deviation, millimeters [0.0 - 0.999]
		{"$12", 0.002},		 // Arc tolerance, millimeters [0.001 - 1.0]
		{"$13", 0.0},		 // Report in inches, boolean [0.0, 1.0]
		{"$14", 0.0},		 // Limit pins invert, mask [0.0, 1.0]
		{"$15", 0.0},		 // Coolant pins invert, mask [0.0, 1.0]
		{"$16", 0.0},		 // Spindle pins invert, mask [0.0, 1.0]
		{"$17", 0.0},		 // Control pins pullup disable, mask [0.0, 1.0]
		{"$18", 0.0},		 // Limit pins pullup disable, mask [0.0, 1.0]
		{"$19", 0.0},		 // Probe pin pullup disable, boolean [0.0, 1.0]
		{"$20", 1.0},		 // Soft limits enable, boolean [0.0, 1.0]
		{"$21", 1.0},		 // Hard limits enable, boolean [0.0, 1.0]
		{"$22", 1.0},		 // Homing cycle enable, boolean [0.0, 1.0]
		{"$23", 7.0},		 // Homing direction invert, mask [0.0 - 7.0]
		{"$24", 1000.0},	 // Homing locate feed rate, mm/min [0.0 - 5000.0]
		{"$25", 7500.0},	 // Homing search seek rate, mm/min [0.0 - 7500.0]
		{"$26", 250.0},		 // Homing switch debounce delay, milliseconds [0.0 - 255.0]
		{"$27", 5.000},		 // Homing switch pull-off distance, millimeters [0.0 - 50.0]
		{"$28", 0.100},		 // G73 retract distance, millimeters [0.0 - 10.0]
		{"$29", 5.0},		 // Step pulse delay, milliseconds [0.0 - 10.0]
		{"$30", 1000.000},	 // Maximum spindle speed, RPM [0.0 - 100000.0]
		{"$31", 0.000},		 // Minimum spindle speed, RPM [0.0 - 1000.0]
		{"$32", 0.0},		 // Laser-mode enable, boolean [0.0, 1.0]
		{"$33", 50.0},		 // Spindle PWM frequency [0.0 - 5000.0]
		{"$34", 0.0},		 // Spindle off value [0.0 - 100.0]
		{"$35", 5.0},		 // Spindle min value [0.0 - 100.0]
		{"$36", 10.0},		 // Spindle max value [0.0 - 100.0]
		{"$37", 0.0},		 // Stepper deenergize mask [0.0, 1.0]
		{"$39", 1.0},		 // Enable printable realtime command characters, boolean [0.0, 1.0]
		{"$40", 1.0},		 // Apply soft limits for jog commands, boolean [0.0, 1.0]
		{"$43", 1.0},		 // Homing passes [0.0 - 5.0]
		{"$44", 3.0},		 // Homing cycle 1 [0.0 - 7.0]
		{"$45", 0.0},		 // Homing cycle 2 [0.0 - 7.0]
		{"$46", 0.0},		 // Homing cycle 3 [0.0 - 7.0]
		{"$62", 0.0},		 // Sleep Enable [0.0, 1.0]
		{"$63", 2.0},		 // Feed Hold Actions [0.0 - 2.0]
		{"$64", 0.0},		 // Force Init Alarm [0.0, 1.0]
		{"$65", 0.0},		 // Require homing sequence at startup [0.0, 1.0]
		{"$70", 7.0},		 // Network Services [0.0 - 7.0]
		{"$73", 1.0},		 // Wifi Mode [0.0, 1.0]
		{"$100", 114.286},	 // X-axis steps per millimeter
		{"$101", 114.286},	 // Y-axis steps per millimeter
		{"$102", 114.286},	 // Z-axis steps per millimeter
		{"$110", 50000.000}, // X-axis maximum rate, mm/min [0.0 - 100000.0]
		{"$111", 50000.000}, // Y-axis maximum rate, mm/min [0.0 - 100000.0]
		{"$112", 50000.000}, // Z-axis maximum rate, mm/min [0.0 - 100000.0]
		{"$120", 500.000},	 // X-axis acceleration, mm/sec^2 [0.0 - 10000.0]
		{"$121", 500.000},	 // Y-axis acceleration, mm/sec^2 [0.0 - 10000.0]
		{"$122", 500.000},	 // Z-axis acceleration, mm/sec^2 [0.0 - 10000.0]
		{"$130", 830.000},	 // X-axis maximum travel, millimeters [0.0 - 2000.0]
		{"$131", 1016.000},	 // Y-axis maximum travel, millimeters [0.0 - 2000.0]
		{"$132", 0.000},	 // Z-axis maximum travel, millimeters [0.0 - 2000.0]
		{"$341", 0.0},		 // Tool Change Mode [0.0, 1.0]
		{"$342", 30.0},		 // Tool Change probing distance, millimeters [0.0 - 100.0]
		{"$343", 25.0},		 // Tool Change Locate Feed rate, mm/min [0.0 - 5000.0]
		{"$344", 200.0},	 // Tool Change Search Seek rate, mm/min [0.0 - 5000.0]
		{"$345", 200.0},	 // Tool Change Probe Pull Off rate, mm/min [0.0 - 5000.0]
		{"$346", 1.0},		 // Restore position after M6 as boolean [0.0, 1.0]
		{"$370", 0.0},		 // Invert I/O Port Inputs (mask) [0.0, 7.0]
		{"$384", 0.0},		 // Disable G92 Persistence [0.0, 1.0]
		{"$396", 30.0},		 // WebUI timeout in minutes [0.0 - 60].
	};

	// Instance of MazeDebug class for debugging messages
	MazeDebug _Dbg;

	// ---------------METHODS---------------
public:
	GantryOperation();

public:
	uint8_t grblWrite(const String &cmd, bool do_wait_ack = true, unsigned long timeout = 500);

public:
	uint8_t grblRead(String &data, unsigned long timeout = 500, bool do_print_response = false);

public:
	void grblInitSystem();

public:
	void grblInitRuntime(float max_feed_rate, float max_acceleration);

public:
	void gantryHome(uint16_t home_speed);

public:
	void gantryMove(float x, float y, float max_feed_rate);

public:
	void grblJogCancel();

public:
	void grblResetAlarm();

public:
	void procEcatMessage();

public:
	void debugPrintSerialChars();

public:
	void servoInit();

public:
	void feederMove(uint8_t move_dir);

public:
	void pumpRun(uint8_t run_state);

public:
	void runReward(float diration);
};

#endif
