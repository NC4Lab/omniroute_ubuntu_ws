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

	// Feeder port servo parameters
	Servo portServo;			  // Servo object
	const int portServoPin = 8;	  // Servo pwm pin
	const int portUpAngle = 180;  // Servo up angle
	const int portDownAngle = 90; // Servo down angle

	// Pump servo parameters
	Servo pumpServo;				// Servo object
	const int pumpServoPin = 9;		// Servo pwm pin
	const int pumpRunSpeed = 500;	// Servo run speed (Forward: 500us to 1400us, Backward: 1600us to 2500us, Stop: 1500us)
	const int pumpStopSpeed = 1500; // Servo run speed (Forward: 500us to 1400us, Backward: 1600us to 2500us, Stop: 1500us)

	// Instance of EsmacatCom class using SPI chip select pin 10
	EsmacatCom EsmaCom; 

private:
	// Instance of MazeDebug class for debugging messages
	MazeDebug _Dbg;

	// ---------------METHODS---------------
public:
	GantryOperation();

private:
	uint8_t _grblWrite(const String &cmd, unsigned long timeout = 1000);

private:
	uint8_t _grblRead(String &data, unsigned long timeout = 1000);

public:
	void grblInit();

public:
	void gantryHome();

public:
	void procEcatMessage();

public:
	void debugPrintSerialChars();

public:
	void servoInit();

public:
	void feederLower();

public:
	void feederRaise();

public:
	void pumpStart();

public:
	void pumpStop();

public:
	void reward(int dt_run);
};

#endif
