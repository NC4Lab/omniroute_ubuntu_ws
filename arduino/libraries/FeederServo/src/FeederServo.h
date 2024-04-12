// ######################################

//=========== FeederServo.h =============

// ######################################

/// @file Used for the FeederServo class

//============= INCLUDE ================
#include "Arduino.h"
#include "Servo.h"
#include "MazeDebug.h"
#include "EsmacatCom.h"

#ifndef _FEEDER_SERVO_h
#define _FEEDER_SERVO_h

extern bool DB_VERBOSE; ///< set this variable in your INO file to control debugging [0:silent, 1:verbose]

/// @brief Used for printing different types of information to the Serial Output Window.
///
/// @remarks This class is used in both the CypressComm and WallOperation classes.
class FeederServo
{

	// ---------------VARIABLES---------------
public:
	// Feeder port servo parameters
	Servo portServo;		// Servo object
	int portServoPin = 11;	// Servo pwm pin
	int portUpAngle = 180;	// Servo up angle
	int portDownAngle = 90; // Servo down angle

	// Pump servo parameters
	Servo pumpServo;		  // Servo object
	int pumpServoPin = 12;	  // Servo pwm pin
	int pumpRunSpeed = 500;	  // Servo run speed (Forward: 500us to 1400us, Backward: 1600us to 2500us, Stop: 1500us)
	int pumpStopSpeed = 1500; // Servo run speed (Forward: 500us to 1400us, Backward: 1600us to 2500us, Stop: 1500us)

private:
	MazeDebug _Dbg; /// unique instance of MazeDebug class

	// ---------------METHODS---------------
public:
	FeederServo();

public:
	void initServo();

public:
	void lowerFeeder();

public:
	void raiseFeeder();

public:
	void startPump();

public:
	void stopPump();

public:
	void runFeeder(int dt_run);
};

#endif
