// ######################################

//========== FeederServo.cpp ============

// ######################################

/// <file>
/// Used for the FeederServo class
/// <file>

//============= INCLUDE ================
#include "FeederServo.h"

//===========CLASS: FeederServo============

/// @brief Constructor
FeederServo::FeederServo() {}

/// @brief Initialize the servo objects.
void FeederServo::initServo()
{
	// Setup the port servo
	portServo.attach(portServoPin); // Attach the servo object to the pwm pin
	portServo.write(portUpAngle);  // Set the servo to the up position

	// Setup the pump servo
	pumpServo.attach(pumpServoPin); // Attach the servo object to the pwm pin
}

/// @brief Lower the feeder.
void FeederServo::lowerFeeder()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Lowering the feeder");
	portServo.write(portDownAngle);
}

/// @brief Raise the feeder.
void FeederServo::raiseFeeder()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Raising the feeder");
	portServo.write(portUpAngle);
}

/// @brief Start the pump.
void FeederServo::startPump()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Running the pump");
	pumpServo.write(pumpRunSpeed);
}

/// @brief Stop the pump.
void FeederServo::stopPump()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Stopping the pump");
	pumpServo.write(pumpStopSpeed);
}

/// @brief Run the feeder.
void FeederServo::runFeeder(int dt_run)
{
	// Lower the feeder
	lowerFeeder();
	delay(1000);

	// Run the pump
	startPump();
	delay(dt_run);
	stopPump();

	// Raise the feeder
	raiseFeeder();
}
