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
	portServo.write(portUpAngle);	// Set the servo to the up position

	// Setup the pump servo
	pumpServo.attach(pumpServoPin); // Attach the servo object to the pwm pin
}

/// @brief Lower the feeder.
void FeederServo::lowerFeeder()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "[lowerFeeder] Lowering the feeder");
	portServo.write(portDownAngle);
}

/// @brief Raise the feeder.
void FeederServo::raiseFeeder()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "[lowerFeeder] Raising the feeder");
	portServo.write(portUpAngle);
}

/// @brief Start the pump.
void FeederServo::startPump()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "[lowerFeeder] Running the pump");
	pumpServo.write(pumpRunSpeed);
}

/// @brief Stop the pump.
void FeederServo::stopPump()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "[lowerFeeder] Stopping the pump");
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

void FeederServo::_grblInit(const std::string &cmd)
{
	std::string full_cmd = cmd + "\n";
	Serial1.write(full_cmd);
	delay(1000);
	while (true)
	{
		std::string feedback = ser.readline();
		if (feedback == "ok\n")
		{
			_Dbg.printMsg(_Dbg.MT::INFO, "[_grblInit] feedback = %s", feedback.c_str());
			break;
		}
	}
}

void _readSerial(std::string &data) {
  // Clear the string to remove any old data
  data.clear();
  
  while (Serial1.available() > 0) {
    // Read a byte from the Serial1 buffer
    char incomingByte = Serial1.read();
    // Append the byte to the std::string
    data += incomingByte;
  }
}

void FeederServo::grblSetup()
{

	// Set Units (does not seem to work on ender 5)
	_grblInit("G21"); // millimeters

	// Absolute Mode
	_grblInit("G90");

	// Relative Mode
	// _grblInit("G91");

	// Feed Rate
	_grblInit("F25000");
}

void FeederServo::cmdRealTime(const std::string &cmd)
{
	Serial1.write(cmd);
	delay(100); // wait for the command to be executed
}

bool FeederServo::cmdRaw(const std::string &cmd)
{
	try
	{
		std::string full_cmd = cmd + "\n";
		Serial1.write(full_cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		std::string feedback = ser.readline();
		return feedback == "ok\n";
	}
	catch (const std::exception &e)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[_grblInit] Gcode commands must be a string");
		return false;
	}
}

// Function prototype
void readSerial1Buffer(std::string &data);

// Global variable to store serial data
std::string serialData;

void setup()
{
	// Initialize Serial1 at a baud rate of 9600
	Serial1.begin(9600);
	// Initialize Serial for debugging
	Serial.begin(9600);
}

void loop()
{
	// Call the function to read data from Serial1
	readSerial1Buffer(serialData);

	// Optionally, print the data for debugging
	if (!serialData.empty())
	{
		Serial.println(serialData.c_str());
	}

	// Add a small delay to avoid overwhelming the loop
	delay(100);
}

// Function to read from Serial1 buffer and store it in an std::string
void readSerial1Buffer(std::string &data)
{
	while (Serial1.available() > 0)
	{
		// Read a byte from the Serial1 buffer
		char incomingByte = Serial1.read();
		// Append the byte to the std::string
		data += incomingByte;
	}
}

// void command(const std::string &cmd)
// {
// 	try
// 	{
// 		std::string upper_cmd = cmd;
// 		std::transform(upper_cmd.begin(), upper_cmd.end(), upper_cmd.begin(), ::toupper);
// 		std::istringstream iss(upper_cmd);
// 		std::string subcmd;

// 		while (iss >> subcmd)
// 		{
// 			if (subcmd[0] == 'X')
// 			{
// 				value_X += std::stod(subcmd.substr(1));
// 			}
// 			else if (subcmd[0] == 'Y')
// 			{
// 				value_Y += std::stod(subcmd.substr(1));
// 			}
// 		}

// 		std::string full_cmd = cmd + "\n";
// 		Serial1.write(full_cmd);
// 		std::this_thread::sleep_for(std::chrono::seconds(1));
// 		while (true)
// 		{
// 			std::string feedback = ser.readline();
// 			if (feedback == "ok\n")
// 			{
// 				// std::cout << feedback << std::endl;
// 				break;
// 			}
// 		}
// 	}
// 	catch (const std::exception &e)
// 	{
// 		std::cerr << "Gcode commands must be a string" << std::endl;
// 	}
// }
