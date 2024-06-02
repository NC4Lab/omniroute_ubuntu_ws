// ######################################

//======== GantryOperation.cpp ==========

// ######################################

/// <file>
/// Used for the GantryOperation class
/// <file>

//============= INCLUDE ================
#include "GantryOperation.h"

//===========CLASS: GantryOperation============

/// @brief Constructor
GantryOperation::GantryOperation() {}

/// @brief: Write to grbl serial buffer.
///
/// @param cmd_str: Command string to write.
/// @param timeout: Timeout for the grbl acknoledgement.
///
/// @return Status/error codes [0:success, 1:grbl error, 2:timeout].
uint8_t GantryOperation::_grblWrite(const String &cmd_str, unsigned long timeout)
{
	// Write the command with a new line character
	String full_cmd = cmd_str + "\n";
	Serial1.write(full_cmd.c_str());
	delay(100);

	// Check for acknoledgement
	_Dbg.dtTrack(1);
	String ack_str;
	uint8_t status = _grblRead(ack_str, timeout);

	// Print the acknoledgement
	if (status == 0)
		_Dbg.printMsg(_Dbg.MT::INFO, "[_grblWrite] Acknoledgement recived: cmd_str[%s] dt[%s]", cmd_str.c_str(), _Dbg.dtTrack());

	// Return status from read
	return status;
}

/// @brief: Read the grbl response from the serial buffer.
///
/// @param resonse: The string to store the grbl response.
/// @param timeout: Timeout for the grbl response.
///
/// @return Status/error codes [0:response, 1:grbl error, 2:timeout].
uint8_t GantryOperation::_grblRead(String &resonse_str, unsigned long timeout)
{
	// Check for new message
	unsigned long start_time = millis(); // start time

	// Read from the Serial1 buffer
	while (Serial1.available() > 0 || millis() - start_time < timeout)
	{
		// Read a byte from the Serial buffer
		char new_bite = Serial1.read();

		// Check for carriage return character
		if (new_bite == '\r')
		{
			continue;
		}

		// Check for new line character
		if (new_bite == '\n')
		{
			break;
		}

		// Append the byte to the String
		resonse_str += new_bite;
	}

	// Check for timeout
	if (millis() - start_time > timeout)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[_grblWrite] Timeout: resonse_str[%s]", resonse_str.c_str());
		return 2;
	}

	// Check for error message
	if (resonse_str == "error")
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[_grblRead] Error message: resonse_str[%s]", resonse_str.c_str());
		return 1;
	}

	// Return message received
	return 0;
}

/// @brief Initialize the grbl settings.
void GantryOperation::grblInit()
{
	_Dbg.printMsg(_Dbg.MT::HEAD1A, "START: GRBL INITIALIZATION");

	// Set Units (mm)
	if (_grblWrite("G21") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInit] Error setting units");
	}

	// Set Mode (G90 = Absolute, G91 = Relative)
	if (_grblWrite("G90") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInit] Error setting absolute mode");
	}

	// Feed Rate (mm/min)
	if (_grblWrite("F25000") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInit] Error setting feed rate");
	}

	_Dbg.printMsg(_Dbg.MT::HEAD1B, "FINISHED: GRBL INITIALIZATION");
}

/// @brief Home the gantry.
void GantryOperation::gantryHome()
{
	_Dbg.printMsg(_Dbg.MT::HEAD1A, "START: GANTRY HOMING");

	// Set the homing seek speed to 5000 mm/min
	if (_grblWrite("$25=5000") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryHome] Error setting homing seek speed");
	}

	// Start the homing cycle
	if (_grblWrite("$H", 60000) != 0) // allow for a longer timeout (60 sec)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryHome] Error starting homing cycle");
	}

	// Set the current position as the origin (0,0,0) for the coordinate system
	if (_grblWrite("G10 P0 L20 X0 Y0 Z0") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryHome] Error setting origin");
	}

	_Dbg.printMsg(_Dbg.MT::HEAD1B, "FINISHED: GANTRY HOMING");
}

/// @brief Move the gantry to the target coordinates.
void GantryOperation::gantryMove(uint16_t x, uint16_t y)
{
	char buffer[50]; // Buffer to hold the formatted string
	sprintf(buffer, "$J=G91 G21 X%d.0 Y%d.0 F25000", x, y);
	String cmd_str = String(buffer);
	if (_grblWrite(cmd_str) != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryMove] Error moving to target coordinates");
	}
}

/// @brief Used to process new ROS ethercat msg argument data.
void GantryOperation::procEcatMessage()
{
	uint8_t msg_arg_arr[9]; // store message arguments
	uint8_t arg_len = 0;	// store argument length

	// Check for new message
	if (!EsmaCom.rcvEM.isNew)
		return;

	// Copy and send back recieved message arguments as the default response
	arg_len = EsmaCom.rcvEM.argLen;
	for (size_t arg_i = 0; arg_i < arg_len; arg_i++)
		msg_arg_arr[arg_i] = EsmaCom.rcvEM.ArgU.ui8[arg_i];

	_Dbg.printMsg(_Dbg.MT::INFO, "(%d)ECAT PROCESSING: %s", EsmaCom.rcvEM.msgID, EsmaCom.rcvEM.msg_tp_str);

	//............... Process and Execute Messages ...............

	// HANDSHAKE
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::HANDSHAKE)
	{
		// Initialize ecat message variables
		EsmaCom.initEcat(true);
		_Dbg.printMsg(_Dbg.MT::INFO, "[HANDSHAKE] Initializing ecat message variables");
	}

	// GANTRY_INITIALIZE_GRBL
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_INITIALIZE_GRBL)
	{
		grblInit();
		_Dbg.printMsg(_Dbg.MT::INFO, "[GANTRY_INITIALIZE_GRBL] Initializing grbl");
	}

	// GANTRY_HOME
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_HOME)
	{
		gantryHome();
	}

	// GANTRY_MOVE_REL
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_MOVE_REL)
	{
		/// HACK: To correct the
		uint16_t x = EsmaCom.rcvEM.ArgU.ui16[0]; // get the x position
		uint16_t y = EsmaCom.rcvEM.ArgU.ui16[1]; // get the y position
		gantryMove(x, y);
	}

	// GANTRY_SET_FEEDER
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_SET_FEEDER)
	{
		uint8_t move_dir = EsmaCom.rcvEM.ArgU.ui8[0]; // get the move direction
		feederMove(move_dir);
	}

	// GANTRY_RUN_PUMP
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_RUN_PUMP)
	{
		uint8_t run_state = EsmaCom.rcvEM.ArgU.ui8[0]; // get the run state
		pumpRun(run_state);
	}

	//............... Send Ecat Ack ...............

	// NO ERROR
	EsmaCom.writeEcatAck(EsmaCom.ErrorType::ERR_NONE, msg_arg_arr, arg_len); // send back recieved message arguments
}

void GantryOperation::debugPrintSerialChars()
{
	while (Serial1.available() > 0)
	{
		// Read a byte from the Serial1 buffer
		char incomingByte = Serial1.read();

		// Print the character with explicit handling for special characters
		if (incomingByte == '\n')
		{
			Serial.print("[newline]");
		}
		else if (incomingByte == '\r')
		{
			Serial.print("[carriage return]");
		}
		else if (incomingByte == '\t')
		{
			Serial.print("[tab]");
		}
		else if (incomingByte == ' ')
		{
			Serial.print("[space]");
		}
		else
		{
			Serial.print(incomingByte);
		}
	}
}

/// @brief Initialize the servo objects.
void GantryOperation::servoInit()
{
	// Setup the port servo
	portServo.attach(portServoPin); // Attach the servo object to the pwm pin
	portServo.write(portUpAngle);	// Set the servo to the up position

	// Setup the pump servo
	pumpServo.attach(pumpServoPin); // Attach the servo object to the pwm pin
}

/// @brief Lower or raise the feeder.
///
/// @param move_dir: Direction to move the feeder [0:lower, 1:raise].
void GantryOperation::feederMove(uint8_t move_dir)
{
	if (move_dir == 1)
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "[feederMove] Lowering the feeder");
		portServo.write(portDownAngle);
	}
	else
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "[feederMove] Raising the feeder");
		portServo.write(portUpAngle);
	}
}

/// @brief Start or stop the pump.
///
/// @param run_state: State to run the pump [0:stop, 1:run].
void GantryOperation::pumpRun(uint8_t run_state)
{
	if (run_state == 1)
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "[pumpRun] Running the pump");
		pumpServo.write(pumpRunSpeed);
	}
	else
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "[pumpRun] Stopping the pump");
		pumpServo.write(pumpStopSpeed);
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
// 			std::string ack_str = ser.readline();
// 			if (ack_str == "ok\n")
// 			{
// 				// std::cout << ack_str << std::endl;
// 				break;
// 			}
// 		}
// 	}
// 	catch (const std::exception &e)
// 	{
// 		std::cerr << "Gcode commands must be a string" << std::endl;
// 	}
// }
