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
/// @param do_wait_ack: Flag to wait for grbl acknoledgement.
/// @param timeout: Timeout for the grbl acknoledgement.
///
/// @return Status/error codes [0:success, 1:grbl error, 2:timeout].
uint8_t GantryOperation::grblWrite(const String &cmd_str, bool do_wait_ack, unsigned long timeout)
{
	// Write the command with a new line character
	String full_cmd = cmd_str + "\n";
	Serial1.write(full_cmd.c_str());

	// Bail if no read is needed
	if (!do_wait_ack)
		return 0;

	// Check for acknoledgement
	_Dbg.dtTrack(1);
	String ack_str;
	uint8_t status = grblRead(ack_str, timeout);
	// uint8_t status = 0;

	// Print the acknoledgement
	if (status == 0)
		_Dbg.printMsg(_Dbg.MT::INFO, "[grblWrite] Ack recived: cmd_str[%s] dt[%s]", cmd_str.c_str(), _Dbg.dtTrack());
	else
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblWrite] No Ack recived: cmd_str[%s] dt[%s]", cmd_str.c_str(), _Dbg.dtTrack());

	// Return status from read
	return status;
}

/// @brief: Read the grbl response from the serial buffer.
///
/// @param resonse: The string to store the grbl response.
/// @param timeout: Timeout for the grbl response.
/// @param do_print_response: Flag to print the response.
///
/// @return Status/error codes [0:response, 1:grbl error, 2:timeout].
uint8_t GantryOperation::grblRead(String &resonse_str, unsigned long timeout, bool do_print_response)
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
	if (millis() - start_time >= timeout)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblWrite] Timeodut");
		return 2;
	}

	// Check for error message
	if (resonse_str == "error")
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblRead] Error message");
		return 1;
	}

	// Print response
	if (do_print_response)
		_Dbg.printMsg(_Dbg.MT::INFO, "[grblRead] Response: \r\n%s", resonse_str);

	// Return message received
	return 0;
}

/// @brief Initialize the grbl system settings.
/// @brief Initialize the grbl system settings.
void GantryOperation::grblInitSystem()
{
	// Loop through the settings array
	for (size_t i = 0; i < sizeof(_grblSettings) / sizeof(_grblSettings[0]); ++i)
	{
		// Convert the float value to a string
		String float_str = String(_grblSettings[i].value, 3); // Format the value with 3 decimal places

		// Format the GRBL command string
		String command_str = _grblSettings[i].command + String("=") + float_str;

		// Send the command using grblWrite
		if (grblWrite(command_str.c_str()) != 0)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInitSystem] Error for GRBL commands: %s", command_str.c_str());
		}
		else
		{
			_Dbg.printMsg(_Dbg.MT::INFO, "[grblInitSystem] GRBL command sent: %s", command_str.c_str());
		}
	}

	_Dbg.printMsg(_Dbg.MT::ATTN, "FINISHED: GRBL SYSTEM INITIALIZATION");
}

/// @brief Initialize the grbl settings for this session.
void GantryOperation::grblInitRuntime(float max_feed_rate, float max_acceleration)
{
	_Dbg.printMsg(_Dbg.MT::ATTN, "START: GRBL INITIALIZATION");

	// Set Units (mm)
	if (grblWrite("G21") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInitRuntime] Error setting units");
	}

	// Set Mode to relative (G90 = Absolute, G91 = Relative)
	if (grblWrite("G91") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInitRuntime] Error setting relative mode");
	}

	// Feed Rate (mm/min)
	String fr_cmd = "F" + String((long)max_feed_rate);
	if (grblWrite(fr_cmd.c_str()) != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInitRuntime] Error setting feed rate");
	}
	else
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "[grblInitRuntime] Feed rate set to: %d", (long)max_feed_rate);
	}

	// Acceleration (mm/sec²) for X axis
	String acc_x_cmd = "$120=" + String((long)max_acceleration);
	if (grblWrite(acc_x_cmd.c_str()) != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInitRuntime] Error setting X-axis acceleration");
	}
	else
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "[grblInitRuntime] X-axis acceleration set to: %d", (long)max_acceleration);
	}

	// Acceleration (mm/sec²) for Y axis
	String acc_y_cmd = "$121=" + String((long)max_acceleration);
	if (grblWrite(acc_y_cmd.c_str()) != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblInitRuntime] Error setting Y-axis acceleration");
	}
	else
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "[grblInitRuntime] Y-axis acceleration set to: %d", (long)max_acceleration);
	}

	_Dbg.printMsg(_Dbg.MT::ATTN, "FINISHED: GRBL RUNTIME INITIALIZATION");
}

/// @brief Home the gantry.
void GantryOperation::gantryHome(uint16_t home_speed)
{
	_Dbg.printMsg(_Dbg.MT::ATTN, "GANTRY HOMING: speed[%d]", home_speed);

	// Set the homing seek speed
	String home_speed_cmd = "$25=" + String(home_speed);
	if (grblWrite(home_speed_cmd) != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryHome] Error setting homing seek speed");
	}

	// Start the homing cycle
	if (grblWrite("$H") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryHome] Error starting homing cycle");
	}

	// Set the current position as the origin (0,0,0) for the coordinate system
	if (grblWrite("G10 P0 L20 X0 Y0 Z0") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryHome] Error setting origin");
	}
}

/// @brief Move the gantry to the target coordinates.
void GantryOperation::gantryMove(float x, float y, float max_feed_rate)
{
	// Convert float values to String with 2 decimal places
	String x_str = String(x, 2);
	String y_str = String(y, 2);
	String fr_str = "F" + String((long)max_feed_rate);

	// Format the jog command string using Strings
	String cmd_str = "$J=G91 G21 X" + x_str + " Y" + y_str + " " + fr_str;

	// Send the jog command
	if (grblWrite(cmd_str) != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[gantryMove] Error moving to target coordinates");
	}

	// TEMP print the command string
	_Dbg.printMsg(_Dbg.MT::INFO, "[gantryMove] Command string: %s", cmd_str.c_str());
}


void GantryOperation::grblJogCancel()
{
	// Send the jog cancel command
	if (grblWrite("" + char(0x85)) != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblJogCancel] Error canceling jog");
	}
}

void GantryOperation::grblResetAlarm()
{
	// Send the reset alarm command
	if (grblWrite("$X") != 0)
	{
		_Dbg.printMsg(_Dbg.MT::ERROR, "[grblResetAlarm] Error resetting alarm");
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
	}

	// GANTRY_INITIALIZE_GRBL
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_INITIALIZE_GRBL)
	{
		float max_feed_rate = EsmaCom.rcvEM.ArgU.f32[0];	// get the max feed rate
		float max_acceleration = EsmaCom.rcvEM.ArgU.f32[1]; // get the max acceleration
		grblInitSystem();
		grblInitRuntime(max_feed_rate, max_acceleration);
		// Store mzx feed rate
		maxFeedRate = max_feed_rate;
	}

	// GANTRY_HOME
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_HOME)
	{
		uint16_t home_speed = EsmaCom.rcvEM.ArgU.ui16[0]; // get the homing speed
		gantryHome(home_speed);
	}

	// GANTRY_MOVE_REL
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_MOVE_REL)
	{
		float x = EsmaCom.rcvEM.ArgU.f32[0];			 // get the x position
		float y = EsmaCom.rcvEM.ArgU.f32[1];			 // get the y position
		gantryMove(x, y, maxFeedRate);
	}

	// GANTRY_JOG_CANCEL
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_JOG_CANCEL)
	{
		grblJogCancel();
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

	// GANTRY_REWARD
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::GANTRY_REWARD)
	{
		float duration = EsmaCom.rcvEM.ArgU.f32[0] * 1000; // get the reward durration in ms
		runReward(duration);
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
/// @param move_dir: Direction to move the feeder [0:raise, 1:lower].
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

/// @brief Start or stop the pump.
///
/// @param diration: Duration to run the pump (ms).
///
void GantryOperation::runReward(float diration)
{
	// Lower the feeder
	feederMove(1);
	delay(250);

	// Start the pump
	pumpRun(1);

	// Wait for the duration
	delay(diration);

	// Stop the pump
	pumpRun(0);

	// Raise the feeder
	feederMove(0);
}
