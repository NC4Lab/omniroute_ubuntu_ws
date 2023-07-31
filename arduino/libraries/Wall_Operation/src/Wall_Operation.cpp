// ######################################

//======== Wall_Operation.cpp ==========

// ######################################

/// <file>
/// Used for the Wall_Operation class
/// <file>

//============= INCLUDE ================
#include "Wall_Operation.h"

//======== CLASS: Wall_Operation ==========

/// <summary>
/// CONSTUCTOR: Create Wall_Operation class instance
/// </summary>
/// <param name="_nCham">Spcify number of chambers to track [1-49]</param>
/// <param name="do_spi">OPTIONAL: spcify if SPI should be strated, will interfere with LiquidCrystal library</param>
Wall_Operation::Wall_Operation(uint8_t _nCham, uint8_t _pwmDuty, uint8_t do_spi)
{
	nCham = _nCham;									  // store number of chambers
	pwmDuty = _pwmDuty;								  // store pwm duty cycle
	for (size_t cham_i = 0; cham_i < nCham; cham_i++) // update chamber struct entries
	{
		C[cham_i].num = cham_i;
		C[cham_i].addr = _C_COM.ADDR_LIST[cham_i];
		// Start SPI for Ethercat
		if (do_spi)
			ESMA.start_spi();
	}
	// Create WallMapStruct lists for each function
	_makePMS(pmsAllIO, wms.ioDown[0], wms.ioDown[1], wms.ioUp[0], wms.ioUp[1]);		 // all io pins
	_makePMS(pmsAllPWM, wms.pwmDown[0], wms.pwmDown[1], wms.pwmUp[0], wms.pwmUp[1]); // all pwm pins
	_makePMS(pmsUpIO, wms.ioUp[0], wms.ioUp[1]);									 // io up pins
	_makePMS(pmsDownIO, wms.ioDown[0], wms.ioDown[1]);								 // io down pins
	_makePMS(pmsUpPWM, wms.pwmUp[0], wms.pwmUp[1]);									 // pwm up pins
	_makePMS(pmsDownPWM, wms.pwmDown[0], wms.pwmDown[1]);							 // pwm down pins
	// Update pin function map [0,1,2,3] [io down, io up, pwm down, pwm up]
	for (size_t i = 0; i < 8; i++)
	{														  // loop wall map entries
		wms.funMap[wms.ioDown[0][i]][wms.ioDown[1][i]] = 1;	  // label io down
		wms.funMap[wms.ioUp[0][i]][wms.ioUp[1][i]] = 2;		  // label io up
		wms.funMap[wms.pwmDown[0][i]][wms.pwmDown[1][i]] = 3; // label pwm down
		wms.funMap[wms.pwmUp[0][i]][wms.pwmUp[1][i]] = 4;	  // label pwm up
	}
}

//++++++++++++ Data Handeling +++++++++++++

/// <summary>
/// Build @ref Wall_Operation::PinMapStruct (PMS) structs, which are used to store information
/// related to pin/port and wall mapping for specified functions (i.e., pwm, io, up, down)
/// Note, these methods are only used in the construtor
/// </summary>
/// <param name="r_pms">Reference to PMS to be updated</param>
/// <param name="p_port_1">Array of port values from an @ref Wall_Operation::WallMapStruct </param>
/// <param name="p_pin_1">Array of pin values from an @ref Wall_Operation::WallMapStruct</param>
void Wall_Operation::_makePMS(PinMapStr &r_pms, uint8_t p_port_1[], uint8_t p_pin_1[])
{
	_resetPMS(r_pms);
	_addPortPMS(r_pms, p_port_1, p_pin_1);
	_addPinPMS(r_pms, p_port_1, p_pin_1);
}
/// <summary>
/// OVERLOAD:  with option for additional @ref Wall_Operation::WallMapStruct entries used
/// for creating PMS structs that include pins both up and down (e.g., all IO or all PWM pins)
/// </summary>
/// <param name="p_port_2">Array of port values from an @ref Wall_Operation::WallMapStruct </param>
/// <param name="p_pin_2">Array of pin values from an @ref Wall_Operation::WallMapStruct</param>
void Wall_Operation::_makePMS(PinMapStr &r_pms, uint8_t p_port_1[], uint8_t p_pin_1[], uint8_t p_port_2[], uint8_t p_pin_2[])
{
	_resetPMS(r_pms);
	_addPortPMS(r_pms, p_port_1, p_pin_1);
	_addPortPMS(r_pms, p_port_2, p_pin_2);
	_addPinPMS(r_pms, p_port_1, p_pin_1);
	_addPinPMS(r_pms, p_port_2, p_pin_2);
}

/// <summary>
/// Used within @ref Wall_Operation::_makePMS to do the actual work of adding the pin entries to the PMS structs.
/// </summary>
/// <param name="r_pms">Reference to PMS to be updated</param>
/// <param name="p_port">Array of port values from an @ref Wall_Operation::WallMapStruct </param>
/// <param name="p_pin">Array of pin values from an @ref Wall_Operation::WallMapStruct</param>
void Wall_Operation::_addPortPMS(PinMapStr &r_pms, uint8_t p_port[], uint8_t p_pin[])
{

	for (size_t wal_i = 0; wal_i < 8; wal_i++)
	{ // loop port list by wall
		for (size_t prt_i = 0; prt_i < 6; prt_i++)
		{ // loop port array in struct
			if (r_pms.port[prt_i] != p_port[wal_i] && r_pms.port[prt_i] != 255)
				continue;														// find first emtpy  or existing entry and store there
			r_pms.nPorts = r_pms.port[prt_i] == 255 ? prt_i + 1 : r_pms.nPorts; // update length
			r_pms.port[prt_i] = p_port[wal_i];
			break;
		}
	}
	// Sort array
	_sortArr(r_pms.port, 6);
}

/// <summary>
/// Used within @ref Wall_Operation::_makePMS to do the actual work of adding the port entries to the PMS structs.
/// </summary>
/// <param name="r_pms">Reference to PMS to be updated.</param>
/// <param name="p_port">Array of port values from an @ref Wall_Operation::WallMapStruct.</param>
/// <param name="p_pin">Array of pin values from an @ref Wall_Operation::WallMapStruct.</param>
void Wall_Operation::_addPinPMS(PinMapStr &r_pms, uint8_t p_port[], uint8_t p_pin[])
{
	for (size_t prt_i = 0; prt_i < 6; prt_i++)
	{ // loop ports in struct arr
		if (r_pms.port[prt_i] == 255)
			break; // bail if reached end of list
		for (size_t wal_i = 0; wal_i < 8; wal_i++)
		{ // loop wall list
			if (p_port[wal_i] != r_pms.port[prt_i])
				continue; // check port match
			for (size_t pin_ii = 0; pin_ii < 8; pin_ii++)
			{ // loop pin struct
				if (r_pms.pin[prt_i][pin_ii] != 255)
					continue;					 // find first emtpy entry and store there
				r_pms.nPins[prt_i] = pin_ii + 1; // update length
				r_pms.pin[prt_i][pin_ii] = p_pin[wal_i];
				r_pms.wall[prt_i][pin_ii] = wal_i;
				break;
			}
		}
		// Sort pin and wall array based on pin number
		_sortArr(r_pms.pin[prt_i], 8, r_pms.wall[prt_i]);

		// Update registry byte
		for (size_t pin_i = 0; pin_i < r_pms.nPins[prt_i]; pin_i++)
		{
			bitWrite(r_pms.bitMask[prt_i], r_pms.pin[prt_i][pin_i], 1); // update short/truncated version
		}
		r_pms.bitMaskLong[r_pms.port[prt_i]] = r_pms.bitMask[prt_i]; // update long/complete version
	}
}

/// <summary>
/// Sorts array values in assending order
/// Note, this is used exclusively by the above methods when creating the PMS structs.
/// </summary>
/// <param name="p_arr">Array to be sorted</param>
/// <param name="s">Length of array</param>
/// <param name="p_co_arr">OPTIONAL: array to be sorted in the same order as "p_arr"</param>
void Wall_Operation::_sortArr(uint8_t p_arr[], size_t s, uint8_t p_co_arr[])
{
	bool is_sorted = false;
	while (!is_sorted)
	{
		is_sorted = true;

		// Iterate over the array, swapping adjacent elements if they are out of order
		for (size_t i = 0; i < s - 1; ++i)
		{
			if (p_arr[i] > p_arr[i + 1])
			{
				// Swap the elements without using the std::swap function
				uint8_t tmp1 = p_arr[i];
				p_arr[i] = p_arr[i + 1];
				p_arr[i + 1] = tmp1;
				// Update sort ind
				if (p_co_arr)
				{
					uint8_t tmp2 = p_co_arr[i];
					p_co_arr[i] = p_co_arr[i + 1];
					p_co_arr[i + 1] = tmp2;
				}
				is_sorted = false;
			}
		}
	}
}

/// <summary>
/// Resets most entries in a dynamic PMS struct to there default values.
/// </summary>
/// <param name="r_pms">Reference to PMS struct to be reset</param>
void Wall_Operation::_resetPMS(PinMapStr &r_pms)
{
	r_pms.nPorts = 0;
	for (size_t prt_i = 0; prt_i < 6; prt_i++)
	{
		r_pms.port[prt_i] = 255;
		r_pms.bitMask[prt_i] = 0;
		r_pms.bitMaskLong[prt_i] = 0;
		r_pms.nPins[prt_i] = 0;
		for (size_t pin_i = 0; pin_i < 8; pin_i++)
		{
			r_pms.pin[prt_i][pin_i] = 255;
			r_pms.wall[prt_i][pin_i] = 255;
		}
	}
}

/// <summary>
/// Updates dynamic PMS structs based on new wall configuration input.
/// </summary>
/// <param name="r_pms1">PMS struct to use as the basis for entries in "r_pms2"</param>
/// <param name="r_pms2">Reference to a PMS struct to update</param>
/// <param name="wall_byte_mask">Byte mask in which bits set to one denote the active walls to include in the "r_pms2" struct.</param>
void Wall_Operation::_updateDynamicPMS(PinMapStr r_pms1, PinMapStr &r_pms2, uint8_t wall_byte_mask)
{
	for (size_t prt_i = 0; prt_i < r_pms1.nPorts; prt_i++)
	{ // loop ports
		for (size_t pin_i = 0; pin_i < r_pms1.nPins[prt_i]; pin_i++)
		{ // loop pins
			if (bitRead(wall_byte_mask, r_pms1.wall[prt_i][pin_i]) == 1)
			{ // check if wall is on this port

				// Copy all struct fields
				for (size_t prt_ii = 0; prt_ii < 6; prt_ii++)
				{ // loop ports
					if (r_pms2.port[prt_ii] != r_pms1.port[prt_i] && r_pms2.port[prt_ii] != 255)
						continue;																	// find first emtpy  or existing entry and store there
					r_pms2.nPorts = r_pms2.port[prt_ii] == 255 ? r_pms2.nPorts + 1 : r_pms2.nPorts; // update port count for new entry
					r_pms2.nPins[prt_ii]++;															// update pin count
					r_pms2.port[prt_ii] = r_pms1.port[prt_i];										// update port number
					for (size_t pin_ii = 0; pin_ii < 8; pin_ii++)
					{ // loop pins
						if (r_pms2.pin[prt_ii][pin_ii] != 255)
							continue;													  // find first emtpy entry and store there
						r_pms2.pin[prt_ii][pin_ii] = r_pms1.pin[prt_i][pin_i];			  // update pin number
						r_pms2.wall[prt_ii][pin_ii] = r_pms1.wall[prt_i][pin_i];		  // update wall number
						bitWrite(r_pms2.bitMask[prt_ii], r_pms2.pin[prt_ii][pin_ii], 1);  // update short/truncated registry
						r_pms2.bitMaskLong[r_pms2.port[prt_ii]] = r_pms2.bitMask[prt_ii]; // update long/complete version
						break;
					}
					break;
				}
			}
		}
	}
}

//++++++++++++ Setup Methods +++++++++++++

/// <summary>
/// Reset @ref Wall_Operation::ChamberTrackStruct struct flags, which are used to track
/// the wall states and errors
/// <param name="do_full_reset">reset all variables</param>
/// </summary>
void Wall_Operation::resetMaze(uint8_t do_full_reset)
{
	uint8_t resp = 0;

	// Setup cypress chips
	for (size_t ch_i = 0; ch_i < nCham; ch_i++)
	{
		resp = _C_COM.setupCypress(C[ch_i].addr);
		if (resp != 0)
			_DB.printMsgTime("!!ERROR: Failed Cypress setup: chamber=%d address=%s!!", ch_i, _DB.hexStr(C[ch_i].addr));
		if (resp == 0 && ch_i == nCham - 1)
		{ // print success for last itteration
			_DB.printMsgTime("Finished Cypress setup: chamber=%d address=%s", ch_i, _DB.hexStr(C[ch_i].addr));
		}
	}

	// Setup IO pins for each chamber
	resp = _setupCypressIO();

	// Setup PWM pins for each chamber
	resp = _setupCypressPWM(pwmDuty);

	// Reset to starting state
	if (do_full_reset)
	{
		// Reset all chamber flags
		for (size_t cham_i = 0; cham_i < nCham; cham_i++) // update chamber struct entries
		{
			C[cham_i].bitWallMoveFlag = 0;
			C[cham_i].bitWallPosition = 0;
			C[cham_i].bitWallErrorFlag = 0;
			C[cham_i].bitWallUpdateFlag = 0;
			for (size_t i = 0; i < 6; i++)
				C[cham_i].bitOutRegLast[i] = 0;
		}

		// Reset message counters
		rcvMsgID = 0;
		sndMsgID = 0;

		// Reset Ethercat handshake flag
		isHandshakeDone = false;
	}

	// Test and reset all walls
	resp = _setupWalls();

	if (do_full_reset)
		_DB.printMsgTime("RESET Maze Complete");
	else
		_DB.printMsgTime("SETUP Maze Complete");
}

/// /// <summary>
/// Setup IO pins for each chamber including the pin direction (input) a
/// and the type of drive mode (high impedance).
/// Note, have to do some silly stuff with the output pins as well based on
/// page 11 of the Cypress datasheet "To  allow  input  operations  without
/// reconfiguration, these registers [output] have to store �1�s."
/// </summary>
/// <returns>Wire::method output from @ref Cypress_Com::method.</returns>
uint8_t Wall_Operation::_setupCypressIO()
{
	_DB.printMsgTime("Running IO pin setup");
	uint8_t resp = 0;

	// Setup wall io pins
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{
		// Set entire output register to off
		uint8_t p_byte_mask_in[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		resp = _C_COM.ioWriteReg(C[cham_i].addr, p_byte_mask_in, 6, 0);

		if (resp == 0)
		{
			for (size_t prt_i = 0; prt_i < pmsAllIO.nPorts; prt_i++)
			{ // loop through port list
				// Set input pins as input
				resp = _C_COM.setPortRegister(C[cham_i].addr, REG_PIN_DIR, pmsAllIO.port[prt_i], pmsAllIO.bitMask[prt_i], 1);
				if (resp == 0)
				{
					// Set pins as pull down
					resp = _C_COM.setPortRegister(C[cham_i].addr, DRIVE_PULLDOWN, pmsAllIO.port[prt_i], pmsAllIO.bitMask[prt_i], 1);
					if (resp == 0)
					{
						// Set corrisponding output register entries to 1 as per datasheet
						resp = _C_COM.ioWritePort(C[cham_i].addr, pmsAllIO.port[prt_i], pmsAllIO.bitMask[prt_i], 1);
						if (resp != 0)
							return resp;
					}
				}
			}
		}
		if (resp != 0)
			_DB.printMsgTime("\t!!failed IO pin setup: chamber=%d", cham_i);
	}
	_DB.printMsgTime("Finished IO pin setup");
	return resp;
}

/// <summary>
/// Setup PWM pins for each chamber including the specifying them as PWM outputs
/// and also detting the drive mode to "Strong Drive". This also sets up the PWM
/// Source using @ref Cypress_Com::setupSourcePWM()
/// </summary>
/// <param name="pwm_duty">Duty cycle for the pwm [0-255]</param>
/// <returns>Wire::method output from @ref Cypress_Com::method.</returns>
uint8_t Wall_Operation::_setupCypressPWM(uint8_t pwm_duty)
{
	_DB.printMsgTime("Running PWM setup");
	uint8_t resp = 0;

	// Setup pwm
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{
		// Setup source
		for (size_t src_i = 0; src_i < 8; src_i++)
		{
			resp = _C_COM.setupSourcePWM(C[cham_i].addr, wms.pwmSrc[src_i], pwm_duty);
			if (resp != 0)
				_DB.printMsgTime("\t!!failed PWM source setup: source=%d", src_i);
		}

		// Setup wall pwm pins
		for (size_t prt_i = 0; prt_i < pmsAllPWM.nPorts; prt_i++)
		{ // loop through port list
			// Set pwm pins as pwm output
			resp = _C_COM.setPortRegister(C[cham_i].addr, REG_SEL_PWM_PORT_OUT, pmsAllPWM.port[prt_i], pmsAllPWM.bitMask[prt_i], 1);
			if (resp == 0)
			{
				// Set pins as strong drive
				resp = _C_COM.setPortRegister(C[cham_i].addr, DRIVE_STRONG, pmsAllPWM.port[prt_i], pmsAllPWM.bitMask[prt_i], 1);
			}
		}
		if (resp != 0)
			_DB.printMsgTime("\t!!failed PWM pin setup: chamber=%d", cham_i);
	}
	_DB.printMsgTime("Finished PWM setup");
	return resp;
}

/// <summary>
/// Test up and down movment using @ref Wall_Operation::setWallCmdManual()
/// and @ref Wall_Operation::moveWalls()
/// <returns>Success/error codes from @ref Wall_Operation::moveWalls().</returns>
/// </summary>
uint8_t Wall_Operation::_setupWalls()
{
	_DB.printMsgTime("Running wall initialization");

	// Run all walls up then down for each chamber
	uint8_t resp = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{

		// Run walls up
		setWallCmdManual(cham_i, 1);
		resp = moveWalls(); // move walls up

		// Run walls down
		setWallCmdManual(cham_i, 0);
		resp = moveWalls(); // move walls down

		if (resp != 0)
			_DB.printMsgTime("\t!!failed wall initialization: chamber=%d", cham_i);
	}

	// Set resp to any non-zero flag and return
	_DB.printMsgTime("Finished wall initialization");
	return resp;
}

//++++++++++++++ Comms Methods +++++++++++++++

/// <summary>
/// Used to send outgoing ROS ethercat msg data signalling which walls to raise.
/// </summary>
/// </remarks>
///	The outgoing register is structured arr[8] of 16 bit integers
///	with all but first 16 bit value seperated into bytes
///	i16[0]: Message ID [0-65535]
///	i16[1]: Message Info
///		i8[0] message type [0-255] [see: MessageTypeID]
///		i8[1] arg length [0-10] [number of message args in bytes]
///	i16[NA,2:6] Arguments
///		i16[2-3] message confirmation
///			i16[2] rcv message id [0-65535]
///			i16[2] rcv message type [0-65535]
///	i16[x+1]: Footer
///		i8[0] [254]
///		i8[1] [254]
/// </remarks>
/// <param name="snd_msg_type">The type of the message to be sent.</param>
/// <param name="p_msg_arg_data">OPTIONAL: The data for the message arguments. DEFAULT: nullptr.</param>
/// <param name="msg_arg_lng">OPTIONAL: The length of the message arguments in uint8. DEFAULT: 0.</param>
/// <returns>Success/error codes [0:no error, 1:error]</returns>
void Wall_Operation::sendEthercatMessage(MessageType snd_msg_type, uint8_t p_msg_arg_data[], uint8_t msg_arg_lng)
{
	// Itterate message number id and roll over to 1 if max 16 bit value is reached
	sndMsgID = sndMsgID < 65535 ? sndMsgID + 1 : 1;

	// Clear union
	U.ui64[0] = 0;
	U.ui64[1] = 0;

	// Add shared message data
	U.ui16[0] = sndMsgID;						   // send message id
	U.ui8[2] = static_cast<uint8_t>(snd_msg_type); // send message type

	// HANDLE MESSAGE TYPE

	// 	CONFIRM_RECIEVED
	if (snd_msg_type == MessageType::CONFIRM_RECIEVED)
	{
		_DB.setGetStr("CONFIRM_RECIEVED");
		msg_arg_lng = 2;
		U.ui16[3] = rcvMsgID; // recieved message number
	}
	// 	HANDSHAKE
	else if (snd_msg_type == MessageType::HANDSHAKE)
	{
		_DB.setGetStr("HANDSHAKE");
		msg_arg_lng = 0;
	}
	// 	ERROR
	else if (snd_msg_type == MessageType::ERROR)
	{
		_DB.setGetStr("ERROR");
		msg_arg_lng = 2;
		U.ui16[3] = ErrorType::ERROR_NONE; // error type
	}

	// Add message argument length
	U.ui8[3] = msg_arg_lng;

	// Round msg_arg_lng up to even number and add 4 for id and msg info
	uint8_t ui8_i = 4 + ((msg_arg_lng % 2 == 0) ? msg_arg_lng : (msg_arg_lng + 1));

	// Add footer
	U.ui8[ui8_i] = 254;
	U.ui8[ui8_i + 1] = 254;

	// Send message
	for (size_t i = 0; i < 8; i++)
		ESMA.write_reg_value(i, U.ui16[i]);
	_DB.printMsgTime("SENT Ecat Message: type=%s id=%d", _DB.setGetStr(), sndMsgID);

	// Print message
	_DB.printMsgTime("\tui16[0] %d", U.ui16[0]);
	printEcat(0, U);
}

/// <summary>
/// Used to get incoming ROS ethercat msg data.
/// </summary>
/// <returns>Success/error codes [0:ne message, 1:no message, 2:error]</returns>
uint8_t Wall_Operation::getEthercatMessage()
{
	int rcv_msg_id;			  // incoming msg id number
	uint8_t rcv_msg_type;	  // incoming msg type id
	uint8_t msg_arg_lng;	  // incoming msg argument length
	uint8_t msg_arg_data[10]; // uint16_t[5] devided into uinit8-t[10]
	int reg_dat[8];			  // buffer for reading ethercat registers
	uint8_t reg_i = 0;		  // index for reading ethercat registers

	// Read esmacat buffer
	ESMA.get_ecat_registers(reg_dat);

	// Check first register entry for msg id
	rcv_msg_id = reg_dat[reg_i++];
	U.ui16[0] = reg_dat[reg_i++];
	rcv_msg_type = U.ui8[0];
	msg_arg_lng = U.ui8[1];

	// Skip ethercat setup junk (65535)
	if (rcv_msg_id == 65535)
		return 1;

	// Skip redundant messages
	if (rcv_msg_id == rcvMsgID)
		return 1;

	// Make type strings for all MessageType enum values
	if (rcv_msg_type == MessageType::CONFIRM_RECIEVED)
		_DB.setGetStr("CONFIRM_RECIEVED");
	if (rcv_msg_type == MessageType::HANDSHAKE)
		_DB.setGetStr("HANDSHAKE");
	if (rcv_msg_type == MessageType::MOVE_WALLS)
		_DB.setGetStr("MOVE_WALLS");
	if (rcv_msg_type == MessageType::START_SESSION)
		_DB.setGetStr("START_SESSION");
	if (rcv_msg_type == MessageType::END_SESSION) /*  */
		_DB.setGetStr("END_SESSION");

	// Check if rcv_msg_type matches any of the enum values
	if (rcv_msg_type != MessageType::CONFIRM_RECIEVED &&
		rcv_msg_type != MessageType::HANDSHAKE &&
		rcv_msg_type != MessageType::MOVE_WALLS &&
		rcv_msg_type != MessageType::START_SESSION &&
		rcv_msg_type != MessageType::END_SESSION)
	{
		if (rcvErrTp != ErrorType::NO_MESSAGE_TYPE_MATCH) // only run once
		{
			// Set id last to new value on first error and set error type
			rcvErrTp = ErrorType::NO_MESSAGE_TYPE_MATCH;
			_DB.printMsgTime("!!ERROR: Ecat No Type Match: val=%d id=%d!!", rcv_msg_type, rcv_msg_id);
		}
		return 2; // return error flag
	}
	else
		rcvErrTp = ErrorType::ERROR_NONE; // reset error type

	// // TEMP
	// _DB.printMsgTime("Ether Type=%s id= %d", _DB.setGetStr(), rcv_msg_id);
	// printEcat(0, reg_dat);

	// Check for skipped or out of sequence messages
	if (rcv_msg_id - rcvMsgID != 1)
	{
		if (isHandshakeDone)
		{
			if (rcvErrTp != ErrorType::MESSAGE_ID_DISORDERED) // only run once
			{
				// Set id last to new value on first error and set error type
				rcvErrTp = ErrorType::MESSAGE_ID_DISORDERED;
				_DB.printMsgTime("!!ERROR: Ecat ID Missmatch: old=%d new=%d!!", rcvMsgID, rcv_msg_id);
			}
		}
		return 2; // return error flag
	}
	else
		rcvErrTp = ErrorType::ERROR_NONE; // reset error type

	// Check if message is preceding handshake
	if (!isHandshakeDone && rcv_msg_type != MessageType::HANDSHAKE)
	{
		if (rcvErrTp != ErrorType::REGISTER_LEFTOVERS) // only run once
		{
			// Set id last to new value on first error and set error type
			rcvErrTp = ErrorType::REGISTER_LEFTOVERS;
			_DB.printMsgTime("!!ERROR: Ecat Missed Handshake: type=%s id=%d!!", _DB.setGetStr(), rcv_msg_id);
		}
		return 2; // return error flag
	}
	else
		rcvErrTp = ErrorType::ERROR_NONE; // reset error type

	// Update message id
	rcvMsgID = rcv_msg_id;

	// Update dynamic enum instance
	rcvMsgTp = static_cast<Wall_Operation::MessageType>(rcv_msg_type);

	// Parse 8 bit message arguments
	if (msg_arg_lng > 0)
	{
		// Loop through buffer
		uint8_t msg_arg_lng_i16_round = ((int)msg_arg_lng + 1) / 2; // devide message length by 2 and round up
		uint8_t ui8_i = 0;
		for (size_t ui16_i = 0; ui16_i < msg_arg_lng_i16_round; ui16_i++)
		{
			// Get next entry
			U.ui16[0] = reg_dat[reg_i++];

			// Loop through bytes in given 16 bit entry and store
			for (size_t b_ii = 0; b_ii < 2; b_ii++)
				msg_arg_data[ui8_i++] = U.ui8[b_ii];
		}
	}

	// Check for footer
	U.ui16[0] = reg_dat[reg_i++];
	if (U.ui8[0] != 254 && U.ui8[1] != 254)
	{
		_DB.printMsgTime("\t!!Missing message footer!!");
		rcvErrTp = ErrorType::MISSING_FOOTER;
		return 1;
	}

	// 	Process wall data
	if (rcvMsgTp == MessageType::MOVE_WALLS)
	{

		// Loop through arguments
		for (size_t cham_i = 0; cham_i < msg_arg_lng; cham_i++)
		{
			uint8_t wall_b = msg_arg_data[cham_i];

			uint8_t wall_u_b = ~C[cham_i].bitWallPosition & wall_b; // get walls to move up
			uint8_t wall_d_b = C[cham_i].bitWallPosition & ~wall_b; // move down any unasigned walls

			// Update move flag
			C[cham_i].bitWallMoveFlag = wall_u_b | wall_d_b; // store values in bit flag

			// Print walls set to be moved
			_DB.printMsgTime("\t\tset move walls: chamber=%d", cham_i);
			_DB.printMsgTime("\t\t\tup=%s", _DB.bitIndStr(wall_u_b));
			_DB.printMsgTime("\t\t\tdown=%s", _DB.bitIndStr(wall_d_b));
		}
	}
	_DB.printMsgTime("RECIEVED Ecat Message: type=%s id=%d", _DB.setGetStr(), rcvMsgID);

	// Return new message flag
	return 0;
}

/// <summary>
/// Used to process incoming ROS Ethercat msg data.
/// </summary>
void Wall_Operation::executeEthercatCommand()
{

	// Handle message type
	if (rcvMsgTp == MessageType::HANDSHAKE)
	{
		// Set ethercat flag
		_DB.printMsgTime("\tEcat Comms Established");
		isHandshakeDone = true;
	}
	else if (rcvMsgTp == MessageType::MOVE_WALLS)
	{
		moveWalls();
	}
	else if (rcvMsgTp == MessageType::START_SESSION)
	{
		resetMaze(false); // dont reset certain variables
	}
	else if (rcvMsgTp == MessageType::END_SESSION)
	{
		resetMaze(true); // reset everything
	}

	// // Reset rcvMsgTp
	// rcvMsgTp = MessageType::MSG_NONE;
}


//+++++++++++++ Runtime Methods ++++++++++++++

/// <summary>
/// Option to speicify a given set of walls to move programmatically as an
/// alternative to @ref Wall_Operation::getWallCmdSerial.
/// Note, this function updates the byte mask specifying which walls should be up
/// </summary>
/// <param name="cham_i"> Index of the chamber to set [0-48]</param>
/// <param name="bit_val_set"> Value to set the bits to [0,1]. DEFAULT[1].</param>
/// <param name="p_wall_inc">OPTIONAL: Pointer array specifying wall numbers for walls to move [0-7] max 8 entries. DEFAULT:[all walls]</param>
/// <param name="s">OPTIONAL: Length of "p_wall_inc". DEFAULT:[8] </param>
/// <returns>Success/error codes [0:success, -1=255:input argument error]</returns>
/// <example>
/// Here's an example of how to use setWallCmdManual:
/// <code>
/// cham_i = 0; //index of the chamber
/// uint8_t bit_val_set = 1; //set walls to move up
/// uint8_t s = 3; //number of walls
/// uint8_t p_wall_inc[s] = {0, 2, 5}; //array with wall numbers to move
/// Wall_Operation::setWallCmdManual(cham_i, bit_val_set, p_wall_inc, s);
/// Wall_Operation::setWallCmdManual(3, p_wall_inc, s); //can be run more than once to setup multiple chambers with different wall configurations
/// uint8_t bit_val_set = 0; //set walls not to move up, which will result in them being moved down
/// Wall_Operation::setWallCmdManual(cham_i, 0, p_wall_inc, s); //this will lower all the walls specified in "p_wall_inc"
/// Wall_Operation::W_OPR.setWallCmdManual(0, 0); //this will lower all walls in chamber 0
/// Wall_Operation::moveWalls(); //note that @ref Wall_Operation::moveWalls() must be run after all settings complete
/// </code>
/// </example>
uint8_t Wall_Operation::setWallCmdManual(uint8_t cham_i, uint8_t bit_val_set, uint8_t p_wall_inc[], uint8_t s)
{
	if (s > 8)
		return -1;
	uint8_t p_wi[s]; // initialize array to handle null array argument
	if (p_wall_inc == nullptr)
	{ // set default 8 walls
		for (size_t i = 0; i < s; i++)
			p_wi[i] = i;
	}
	else
	{ // copy over input
		for (size_t i = 0; i < s; i++)
			p_wi[i] = p_wall_inc[i];
	}
	uint8_t wall_b; // store wall settings
	for (size_t i = 0; i < s; i++)
	{											// loop through each entry
		bitWrite(wall_b, p_wi[i], bit_val_set); // set bit for wall to be moved
	}

	// Update chamber/wall info
	uint8_t wall_u_b = ~C[cham_i].bitWallPosition & wall_b; // get walls to move up
	uint8_t wall_d_b = C[cham_i].bitWallPosition & ~wall_b; // get walls to move down
	C[cham_i].bitWallMoveFlag = wall_u_b | wall_d_b;		// store values in bit flag

	_DB.printMsgTime("\tRunning manual wall move command", cham_i, _DB.arrayStr(p_wi, s));
	_DB.printMsgTime("\t\tset move walls: chamber=%d", cham_i);
	_DB.printMsgTime("\t\t\tup=%s", _DB.bitIndStr(wall_u_b));
	_DB.printMsgTime("\t\t\tdown=%s", _DB.bitIndStr(wall_d_b));

	return 0;
}

/// <summary>
/// Option to change the PWM duty cycle for a given wall.
/// Note, this is basically a wrapper for @ref Cypress_Com::setSourceDutyPWM.
/// Could be useful if some walls are running at different speeds.
/// </summary>
/// <param name="cham_i">Index of the chamber to set [0-48]</param>
/// <param name="source">Specifies one of 8 sources to set. See @ref Wall_Operation::wms.pwmSrc.</param>
/// <param name="duty">PWM duty cycle [0-255].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Wall_Operation::changeWallDutyPWM(uint8_t cham_i, uint8_t wall_i, uint8_t duty)
{
	if (cham_i > 48 || wall_i > 7 || duty > 255)
		return -1;
	uint8_t resp = _C_COM.setSourceDutyPWM(C[cham_i].addr, wms.pwmSrc[wall_i], duty); // set duty cycle to duty
	return resp;
}

/// <summary>
/// The main workhorse of the class, which mannages initiating and compleating
/// the wall movement for all active chambers based on either
/// @ref Wall_Operation::getWallCmdSerial or @ref Wall_Operation::getWallCmdSerial.
/// </summary>
/// <param name="dt_timout">Time to attemp movement (ms) DEFAULT:[1000]</param>
/// <returns>Success/error codes [0:success, 1:fail:unspecified, 2:fail:i2c, 3:fail:timeout]</returns>
uint8_t Wall_Operation::moveWalls(uint32_t dt_timout)
{
	// Local vars// TEMP COMMIT TEST
	uint8_t resp = 0;
	uint8_t run_error = 0;						// store run errors
	uint8_t is_timedout = 0;					// flag timeout
	uint8_t do_move_check = 1;					// will track if all chamber movement done
	uint32_t ts_timeout = millis() + dt_timout; // set timout

	_DB.printMsgTime("\tMoving walls");

	// TEMP
	return 0;

	// Update dynamic port/pin structs
	_DB.dtTrack(1); // start timer
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{ // loop chambers
		if (C[cham_i].bitWallMoveFlag == 0)
			continue; // check if any walls flagged for updating

		// Reset structs
		_resetPMS(C[cham_i].pmsDynPWM);
		_resetPMS(C[cham_i].pmsDynIO);

		// Get byte mask for walls that should be moved up/down excluding error flagged walls
		uint8_t wall_up_byte_mask = (~C[cham_i].bitWallPosition & C[cham_i].bitWallMoveFlag) & ~C[cham_i].bitWallErrorFlag;	 // bitwise comparison, up (wall_state == 0 & wall_active == 1) & (wall_error = 0)
		uint8_t wall_down_byte_mask = (C[cham_i].bitWallPosition & C[cham_i].bitWallMoveFlag) & ~C[cham_i].bitWallErrorFlag; // bitwise comparison, down (wall_state == 1 & wall_active == 0) & (wall_error = 0)

		// Update dynamic PMS struct
		_updateDynamicPMS(pmsDownIO, C[cham_i].pmsDynIO, wall_down_byte_mask);	 // pwm down
		_updateDynamicPMS(pmsUpIO, C[cham_i].pmsDynIO, wall_up_byte_mask);		 // pwm up
		_updateDynamicPMS(pmsDownPWM, C[cham_i].pmsDynPWM, wall_down_byte_mask); // io down
		_updateDynamicPMS(pmsUpPWM, C[cham_i].pmsDynPWM, wall_up_byte_mask);	 // io up

		// Move walls up/down
		resp = _C_COM.ioWriteReg(C[cham_i].addr, C[cham_i].pmsDynPWM.bitMaskLong, 6, 1);

		// Print walls being moved
		if (wall_up_byte_mask > 0)
			_DB.printMsgTime("\t\tstart move up: chamber=%d walls=%s", cham_i, _DB.bitIndStr(wall_up_byte_mask));
		if (wall_down_byte_mask > 0)
			_DB.printMsgTime("\t\tstart move down: chamber=%d walls=%s", cham_i, _DB.bitIndStr(wall_down_byte_mask));
	}

	// Check IO pins
	while (!is_timedout && do_move_check != 0)
	{										  // loop till finished or timed out
		do_move_check = 0;					  // reset
		is_timedout = millis() >= ts_timeout; // check for timeout
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		{												// loop chambers
			do_move_check += C[cham_i].bitWallMoveFlag; // update flag
			if (C[cham_i].bitWallMoveFlag == 0)
				continue; // check if chamber flagged for updating

			// Get io registry bytes. Note we are reading out the output registry as well to save an additional read if we write the pwm output later
			uint8_t io_all_reg[14];											  // zero out union values
			resp = _C_COM.ioReadReg(C[cham_i].addr, REG_GI0, io_all_reg, 14); // read through all input registers (6 active, 2 unused) and the 6 active output registers
			if (resp != 0)
				run_error = 1; // set error flag

			uint8_t io_in_reg[6] = {io_all_reg[0], io_all_reg[1], io_all_reg[2], io_all_reg[3], io_all_reg[4], io_all_reg[5]};		// copy out values
			uint8_t io_out_reg[6] = {io_all_reg[8], io_all_reg[9], io_all_reg[10], io_all_reg[11], io_all_reg[12], io_all_reg[13]}; // copy out values
			uint8_t io_out_mask[6] = {0};

			// Compare check ifcurrent registry io to saved registry
			uint8_t f_track_reg = 0;	 // will track if all io pins have been been triggered
			uint8_t f_do_pwm_update = 0; // will track if pwm registry should be updated
			for (size_t prt_i = 0; prt_i < C[cham_i].pmsDynIO.nPorts; prt_i++)
			{ // loop ports

				// Compare current io registry to saved mask
				uint8_t port_n = C[cham_i].pmsDynIO.port[prt_i];						   // get port number
				uint8_t comp_byte = C[cham_i].pmsDynIO.bitMask[prt_i] & io_in_reg[port_n]; // triggered pin/bit matching mask will be 1
				f_track_reg += C[cham_i].pmsDynIO.bitMask[prt_i];						   // update flag

				// Check each bit in comparison byte
				for (size_t pin_i = 0; pin_i < C[cham_i].pmsDynIO.nPins[prt_i]; pin_i++)
				{														  // loop pins
					uint8_t pin_n = C[cham_i].pmsDynIO.pin[prt_i][pin_i]; // get pin number
					if (!bitRead(C[cham_i].pmsDynIO.bitMask[prt_i], pin_n))
						continue; // check if pin still flagged in reg byte
					if (!bitRead(comp_byte, pin_n))
						continue;										   // check io pin
					bitWrite(C[cham_i].pmsDynIO.bitMask[prt_i], pin_n, 0); // remove from byte reg

					// Update pwm registry array (note, sets both up and down pwm reg entries as this makes the code easier)
					uint8_t wall_n = C[cham_i].pmsDynIO.wall[prt_i][pin_i]; // get wall number
					bitWrite(io_out_mask[wms.pwmDown[0][wall_n]], wms.pwmDown[1][wall_n], 1);
					bitWrite(io_out_mask[wms.pwmUp[0][wall_n]], wms.pwmUp[1][wall_n], 1);

					// Update state [0,1] [down,up] based on active switch function [1:io_down, 2:io_up]
					uint8_t swtch_fun = wms.funMap[port_n][pin_n]; // get wall state
					bitWrite(C[cham_i].bitWallPosition, wall_n, swtch_fun == 1 ? 0 : 1);

					// Set flags
					bitWrite(C[cham_i].bitWallMoveFlag, wall_n, 0); // reset wall bit in flag
					f_do_pwm_update = 1;							// flag to update pwm

					_DB.printMsgTime("\t\tend move %s: chamber=%d wall=%d dt=%s", swtch_fun == 1 ? "down" : "up", cham_i, wall_n, _DB.dtTrack());
				}
			}

			// Send pwm off command
			if (f_do_pwm_update)
			{																			 // check for update flag
				resp = _C_COM.ioWriteReg(C[cham_i].addr, io_out_mask, 6, 0, io_out_reg); // include last reg read and turn off pwms
				if (resp != 0)
					run_error = 1; // set error flag
			}
		}
	}

	// Check for any unifinished moves
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{ // loop chambers
		if (C[cham_i].bitWallMoveFlag == 0)
			continue; // check if chamber flagged for updating

		// Update error flag bitwise, set bit in bitWallErrorFlag to 1 if it or the corresponding bit in bitWallMoveFlag is equal to 1
		C[cham_i].bitWallErrorFlag = C[cham_i].bitWallErrorFlag | C[cham_i].bitWallMoveFlag;

		// Identify walls that were not moved
		for (size_t prt_i = 0; prt_i < C[cham_i].pmsDynIO.nPorts; prt_i++)
		{ // loop ports
			for (size_t pin_i = 0; pin_i < C[cham_i].pmsDynIO.nPins[prt_i]; pin_i++)
			{ // loop pins

				// Check if pin still flagged for movement
				uint8_t pin_n = C[cham_i].pmsDynIO.pin[prt_i][pin_i]; // get pin number
				if (!bitRead(C[cham_i].pmsDynIO.bitMask[prt_i], pin_n))
					continue;

				// Flag error for debugging and to stop all wall movement
				uint8_t wall_n = C[cham_i].pmsDynIO.wall[prt_i][pin_i]; // get wall number
				run_error = is_timedout && run_error != 1 ? 2 : 3;		// set error flag
				_DB.printMsgTime("\t\t!!movement failed: chamber=%d wall=%d cause=%s dt=%s!!", cham_i, wall_n,
								 run_error == 1 ? "i2c" : run_error == 2 ? "timedout"
																		 : "unknown",
								 _DB.dtTrack());
				// TEMP bitWrite(C[cham_i].bitWallErrorFlag, wall_n, 1); // set chamber/wall specific error flag
			}
		}
	}

	// Force stop all walls if error encountered
	if (run_error != 0)
		resp = forceStopWalls();
	else
		_DB.printMsgTime("\tFinished all movement");

	return run_error; // return error
}

/// <summary>
/// Sends a command to stop all walls by setting all PWM outputs to zero.
/// </summary>
/// <returns>Wire::method output [0-4].</returns>
uint8_t Wall_Operation::forceStopWalls()
{
	uint8_t resp = 0;
	// TEMP _DB.printMsgTime("\t!!Running forse stop!!");
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{																		   // loop chambers
		resp = _C_COM.ioWriteReg(C[cham_i].addr, pmsAllPWM.bitMaskLong, 6, 0); // stop all pwm output
		if (resp != 0)
			return resp;
	}
	return resp;
}

//+++++++ Testing and Debugging Methods ++++++++

/// <summary>
/// Used for testing the limit switch IO pins on a given Cypress chip.
/// Note, this will loop indefinaitely. Best to put this in the Arduino
/// Setup() function.
/// </summary>
/// <param name="cham_i">Index/number of the chamber to set [0-48]</param>
/// <param name="p_wall_inc">OPTIONAL: pointer array specifying wall index/number for wall(s) to test [0-7] max 8 entries. DEFAULT:[all walls]</param>
/// <param name="s">OPTIONAL: length of "p_wall_inc" array. DEFAULT:[8] </param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
/// <example>
/// Here's an example of how to use testWallIO:
/// <code>
/// cham_i = 0; //index of the chamber
/// uint8_t s = 3; //number of walls
/// uint8_t p_wall_inc[s] = {0, 2, 5}; //array with wall numbers to move
/// Wall_Operation::testWallIO(cham_i, a_wall, s); //this can be run more than once to setup multiple chambers
/// Wall_Operation::testWallIO(0); //this will test all walls in chamber 0
/// </code>
/// Mannually trigger switches to test there function
/// </example>
uint8_t Wall_Operation::testWallIO(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s)
{
	if (cham_i > 48 || s > 8)
		return -1;
	uint8_t p_wi[s]; // initialize array to handle null array argument
	if (p_wall_inc == nullptr)
	{ // set default 8 walls
		for (size_t i = 0; i < s; i++)
			p_wi[i] = i;
	}
	else
	{ // copy over input
		for (size_t i = 0; i < s; i++)
			p_wi[i] = p_wall_inc[i];
	}

	// Test input pins
	uint8_t r_bit_out;
	uint8_t resp = 0;
	_DB.printMsgTime("Runing test IO switches: chamber=%d wall=%s", cham_i, _DB.arrayStr(p_wi, s));
	while (true)
	{ // loop indefinitely
		for (size_t i = 0; i < s; i++)
		{ // loop walls
			uint8_t wall_n = p_wi[i];

			// Check down pins
			resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioDown[0][wall_n], wms.ioDown[1][wall_n], r_bit_out);
			if (resp != 0) // break out of loop if error returned
				break;
			if (r_bit_out == 1)
				_DB.printMsgTime("\tWall %d: down", wall_n);

			// Check up pins
			resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioUp[0][wall_n], wms.ioUp[1][wall_n], r_bit_out);
			if (resp != 0) // break out of loop if error returned
				break;
			if (r_bit_out == 1)
				_DB.printMsgTime("\tWall %d: up", wall_n);

			// Add small delay
			delay(10);
		}
	}
	// Print failure message if while loop is broken out of because of I2C coms issues
	_DB.printMsgTime("!!ERROR: Failed test IO switches: chamber=%d wall=%s!!", cham_i, _DB.arrayStr(p_wi, s));
	return resp;
}

/// <summary>
/// Used for testing the motor PWM outputs for a given Cypress chip.
/// Note, best to put this in the Arduino  Setup() function.
/// </summary>
/// <param name="cham_i">Index/number of the chamber to set [0-48]</param>
/// <param name="p_wall_inc">OPTIONAL: pointer array specifying wall index/number for wall(s) to test [0-7] max 8 entries. DEFAULT:[all walls]</param>
/// <param name="s">OPTIONAL: length of "p_wall_inc" array. DEFAULT:[8] </param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
/// <example>
/// @see Wall_Operation::testWallIO()
/// </example>
uint8_t Wall_Operation::testWallPWM(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s, uint32_t dt_run)
{
	if (cham_i > 48 || s > 8)
		return -1;
	uint8_t p_wi[s];
	if (p_wall_inc == nullptr)
	{ // set default 8 walls
		for (size_t i = 0; i < s; i++)
			p_wi[i] = i;
	}
	else
	{ // copy over input
		for (size_t i = 0; i < s; i++)
			p_wi[i] = p_wall_inc[i];
	}

	// Run each wall up then down for dt_run ms
	_DB.printMsgTime("Testing PWM: chamber=%d wall=%s", cham_i, _DB.arrayStr(p_wi, s));
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++)
	{ // loop walls
		uint8_t wall_n = p_wi[i];
		_DB.printMsgTime("\tWall %d: Up", wall_n);
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 1); // run wall up
		if (resp != 0)
			return resp;
		delay(dt_run);
		_DB.printMsgTime("\tWall %d: Down", wall_n);
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 1); // run wall down (run before so motoro hard stops)
		if (resp != 0)
			return resp;
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 0); // stop wall up pwm
		if (resp != 0)
			return resp;
		delay(dt_run);
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 0); // stop wall down pwm
		if (resp != 0)
			return resp;
	}
	return resp;
}

/// <summary>
/// Used for testing the overal wall module function for a given Cypress chip.
/// Note, best to put this in the Arduino  Setup() function.
/// </summary>
/// <param name="cham_i">Index/number of the chamber to set [0-48]</param>
/// <param name="p_wall_inc">OPTIONAL: pointer array specifying wall index/number for wall(s) to test [0-7] max 8 entries. DEFAULT:[all walls]</param>
/// <param name="s">OPTIONAL: length of "p_wall_inc" array. DEFAULT:[8] </param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
/// <example>
/// @see Wall_Operation::testWallIO()
/// </example>
uint8_t Wall_Operation::testWallOperation(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s)
{
	if (cham_i > 48 || s > 8)
		return -1;
	uint8_t p_wi[s];
	if (p_wall_inc == nullptr)
	{ // set default 8 walls
		for (size_t i = 0; i < s; i++)
			p_wi[i] = i;
	}
	else
	{ // copy over input
		for (size_t i = 0; i < s; i++)
			p_wi[i] = p_wall_inc[i];
	}

	// Test all walls
	_DB.printMsgTime("Testing move opperation: chamber=%d wall=%s", cham_i, _DB.arrayStr(p_wi, s));
	uint8_t r_bit_out = 1;
	uint16_t dt = 2000;
	uint16_t ts;
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++)
	{ // loop walls
		uint8_t wall_n = p_wi[i];
		_DB.printMsgTime("Running wall %d", wall_n);

		// Run up
		_DB.printMsgTime("\tup start");
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 1);
		if (resp != 0)
			return resp;
		ts = millis() + dt; // set timeout
		_DB.dtTrack(1);		// start timer
		while (true)
		{ // check up switch
			resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioUp[0][wall_n], wms.ioUp[1][wall_n], r_bit_out);
			if (resp != 0)
				return resp;
			if (r_bit_out == 1)
			{
				_DB.printMsgTime("\tup end [%s]", _DB.dtTrack());
				break;
			}
			else if (millis() >= ts)
			{
				_DB.printMsgTime("\t!!up timedout [%s]!!", _DB.dtTrack());
				break;
			}
			delay(10);
		}

		// Run down
		_DB.printMsgTime("\tDown start");
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 1);
		if (resp != 0)
			return resp;
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 0);
		if (resp != 0)
			return resp;
		ts = millis() + dt; // set timeout
		_DB.dtTrack(1);		// start timer
		while (true)
		{ // check up switch
			resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioDown[0][wall_n], wms.ioDown[1][wall_n], r_bit_out);
			if (resp != 0)
				return resp;
			if (r_bit_out == 1)
			{
				_DB.printMsgTime("\tdown end [%s]", _DB.dtTrack());
				break;
			}
			else if (millis() >= ts)
			{
				_DB.printMsgTime("\t!!down timedout [%s]!!", _DB.dtTrack());
				break;
			}
			delay(10);
		}
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 0);
		if (resp != 0)
			return resp;

		// Pause
		delay(500);
	}
	return resp;
}

/// <summary>
/// Used for debugging to print out all fields of a PMS struct.
/// </summary>
/// <param name="p_wall_inc">OPTIONAL: [0-7] max 8 entries. DEFAULT:[all walls] </param>
/// <param name="s">OPTIONAL: length of "p_wall_inc" array. DEFAULT:[8] </param>
void Wall_Operation::printPMS(PinMapStr pms)
{
	char buff[250];
	sprintf(buff, "\nIO/PWM nPorts=%d__________________", pms.nPorts);
	_DB.printMsgTime(buff);
	for (size_t prt_i = 0; prt_i < pms.nPorts; prt_i++)
	{
		sprintf(buff, "port[%d] nPins=%d bitMask=%s", pms.port[prt_i], pms.nPins[prt_i], _DB.binStr(pms.bitMask[prt_i]));
		_DB.printMsgTime(buff);
		for (size_t pin_i = 0; pin_i < pms.nPins[prt_i]; pin_i++)
		{
			sprintf(buff, "   wall=%d   pin=%d", pms.wall[prt_i][pin_i], pms.pin[prt_i][pin_i]);
			_DB.printMsgTime(buff);
		}
	}
}

/// Overloaded function for printing Ethercat register values.
/// This version of the function accepts a RegUnion object.
/// @param d_type: Specifies the data type to print. [0, 1] corresponds to [uint8, uint16].
/// @param u: A RegUnion object containing the register values to print.
void Wall_Operation::printEcat(uint8_t d_type, RegUnion u)
{
	// Print out register
	_DB.printMsgTime("\tEcat Register");
	for (size_t i = 0; i < 8; i++)
	{
		if (d_type == 1 || i == 0)
			_DB.printMsgTime("\t\tui16[%d] %d", i, u.ui16[i]);
		else if (d_type == 0)
			_DB.printMsgTime("\t\tui8[%d]  %d %d", i, u.ui8[2 * i], u.ui8[2 * i + 1]);
	}
	_DB.printMsgTime(" ");
}

/// OVERLOAD: function for printing Ethercat register values.
/// This version of the function accepts an array of integer register values.
/// @param d_type: Specifies the data type to print. [0, 1] corresponds to [uint8, uint16].
/// @param p_reg: An array of existing register values. DEFAULT: the function reads the register values directly from Ethercat.
void Wall_Operation::printEcat(uint8_t d_type, int p_reg[])
{
	RegUnion u;
	int p_r[8]; // initialize array to handle null array argument

	// Get register values directly from Ethercat
	if (p_reg == nullptr)
		ESMA.get_ecat_registers(p_r);
	else // copy array
		for (size_t i = 0; i < 8; i++)
			p_r[i] = p_reg[i];

	// Convert to RegUnion
	for (size_t i = 1; i < 8; i++)
		u.ui16[0] = p_reg[i];

	// Pass to other printEcat
	printEcat(d_type, u);
}