// ######################################

//======== Wall_Operation.cpp ==========

// ######################################

/// <file>
/// Used for the Wall_Operation class
/// <file>

//============= INCLUDE ================
#include "Wall_Operation.h"

//======== CLASS: WALL_OPERATION ==========

/// @brief CONSTUCTOR: Create Wall_Operation class instance
///
/// @param _nCham: Spcify number of chambers to track [1-49]
/// @param do_spi: OPTIONAL: spcify if SPI should be strated, will interfere with LiquidCrystal library
Wall_Operation::Wall_Operation(uint8_t _nCham, uint8_t _pwmDuty, uint8_t do_spi)
{
	// Store input variables
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

//++++++++++++++ ETHERCAT COMMS METHODS +++++++++++++++

/// @brief Set message ID entry in union and updated associated variable
void Wall_Operation::_uSetMsgID(EcatMessageStruct &r_EM, uint16_t msg_id)
{
	// Get new message ID: itterate id and roll over to 1 if max 16 bit value is reached
	if (msg_id == 255)
		msg_id = r_EM.msgID < 65535 ? r_EM.msgID + 1 : 1;

	// Set message ID union entry
	r_EM.RegU.ui16[r_EM.setUI.upd16(0)] = msg_id;
	_uGetMsgID(sndEM); // copy from union to associated struct variable
}

/// @brief Get message ID from union
void Wall_Operation::_uGetMsgID(EcatMessageStruct &r_EM)
{
	r_EM.msgID = r_EM.RegU.ui16[r_EM.getUI.upd16(0)];
}

/// @brief Set message type entry in union and updated associated variable
void Wall_Operation::_uSetMsgType(EcatMessageStruct &r_EM, MessageType msg_type_enum)
{
	// Get message type value
	uint8_t msg_type_val = static_cast<uint8_t>(msg_type_enum);

	// Set message type union entry
	r_EM.RegU.ui8[r_EM.setUI.upd8(2)] = msg_type_val;
	_uGetMsgType(sndEM); // copy from union to associated struct variable
}

/// @brief Get message type from union
bool Wall_Operation::_uGetMsgType(EcatMessageStruct &r_EM)
{
	// Get message type value
	uint8_t msg_type_val = r_EM.RegU.ui8[r_EM.getUI.upd8(2)];

	// Check for valid message type
	bool is_found = false;
	for (int i = 0; i < N_MessageType; ++i)
	{
		if (msg_type_val == i)
		{
			is_found = true;
			break;
		}
	};
	bool is_err = !is_found;

	// Set type to none if not found
	if (is_err)
		msg_type_val = static_cast<uint8_t>(MessageType::MSG_NONE);

	// Store message type value
	r_EM.msg_tp_val = msg_type_val;

	// Get message type enum
	r_EM.msgTp = static_cast<Wall_Operation::MessageType>(msg_type_val);

	// Copy string to struct
	if (!is_err)
		strncpy(r_EM.msg_tp_str, message_type_str[r_EM.msg_tp_val], sizeof(r_EM.msg_tp_str) - 1);
	else
		strncpy(r_EM.msg_tp_str, "NULL", sizeof(r_EM.msg_tp_str) - 1);
	r_EM.msg_tp_str[sizeof(r_EM.msg_tp_str) - 1] = '\0'; // ensure null termination

	return is_err;
}

/// @brief Set message argument length entry in union and updated associated variable
void Wall_Operation::_uSetArgLength(EcatMessageStruct &r_EM, uint8_t msg_arg_len)
{
	r_EM.RegU.ui8[r_EM.setUI.upd8(3)] = msg_arg_len;
	_uGetArgLength(sndEM); // copy from union to associated struct variable
}

/// @brief Get message argument length
void Wall_Operation::_uGetArgLength(EcatMessageStruct &r_EM)
{
	// Get message argument length
	r_EM.argLen = r_EM.RegU.ui8[r_EM.getUI.upd8(3)];
}

/// @brief Set 8 bit message argument data entry in union
void Wall_Operation::_uSetArgData(EcatMessageStruct &r_EM, uint8_t msg_arg_data8)
{
	// Increment argument union index
	r_EM.argUI.upd8();

	// Update message argument length from argument union 8 bit index
	_uSetArgLength(r_EM, r_EM.argUI.i8);

	// Get reg union index
	uint8_t regu8_i = r_EM.argUI.i8 + 3;

	// Set message argument data in reg union
	r_EM.RegU.ui8[r_EM.setUI.upd8(regu8_i)] = msg_arg_data8;
	_uGetArgData(r_EM); // copy from union to associated struct variable
}
/// @brief Set 16 bit message argument data entry in union
void Wall_Operation::_uSetArgData(EcatMessageStruct &r_EM, uint16_t msg_arg_data16)
{
	// Increment argument union index
	r_EM.argUI.upd16();

	// Update message argument length from argument union 16 bit index
	_uSetArgLength(r_EM, r_EM.argUI.i8);

	// Get reg union index
	uint8_t regu16_i = (r_EM.argUI.i8 + 3) / 2;

	// Set message argument data in reg union
	r_EM.RegU.ui16[r_EM.setUI.upd16(regu16_i)] = msg_arg_data16; // set message argument data in reg union
	_uGetArgData(r_EM);											 // copy from union to associated struct variable
}

/// @brief Get reg union message argument data and copy to arg union
void Wall_Operation::_uGetArgData(EcatMessageStruct &r_EM)
{
	for (size_t i = 0; i < r_EM.argLen; i++)
		r_EM.ArgU.ui8[i] = r_EM.RegU.ui8[r_EM.getUI.upd8()]; // copy to 8 bit argument Union
}

/// @brief Set message footer entry in union and updated associated variable
void Wall_Operation::_uSetFooter(EcatMessageStruct &r_EM)
{
	r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254;
	r_EM.RegU.ui8[r_EM.setUI.upd8()] = 254;
	_uGetFooter(sndEM); // copy from union to associated struct variable
}

/// @brief Get message footer from union
bool Wall_Operation::_uGetFooter(EcatMessageStruct &r_EM)
{
	r_EM.msgFoot[0] = r_EM.RegU.ui8[r_EM.getUI.upd8()];
	r_EM.msgFoot[1] = r_EM.RegU.ui8[r_EM.getUI.upd8()];
	return r_EM.msgFoot[0] == 254 && r_EM.msgFoot[1] == 254;
}

/// @brief Reset union data and indeces
void Wall_Operation::_uReset(EcatMessageStruct &r_EM)
{
	// Reset union data
	r_EM.RegU.ui64[0] = 0;
	r_EM.RegU.ui64[1] = 0;

	// Reset dynamic union indeces
	r_EM.setUI.reset();
	r_EM.getUI.reset();
	r_EM.argUI.reset();
}

/// @brief Check for and log any message processing errors
void Wall_Operation::_checkErr(EcatMessageStruct &r_EM, ErrorType err_tp, bool is_err)
{
	// Handle error
	if (is_err)
	{
		// Only run once
		if (r_EM.errTp != err_tp)
		{
			// Set error type
			r_EM.errTp = err_tp;

			// Get error string
			uint8_t err_tp_val = static_cast<uint8_t>(err_tp);
			strncpy(r_EM.err_tp_str, error_type_str[err_tp_val], sizeof(r_EM.err_tp_str) - 1);
			r_EM.err_tp_str[sizeof(r_EM.err_tp_str) - 1] = '\0'; // ensure null termination

			// Print error
			_DB.printMsgTime("!!ERROR: Ecat: %s: id=%d type=%s[%d]!!", r_EM.err_tp_str, r_EM.msgID, r_EM.msg_tp_str, r_EM.msg_tp_val);
			_printEcatReg(0, r_EM.RegU); // TEMP
		}
	}
	else if (r_EM.errTp == err_tp)
		r_EM.errTp = ErrorType::ERROR_NONE; // unset error type
}

/// @brief Overloaded function for printing Ethercat register values.
/// This version of the function accepts a RegUnion object.
/// @param d_type: Specifies the data type to print. [0, 1] corresponds to [uint8, uint16].
/// @param u: A RegUnion object containing the register values to print.
void Wall_Operation::_printEcatReg(uint8_t d_type, RegUnion u_reg)
{
	// Print out register
	_DB.printMsgTime("\tEcat Register");
	for (size_t i = 0; i < 8; i++)
	{
		if (d_type == 1 || i == 0)
			_DB.printMsgTime("\t\tui16[%d] %d", i, u_reg.ui16[i]);
		if (d_type == 0)
			_DB.printMsgTime("\t\tui8[%d]  %d %d", i, u_reg.ui8[2 * i], u_reg.ui8[2 * i + 1]);
	}
	_DB.printMsgTime(" ");
}

/// OVERLOAD: function for printing Ethercat register values.
/// This version of the function accepts an array of integer register values.
/// @param d_type: Specifies the data type to print. [0, 1] corresponds to [uint8, uint16].
/// @param p_reg: An array of existing register values. DEFAULT: the function reads the register values directly from Ethercat.
void Wall_Operation::_printEcatReg(uint8_t d_type, int p_reg[])
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
		u.ui16[i] = p_reg[i];

	// Pass to other _printEcatU
	_printEcatReg(d_type, u);
}

/// @brief: Used to send outgoing ROS ethercat msg data signalling which walls to raise.
///
///	@note: The outgoing register is structured uint16[8]
///	with all but first 16 bit value seperated into bytes
///
/// @note: The message length corresponds to number of bits.
///
/// @details: The outgoing register is structured as follows:
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
///
/// @details: The indexing of the RegUnion is is as follows:
/// The indexing of the RegUnion is is as follows:
///		ui16[0], ui8[0][1]		// id
///		ui16[1], ui8[2][3]		// type		arg length
///		ui16[2], ui8[4][5]		// arg 1	arg 2
///		ui16[3], ui8[6][7]		// arg 3	arg 4
///		ui16[4], ui8[8][9]     	// arg 5	arg 6
///		ui16[5], ui8[10][11]   	// arg 7	arg 8
///		ui16[6], ui8[12][13]  	// arg 9	arg 10
///		ui16[7], ui8[14][15]  	// footer
///
/// @param msg_type_enum: The type of the message to be sent.
/// @param p_msg_arg_data: OPTIONAL: The data for the message arguments. DEFAULT: nullptr.
/// @param msg_arg_len: OPTIONAL: The length of the message arguments in uint8. DEFAULT: 255.
void Wall_Operation::sendEthercatMessage(MessageType msg_type_enum, uint8_t p_msg_arg_data[], uint8_t msg_arg_len)
{

	// Reset union variables
	_uReset(sndEM);

	// Store new message id
	_uSetMsgID(sndEM); // in union

	// Store new message type
	_uSetMsgType(sndEM, msg_type_enum); // in union

	// 	------------- STORE ARGUMENTS -------------

	// CONFIRM_DONE
	if (sndEM.msgTp == MessageType::CONFIRM_DONE)
	{
		_uSetArgData(sndEM, rcvEM.msgID);	   // store 16 bit recieved message id
		_uSetArgData(sndEM, rcvEM.msg_tp_val); // recieved message type value
	}

	// HANDSHAKE
	else if (sndEM.msgTp == MessageType::HANDSHAKE)
	{
		_uSetArgLength(sndEM, 0); // message argument length to 0
	}

	// OTHER
	else
	{
		_uSetArgLength(sndEM, msg_arg_len); // store message argument length if provided
		if (p_msg_arg_data != nullptr)		// store message arguments if provided
			for (size_t i = 0; i < msg_arg_len; i++)
				_uSetArgData(sndEM, p_msg_arg_data[i]); // store message arguments if provided
	}

	// Update associated argument variables in struct
	_uGetArgLength(sndEM);

	// 	------------- FINISH SETUP AND WRITE -------------

	// Store footer
	_uSetFooter(sndEM); // in union

	// Set flag
	sndEM.isDone = false;

	// Write message
	for (size_t i = 0; i < 8; i++)
		ESMA.write_reg_value(i, sndEM.RegU.i16[i]);

	// Print message
	_DB.printMsgTime("SENT Ecat Message: id=%d type=%s", sndEM.msgID, sndEM.msg_tp_str);
	_printEcatReg(0, sndEM.RegU); // TEMP
}

/// @brief Used to get incoming ROS ethercat msg data.
///
/// @return Success/error codes [0:no message, 1:new message, 2:error]
uint8_t Wall_Operation::getEthercatMessage()
{
	bool is_err = false; // error flag

	// Read esmacat buffer to get register data
	ESMA.get_ecat_registers(tmpEM.RegU.i16);

	// Skip ethercat setup junk (255)
	if (tmpEM.RegU.ui8[0] == 255 || tmpEM.RegU.ui8[1] == 255)
		return 0;

	// Get message id
	_uGetMsgID(tmpEM);

	// Skip redundant messages
	if (tmpEM.msgID == rcvEM.msgID)
		return 0;

	// Get message type and check if valid
	is_err = _uGetMsgType(tmpEM);

	// Run check error for valid message type
	_checkErr(tmpEM, ErrorType::NO_MESSAGE_TYPE_MATCH, is_err);
	if (is_err)
		return 2; // return error flag

	// Check if message is preceding handshake
	is_err = !isHandshakeDone && tmpEM.msgTp != MessageType::HANDSHAKE;
	_checkErr(tmpEM, ErrorType::REGISTER_LEFTOVERS, is_err);
	if (is_err)
		return 2; // return error flag

	// Check for skipped or out of sequence messages
	is_err = tmpEM.msgID - rcvEM.msgID != 1;
	_checkErr(tmpEM, ErrorType::MESSAGE_ID_DISORDERED, is_err);
	if (is_err)
		return 2; // return error flag

	// Get argument length and arguments
	_uGetArgLength(tmpEM);
	_uGetArgData(tmpEM);

	// Get and check for footer
	is_err = _uGetFooter(tmpEM);

	// Run check error for valid footer
	_checkErr(tmpEM, ErrorType::MISSING_FOOTER, is_err);
	if (is_err)
		return 2; // return error flag

	// Set flag
	tmpEM.isDone = false;

	// Copy over data
	rcvEM = tmpEM;

	_DB.printMsgTime("RECIEVED Ecat Message: id=%d type=%s", rcvEM.msgID, rcvEM.msg_tp_str);
	_printEcatReg(0, rcvEM.RegU); // TEMP

	// Return new message flag
	return 1;
}

/// @brief Used to process new ROS ethercat msg argument data.
///
/// @return Success/error codes [0:no message, 1:new message, 2:error]
uint8_t Wall_Operation::procEthercatArguments()
{
	_DB.printMsgTime("PROCESSING Ecat Message: id=%d type=%s", rcvEM.msgID, rcvEM.msg_tp_str);

	// MOVE_WALLS
	if (rcvEM.msgTp == MessageType::MOVE_WALLS)
	{

		// Loop through arguments
		for (size_t cham_i = 0; cham_i < rcvEM.argLen; cham_i++)
		{
			// Get wall byte data
			uint8_t wall_b = rcvEM.ArgU.ui8[cham_i];

			// Get walls to move up and down
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

	// Return new message flag
	return 1;
}

/// @brief Used to exicute incoming ROS Ethercat msg data.
void Wall_Operation::executeEthercatMessage()
{

	// Handle message type
	if (rcvEM.msgTp == MessageType::HANDSHAKE)
	{
		// Set ethercat flag
		_DB.printMsgTime("\tEcat Comms Established");
		isHandshakeDone = true;
	}
	else if (rcvEM.msgTp == MessageType::MOVE_WALLS)
	{
		moveWalls();
	}
	else if (rcvEM.msgTp == MessageType::START_SESSION)
	{
		resetMaze(false); // dont reset certain variables
	}
	else if (rcvEM.msgTp == MessageType::END_SESSION)
	{
		resetMaze(true); // reset everything
	}

	// Set flag
	rcvEM.isDone = true;
}

//++++++++++++ DATA HANDELING +++++++++++++

/// @brief Used to create @ref Wall_Operation::PinMapStruct (PMS) structs, which are used to
/// store information related to pin/port and wall mapping for specified functions
/// (i.e., pwm, io, up, down)
///
/// @note these methods are only used in the construtor
///
/// @param r_pms: Reference to PMS to be updated
/// @param p_port_1: Array of port values from an @ref Wall_Operation::WallMapStruct
/// @param p_pin_1: Array of pin values from an @ref Wall_Operation::WallMapStruct
void Wall_Operation::_makePMS(PinMapStruct &r_pms, uint8_t p_port_1[], uint8_t p_pin_1[])
{
	_resetPMS(r_pms);
	_addPortPMS(r_pms, p_port_1, p_pin_1);
	_addPinPMS(r_pms, p_port_1, p_pin_1);
}

/// OVERLOAD: option for additional @ref Wall_Operation::WallMapStruct entries used
/// for creating PMS structs that include pins both up and down (e.g., all IO or all PWM pins)
///
/// @param p_port_2: Array of port values from an @ref Wall_Operation::WallMapStruct
/// @param p_pin_2: Array of pin values from an @ref Wall_Operation::WallMapStruct
void Wall_Operation::_makePMS(PinMapStruct &r_pms, uint8_t p_port_1[], uint8_t p_pin_1[], uint8_t p_port_2[], uint8_t p_pin_2[])
{
	_resetPMS(r_pms);
	_addPortPMS(r_pms, p_port_1, p_pin_1);
	_addPortPMS(r_pms, p_port_2, p_pin_2);
	_addPinPMS(r_pms, p_port_1, p_pin_1);
	_addPinPMS(r_pms, p_port_2, p_pin_2);
}

/// Used within @ref Wall_Operation::_makePMS to do the actual work of adding the pin entries to the PMS structs.
///
/// @param r_pms: Reference to PMS to be updated
/// @param p_port: Array of port values from an @ref Wall_Operation::WallMapStruct
/// @param p_pin: Array of pin values from an @ref Wall_Operation::WallMapStruct
void Wall_Operation::_addPortPMS(PinMapStruct &r_pms, uint8_t p_port[], uint8_t p_pin[])
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

/// Used within @ref Wall_Operation::_makePMS to do the actual work of adding the port entries to the PMS structs.
///
/// @param r_pms: Reference to PMS to be updated.
/// @param p_port: Array of port values from an @ref Wall_Operation::WallMapStruct.
/// @param p_pin: Array of pin values from an @ref Wall_Operation::WallMapStruct.
void Wall_Operation::_addPinPMS(PinMapStruct &r_pms, uint8_t p_port[], uint8_t p_pin[])
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

/// Sorts array values in assending order
/// Note, this is used exclusively by the above methods when creating the PMS structs.
///
/// @param p_arr: Array to be sorted
/// @param s: Length of array
/// @param p_co_arr: OPTIONAL: array to be sorted in the same order as "p_arr"
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

/// Resets most entries in a dynamic PMS struct to there default values.
///
/// @param r_pms: Reference to PMS struct to be reset
void Wall_Operation::_resetPMS(PinMapStruct &r_pms)
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

/// Updates dynamic PMS structs based on new wall configuration input.
///
/// @param r_pms1: PMS struct to use as the basis for entries in "r_pms2"
/// @param r_pms2: Reference to a PMS struct to update
/// @param wall_byte_mask: Byte mask in which bits set to one denote the active walls to include in the "r_pms2" struct.
void Wall_Operation::_updateDynamicPMS(PinMapStruct r_pms1, PinMapStruct &r_pms2, uint8_t wall_byte_mask)
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

//++++++++++++ SETUP METHODS +++++++++++++

/// @brief Reset @ref Wall_Operation::ChamberTrackStruct struct flags, which are used to track
/// the wall states and errors
/// @param do_full_reset: reset all variables
void Wall_Operation::resetMaze(uint8_t do_full_reset)
{
	uint8_t resp = 0;

	// Setup cypress chips
	for (size_t ch_i = 0; ch_i < nCham; ch_i++)
	{
		resp = _C_COM.setupCypress(C[ch_i].addr);
		if (resp != 0)
			_DB.printMsgTime("!!ERROR: Failed Cypress setup: chamber=%d address=%s!!", ch_i, _DB.hexStr(C[ch_i].addr));
		else if (ch_i + 1 == nCham)
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
		sndEM.msgID = 0;
		tmpEM.msgID = 0;
		rcvEM.msgID = 0;

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

/// @brief Setup IO pins for each chamber including the pin direction (input) a
/// and the type of drive mode (high impedance).
///
/// @note have to do some silly stuff with the output pins as well based on
/// page 11 of the Cypress datasheet "To  allow  input  operations  without
/// reconfiguration, these registers [output] have to store �1�s."
///
/// @return method output from @ref Cypress_Com::method.
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

/// @brief Setup PWM pins for each chamber including the specifying them as PWM outputs
/// and also detting the drive mode to "Strong Drive". This also sets up the PWM
/// Source using @ref Cypress_Com::setupSourcePWM()
///
/// @param pwm_duty: Duty cycle for the pwm [0-255]
/// @return Wire::method output from @ref Cypress_Com::method.
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

/// @brief Test up and down movment using @ref Wall_Operation::setWallCmdManual()
/// and @ref Wall_Operation::moveWalls()
/// @return Success/error codes from @ref Wall_Operation::moveWalls().
///
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

//+++++++++++++ RUNTIME METHODS ++++++++++++++

/// @brief Option to specify a given set of walls to move programmatically as an
/// alternative to @ref Wall_Operation::getWallCmdSerial.
/// Note, this function updates the byte mask specifying which walls should be up.
/// @param cham_i Index of the chamber to set [0-48].
/// @param bit_val_set Value to set the bits to [0,1]. DEFAULT[1].
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall numbers for walls to move [0-7] max 8 entries. DEFAULT:[all walls].
/// @param s OPTIONAL: Length of "p_wall_inc". DEFAULT:[8].
/// @return Success/error codes [0:success, -1=255:input argument error].
///
/// @details Here's an example of how to use setWallCmdManual:
/// @code
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
/// @endcode
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

/// @brief Option to change the PWM duty cycle for a given wall.
/// @note This is basically a wrapper for @ref Cypress_Com::setSourceDutyPWM.
/// @note Could be useful if some walls are running at different speeds.
///
/// @param cham_i: Index of the chamber to set [0-48]
/// @param source: Specifies one of 8 sources to set. See @ref Wall_Operation::wms.pwmSrc.
/// @param duty: PWM duty cycle [0-255].
/// @return Wire::method output [0-4] or [-1=255:input argument error].
uint8_t Wall_Operation::changeWallDutyPWM(uint8_t cham_i, uint8_t wall_i, uint8_t duty)
{
	if (cham_i > 48 || wall_i > 7 || duty > 255)
		return -1;
	uint8_t resp = _C_COM.setSourceDutyPWM(C[cham_i].addr, wms.pwmSrc[wall_i], duty); // set duty cycle to duty
	return resp;
}

/// @brief The main workhorse of the class, which mannages initiating and compleating
/// the wall movement for all active chambers based on either
/// @ref Wall_Operation::getWallCmdSerial or @ref Wall_Operation::getWallCmdSerial.
///
/// @param dt_timout: Time to attemp movement (ms) DEFAULT:[1000]
/// @return Success/error codes [0:success, 1:fail:unspecified, 2:fail:i2c, 3:fail:timeout]
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
	// return 0;

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
				/// @todo  set error flag for sepcific chamber/wall
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

/// @brief  a command to stop all walls by setting all PWM outputs to zero.
///
/// @return Wire::method output [0-4].
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

//+++++++ TESTING AND DEBUGGING METHODS ++++++++

/// @brief Used for testing the limit switch IO pins on a given Cypress chip.
/// @details This will loop indefinitely. Best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULT: [all walls]
/// @param s OPTIONAL: Length of "p_wall_inc" array. DEFAULT: [8]
/// @return Wire::method output [0-4] or [-1=255: input argument error].
///
/// @example Here's an example of how to use testWallIO
/// uint8_t cham_i = 0; // Index of the chamber
/// uint8_t s = 3; // Number of walls
/// uint8_t p_wall_inc[s] = {0, 2, 5}; // Array with wall numbers to move
/// Wall_Operation::testWallIO(cham_i, a_wall, s); // This can be run more than once to setup multiple chambers
/// Wall_Operation::testWallIO(0); // This will test all walls in chamber 0
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

/// @brief Used for testing the motor PWM outputs for a given Cypress chip.
/// @details Note, best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULT: [all walls]
/// @param s OPTIONAL: Length of "p_wall_inc" array. DEFAULT: [8]
/// @return Wire::method output [0-4] or [-1=255: input argument error].
///
/// @see Wall_Operation::testWallIO()
/// @example Refer to the example provided in @ref Wall_Operation::testWallIO().
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

/// @brief Used for testing the overall wall module function for a given Cypress chip.
/// @details Note, best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULT: [all walls]
/// @param s OPTIONAL: Length of "p_wall_inc" array. DEFAULT: [8]
/// @return Wire::method output [0-4] or [-1=255: input argument error].
///
/// @see Wall_Operation::testWallIO()
/// @example Refer to the example provided in @ref Wall_Operation::testWallIO().
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

/// @brief Used for debugging to print out all fields of a PMS struct.
///
/// @param p_wall_inc: OPTIONAL: [0-7] max 8 entries. DEFAULT:[all walls]
/// @param s: OPTIONAL: length of @param p_wall_inc array. DEFAULT:[8]
void Wall_Operation::printPMS(PinMapStruct pms)
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