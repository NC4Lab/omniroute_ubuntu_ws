// ######################################

//======== Wall_Operation.cpp ==========

// ######################################

//============= INCLUDE ================
#include "Wall_Operation.h"

//======== CLASS: WALL_OPERATION ==========

/// @brief CONSTUCTOR: Create Wall_Operation class instance
///
/// @param _nCham: Spcify number of chambers to track [1-49]
/// @param _pwmDuty: Defualt duty cycle for all the pwm [0-255]
Wall_Operation::Wall_Operation(uint8_t _nCham, uint8_t _nChambMove, uint8_t _nWallAttempt, uint8_t _pwmDuty, uint16_t _dtMoveTimeout)
{
	// Store input variables
	nCham = _nCham;
	nChambMove = _nChambMove;
	nMoveAttempt = _nWallAttempt;
	pwmDuty = _pwmDuty;
	dtMoveTimeout = _dtMoveTimeout;

	// Update chamber addressess
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		C[cham_i].addr = _CypCom.ADDR_LIST[cham_i];

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

//------------------------ ETHERCAT COMMS METHODS ------------------------

/// @brief Used to process new ROS ethercat msg argument data.
void Wall_Operation::procEcatMessage()
{
	uint8_t msg_arg_arr[9];	  // store message arguments
	uint8_t arg_len = 0;	  // store argument length
	uint8_t i2c_status = 255; // track i2c status
	uint8_t run_status = 255; // track run status

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
		// Get system variables to pass to initSoftware()
		uint8_t setup_arr[5] = {
			EsmaCom.rcvEM.ArgU.ui8[0],
			EsmaCom.rcvEM.ArgU.ui8[1],
			EsmaCom.rcvEM.ArgU.ui8[2],
			EsmaCom.rcvEM.ArgU.ui8[3],
			EsmaCom.rcvEM.ArgU.ui8[4]};

		// Initialize software
		initSoftware(0, setup_arr);
	}

	// INITIALIZE_CYPRESS
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::INITIALIZE_CYPRESS)
	{
		i2c_status = initCypress();
	}

	// INITIALIZE_WALLS
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::INITIALIZE_WALLS)
	{
		run_status = initWalls(2); // move walls up and down
	}

	// REINITIALIZE_SYSTEM
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::REINITIALIZE_SYSTEM)
	{
		// Get system variables to pass to initSoftware()
		uint8_t setup_arr[5] = {
			EsmaCom.rcvEM.ArgU.ui8[0],
			EsmaCom.rcvEM.ArgU.ui8[1],
			EsmaCom.rcvEM.ArgU.ui8[2],
			EsmaCom.rcvEM.ArgU.ui8[3],
			EsmaCom.rcvEM.ArgU.ui8[4]};

		initSoftware(1, setup_arr);		   // reinitialize software
	}

	// RESET_SYSTEM
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::RESET_SYSTEM)
	{
		run_status = initWalls(0);	// move walls down
		i2c_status = initCypress(); // reinitialize cypress
	}

	// MOVE_WALLS
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::MOVE_WALLS)
	{
		// Loop through arguments and update walls to move
		for (size_t cham_i = 0; cham_i < EsmaCom.rcvEM.argLen; cham_i++)
		{
			// Get wall byte mask data
			uint8_t byte_wall_inc = EsmaCom.rcvEM.ArgU.ui8[cham_i];

			// Set walls to move up for this chamber
			setWallMove(cham_i, 1, byte_wall_inc);
		}

		// Run move walls operation
		run_status = moveWalls();
	}

	//............... Format Ecat Ack Data ...............

	// Check for errors and store error type

	// INITIALIZE_CYPRESS
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::INITIALIZE_CYPRESS)
	{
		// Set arg length to number of chambers
		arg_len = nCham;

		// Send i2c status
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			msg_arg_arr[cham_i] = C[cham_i].i2cStatus;
	}

	// INITIALIZE_WALLS OR MOVE_WALLS
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::INITIALIZE_WALLS ||
			 EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::MOVE_WALLS)
	{
		// Set arg length to number of chambers
		arg_len = nCham;

		// Get wall data for each chamber
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			if (run_status == 1) // send wall position byte mask
				msg_arg_arr[cham_i] = C[cham_i].bitWallPosition;
			else // send wall error byte mask
				msg_arg_arr[cham_i] = C[cham_i].bitWallErrorFlag;
	}

	//............... Send Ecat Ack ...............

	// I2C_FAILED
	if (i2c_status != 0 && i2c_status != 255)
		EsmaCom.writeEcatAck(EsmaCom.ErrorType::I2C_FAILED, msg_arg_arr, arg_len);

	// WALL_MOVE_FAILED
	else if (run_status != 1 && run_status != 255)
		EsmaCom.writeEcatAck(EsmaCom.ErrorType::WALL_MOVE_FAILED, msg_arg_arr, arg_len);

	// NO ERROR
	else
		EsmaCom.writeEcatAck(EsmaCom.ErrorType::ERR_NONE, msg_arg_arr, arg_len); // send back recieved message arguments

	//............... Reset Ecat ...............

	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::RESET_SYSTEM)
	{
		// Reset software including Ecat message variables after final ack is sent
		initSoftware(2);
	}
}

//------------------------ DATA HANDELING ------------------------

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

/// @overload: Option for additional @ref Wall_Operation::WallMapStruct entries used
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

/// @brief Used within @ref Wall_Operation::_makePMS to do the actual work of adding the pin entries to the PMS structs.
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
			if (r_pms.portInc[prt_i] != p_port[wal_i] && r_pms.portInc[prt_i] != 255)
				continue;																 // find first emtpy  or existing entry and store there
			r_pms.nPortsInc = r_pms.portInc[prt_i] == 255 ? prt_i + 1 : r_pms.nPortsInc; // update length
			r_pms.portInc[prt_i] = p_port[wal_i];
			break;
		}
	}
	// Sort array
	_sortArr(r_pms.portInc, 6);
}

/// @brief Used within @ref Wall_Operation::_makePMS to do the actual work of adding the port entries to the PMS structs.
///
/// @param r_pms: Reference to PMS to be updated.
/// @param p_port: Array of port values from an @ref Wall_Operation::WallMapStruct.
/// @param p_pin: Array of pin values from an @ref Wall_Operation::WallMapStruct.
void Wall_Operation::_addPinPMS(PinMapStruct &r_pms, uint8_t p_port[], uint8_t p_pin[])
{
	for (size_t prt_i = 0; prt_i < 6; prt_i++)
	{ // loop ports in struct arr
		if (r_pms.portInc[prt_i] == 255)
			break; // bail if reached end of list
		for (size_t wal_i = 0; wal_i < 8; wal_i++)
		{ // loop wall list
			if (p_port[wal_i] != r_pms.portInc[prt_i])
				continue; // check port match
			for (size_t pin_ii = 0; pin_ii < 8; pin_ii++)
			{ // loop pin struct
				if (r_pms.pinInc[prt_i][pin_ii] != 255)
					continue;						// find first emtpy entry and store there
				r_pms.nPinsInc[prt_i] = pin_ii + 1; // update length
				r_pms.pinInc[prt_i][pin_ii] = p_pin[wal_i];
				r_pms.wallInc[prt_i][pin_ii] = wal_i;
				break;
			}
		}
		// Sort pin and wall array based on pin number
		_sortArr(r_pms.pinInc[prt_i], 8, r_pms.wallInc[prt_i]);

		// Update registry byte
		for (size_t pin_i = 0; pin_i < r_pms.nPinsInc[prt_i]; pin_i++)
		{
			bitWrite(r_pms.byteMaskInc[prt_i], r_pms.pinInc[prt_i][pin_i], 1); // update short/truncated version
		}
		r_pms.byteMaskAll[r_pms.portInc[prt_i]] = r_pms.byteMaskInc[prt_i]; // update long/complete version
	}
}

/// @brief Sorts array values in assending order
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

/// @brief Initializes/Reinitializes entries in a dynamic PMS struct to there default values.
///
/// @param r_pms: Reference to PMS struct to be reset
void Wall_Operation::_resetPMS(PinMapStruct &r_pms)
{
	r_pms.nPortsInc = 0;
	for (size_t prt_i = 0; prt_i < 6; prt_i++)
	{
		r_pms.portInc[prt_i] = 255;
		r_pms.nPinsInc[prt_i] = 0;
		for (size_t pin_i = 0; pin_i < 8; pin_i++)
		{
			r_pms.pinInc[prt_i][pin_i] = 255;
			r_pms.wallInc[prt_i][pin_i] = 255;
		}
		r_pms.byteMaskInc[prt_i] = 0;
		r_pms.byteMaskAll[prt_i] = 0;
	}
}

/// @brief Updates dynamic PMS structs based on new wall configuration input.
///
/// @param r_pms1: PMS struct to use as the basis for entries in "r_pms2"
/// @param r_pms2: Reference to a PMS struct to update
/// @param wall_byte_mask: Byte mask in which bits set to one denote the active walls to include in the "r_pms2" struct.
void Wall_Operation::_updateDynamicPMS(PinMapStruct r_pms1, PinMapStruct &r_pms2, uint8_t wall_byte_mask)
{
	for (size_t prt_i = 0; prt_i < r_pms1.nPortsInc; prt_i++)
	{ // loop ports
		for (size_t pin_i = 0; pin_i < r_pms1.nPinsInc[prt_i]; pin_i++)
		{ // loop pins
			if (bitRead(wall_byte_mask, r_pms1.wallInc[prt_i][pin_i]) == 1)
			{ // check if wall is on this port

				// Copy all struct fields
				for (size_t prt_ii = 0; prt_ii < 6; prt_ii++) // loop ports
				{

					if (r_pms2.portInc[prt_ii] != r_pms1.portInc[prt_i] && r_pms2.portInc[prt_ii] != 255)
						continue;																				// find first emtpy  or existing entry and store there
					r_pms2.nPortsInc = r_pms2.portInc[prt_ii] == 255 ? r_pms2.nPortsInc + 1 : r_pms2.nPortsInc; // update active port count for new entry
					r_pms2.nPinsInc[prt_ii]++;																	// update active pin count
					r_pms2.portInc[prt_ii] = r_pms1.portInc[prt_i];												// update active port number

					for (size_t pin_ii = 0; pin_ii < 8; pin_ii++) // loop pins
					{

						if (r_pms2.pinInc[prt_ii][pin_ii] != 255)
							continue;															 // find first emtpy entry and store there
						r_pms2.pinInc[prt_ii][pin_ii] = r_pms1.pinInc[prt_i][pin_i];			 // update active pin number
						r_pms2.wallInc[prt_ii][pin_ii] = r_pms1.wallInc[prt_i][pin_i];			 // update active wall number
						bitWrite(r_pms2.byteMaskInc[prt_ii], r_pms2.pinInc[prt_ii][pin_ii], 1);	 // update active registry byte mask
						r_pms2.byteMaskAll[r_pms2.portInc[prt_ii]] = r_pms2.byteMaskInc[prt_ii]; // update long/complete registry byte mask
						break;
					}
					break;
				}
			}
		}
	}
}

//------------------------ SETUP METHODS ------------------------

/// @brief Initialize/reset all relivant runtime variables to prepare for new session
///
/// @param init_level: Specify level of initialization [0:initialize, 1:reinitialize, 2:reset]
/// @param setup_arg_arr: Setup variables from Ecat
void Wall_Operation::initSoftware(uint8_t init_level, uint8_t setup_arg_arr[])
{
	// Log/print initialization status
	_Dbg.printMsg(_Dbg.MT::ATTN_START, "SOFTWARE %s", init_level == 0 ? "INITIALIZED" : init_level == 1 ? "REINITIALIZED"
																										: "RESET");

	// Update wall opperation variables for initialization or reinitialization
	if (init_level == 0)
	{
		if (setup_arg_arr != nullptr)
		{
			nCham = setup_arg_arr[0];
			nChambMove = setup_arg_arr[1];
			nMoveAttempt = setup_arg_arr[2];
			pwmDuty = setup_arg_arr[3];
			dtMoveTimeout = setup_arg_arr[4] * 10; // convert uint8_t (cs) to uint16_t (ms)
		}

		// Update chamber address
		for (size_t cham_i = 0; cham_i < nCham; cham_i++) 
			C[cham_i].addr = _CypCom.ADDR_LIST[cham_i];

		// Print software setup variables
		_Dbg.printMsg("CHAMBERS[%d] MOVE MAX[%d] ATTEMPT MAX[%d] PWM[%d] TIMEOUT[%d]",
					  nCham, nChambMove, nMoveAttempt, pwmDuty, dtMoveTimeout);
	}

	// Change wall PWM duty cycle for reinitialize
	/// @note: This is the only Cypress setting that needs to be changed
	if (init_level == 1)
		for (size_t cham_i = 0; cham_i < nCham; cham_i++) 
			for (size_t wall_i = 0; wall_i < 8; wall_i++)
				changeWallDutyPWM(cham_i, wall_i, pwmDuty);

	// Reset all status tracking chamber variables
	for (size_t cham_i = 0; cham_i < nCham; cham_i++) 
	{
		C[cham_i].i2cStatus = 0;
		C[cham_i].runStatus = 0;
		C[cham_i].bitWallPosition = 0;
		C[cham_i].bitWallRaiseFlag = 0;
		C[cham_i].bitWallErrorFlag = 0;
	}

	// Initialize and connect or disconnect Ecat
	if (init_level == 0)
		EsmaCom.initEcat(true); // connect to ethercat
	else if (init_level == 2)
		EsmaCom.initEcat(false); // reset/disconnect ethercat
}

/// @brief Initialize/reset Cypress hardware
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t Wall_Operation::initCypress()
{
	_Dbg.printMsg(_Dbg.MT::ATTN_START, "RUNNING: CYPRESS INITIALIZATION");

	// Scan connected I2C devices
	_CypCom.i2cScan();

	// Loop through all chambers
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{
		uint8_t resp = 0;
		_Dbg.printMsg("INITIALIZATING: Chamber[%d] Cypress Chip[%s]", cham_i, _Dbg.hexStr(C[cham_i].addr));

		//............... Initialize Cypress Chip ...............

		// Setup Cypress chips and check I2C
		resp = _CypCom.setupCypress(C[cham_i].addr);
		C[cham_i].i2cStatus = C[cham_i].i2cStatus == 0 ? resp : C[cham_i].i2cStatus; // update i2c status
		if (C[cham_i].i2cStatus != 0)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "\t Cypress Chip Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg("\t FINISHED: Cypress Chip Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);

		//............... Initialize Cypress IO ...............

		// Setup IO pins for each chamber
		resp = _setupCypressIO(C[cham_i].addr);
		C[cham_i].i2cStatus = C[cham_i].i2cStatus == 0 ? resp : C[cham_i].i2cStatus; // update i2c status
		if (C[cham_i].i2cStatus != 0)												 // print error if failed
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "\t Cypress IO Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg("\t FINISHED: Cypress IO Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);

		//............... Get Starting Wall Position ...............

		/// @note: We want to get this before we setup the pwm, as this will cause the walls to move slightly
		getWallState(cham_i, 1, C[cham_i].bitWallPosition); // get walls in up position

		// Print warning if walls initialized in up position
		if (C[cham_i].bitWallPosition != 0)
			_Dbg.printMsg(_Dbg.MT::WARNING, "\t WALLS DETECTED IN UP STATE: chamber[%d] walls%s", cham_i, _Dbg.bitIndStr(C[cham_i].bitWallPosition));

		//............... Initialize Cypress PWM ...............

		// Setup PWM pins for each chamber
		resp = _setupCypressPWM(C[cham_i].addr);
		C[cham_i].i2cStatus = C[cham_i].i2cStatus == 0 ? resp : C[cham_i].i2cStatus; // update i2c status
		if (C[cham_i].i2cStatus != 0)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "\t Cypress PWM Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg("\t FINISHED: Cypress PWM Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
	}

	// Set return flag to value of any non zero chamber flag
	uint8_t i2c_status = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		i2c_status = i2c_status == 0 ? C[cham_i].i2cStatus : i2c_status; // update status

	_Dbg.printMsg(_Dbg.MT::ATTN_END, "FINISHED: CYPRESS INITIALIZATION");
	return i2c_status;
}

/// @brief Initialize/reset wall position
///
/// @note Running walls in both directions (init_walls=1) will allow the system to
/// identify and flag any walls that are not moving properly.
///
/// @param init_walls: Specify wall iniialization [0=down, 1=up, 2=up/down].
///
/// @return Success/error codes from @ref Wall_Operation::moveWalls())
uint8_t Wall_Operation::initWalls(uint8_t init_walls)
{
	_Dbg.printMsg(_Dbg.MT::ATTN_START, "RUNNING: WALL %s INITIALIZATION",
				  init_walls == 0 ? "DOWN" : init_walls == 1 ? "UP"
															 : "UP/DOWN");

	//............... Run Walls Up ...............
	if (init_walls == 1 || init_walls == 2)
	{
		// Set wall movef for all chambers
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			setWallMove(cham_i, 1);

		// Move walls
		moveWallsStaged();
	}

	//............... Run Walls Down ...............
	if (init_walls == 0 || init_walls == 2)
	{
		// Set wall movef for all chambers
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			setWallMove(cham_i, 0);

		// Move walls
		moveWallsStaged();
	}

	//............... Check Status ...............

	// Update overal run status and print
	uint8_t run_status = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		run_status = C[cham_i].runStatus > 1 ? C[cham_i].runStatus : run_status;

	// Print status
	if (run_status > 1)
		_Dbg.printMsg(run_status < 1 ? _Dbg.MT::INFO : _Dbg.MT::ERROR,
					  "%s: WALL %s INITIALIZATION",
					  run_status < 1 ? "FINISHED" : "FAILED",
					  init_walls == 0 ? "DOWN" : init_walls == 1 ? "UP"
																 : "UP/DOWN");

	return run_status;
}

/// @brief Setup IO pins for each chamber including the pin direction (input) a
/// and the type of drive mode (high impedance).
///
/// @note have to do some silly stuff with the output pins as well based on
/// page 11 of the Cypress datasheet "To  allow  input  operations without
/// reconfiguration, these [output] registers have to store 1's."
///
/// @param address: I2C address of Cypress chip to setup.
///
/// @return method output from @ref Wire::endTransmission().
uint8_t Wall_Operation::_setupCypressIO(uint8_t address)
{
	uint8_t resp = 0;
	uint8_t i2c_status = 0;

	// Set entire output register to off
	uint8_t p_byte_mask_in[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	resp = _CypCom.ioWriteReg(address, p_byte_mask_in, 6, 0);
	i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status

	if (resp == 0)
	{
		for (size_t prt_i = 0; prt_i < pmsAllIO.nPortsInc; prt_i++)
		{ // loop through port list
			// Set input pins as input
			resp = _CypCom.setPortRegister(address, REG_PIN_DIR, pmsAllIO.portInc[prt_i], pmsAllIO.byteMaskInc[prt_i], 1);
			i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status
			if (resp == 0)
			{
				// Set pins as pull down
				resp = _CypCom.setPortRegister(address, DRIVE_PULLDOWN, pmsAllIO.portInc[prt_i], pmsAllIO.byteMaskInc[prt_i], 1);
				i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status
				if (resp == 0)
				{
					// Set corrisponding output register entries to 1 as per datasheet
					resp = _CypCom.ioWritePort(address, pmsAllIO.portInc[prt_i], pmsAllIO.byteMaskInc[prt_i], 1);
					i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status
					if (resp != 0)
						return resp;
				}
			}
		}
	}

	return i2c_status;
}

/// @brief Setup PWM pins for each chamber including the specifying them as PWM outputs
/// and also detting the drive mode to "Strong Drive". This also sets up the PWM
/// Source using @ref Cypress_Com::setupSourcePWM()
///
/// @param address: I2C address of Cypress chip to setup.
///
/// @return Wire::method output from @ref Wire::endTransmission().
uint8_t Wall_Operation::_setupCypressPWM(uint8_t address)
{
	uint8_t resp = 0;

	// Setup PWM sources
	for (size_t src_i = 0; src_i < 8; src_i++)
	{
		resp = _CypCom.setupSourcePWM(address, wms.pwmSrc[src_i], pwmDuty);
		if (resp != 0)
			_Dbg.printMsg("\t !!failed PWM source setup: source[%d]", src_i);
	}

	// Setup wall pwm pins
	for (size_t prt_i = 0; prt_i < pmsAllPWM.nPortsInc; prt_i++)
	{ // loop through port list
		// Set pwm pins as pwm output
		resp = _CypCom.setPortRegister(address, REG_SEL_PWM_PORT_OUT, pmsAllPWM.portInc[prt_i], pmsAllPWM.byteMaskInc[prt_i], 1);
		if (resp == 0)
		{
			// Set pins as strong drive
			resp = _CypCom.setPortRegister(address, DRIVE_STRONG, pmsAllPWM.portInc[prt_i], pmsAllPWM.byteMaskInc[prt_i], 1);
		}
	}

	return resp;
}

//------------------------ RUNTIME METHODS ------------------------

/// @overload: Option set all walls to move
///
/// @param cham_i Index of the chamber to set [0-nCham].
/// @param pos_state_set Value specifying the new wall position to set the reg bits to [0:down, 1:up].
///
/// @return Success/error codes [0:success, -1=255:input argument error].
///
/// @details Here's an example of how to use this overload:
/// @code
/// Wall_Operation::WallOper.setWallMove(cham_i, 0); // lower all walls in chamber 0 up
/// Wall_Operation::WallOper.setWallMove(cham_i, 0); // raise all walls in chamber 0 down
/// @endcode
uint8_t Wall_Operation::setWallMove(uint8_t cham_i, uint8_t pos_state_set)
{
	// set all bits to 0 or 1
	uint8_t byte_wall_inc = pos_state_set == 0 ? 0 : 255;

	// Run private version of the method
	return _setWallMove(cham_i, pos_state_set, byte_wall_inc);
}
/// @overload: Option for including an array of wall numbers to move
///
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall numbers for walls to move [0-7] max 8 entries. .
/// @param s OPTIONAL: Length of "p_wall_inc".
///
/// @details Here's an example of how to use this overload
/// @code
/// uint8_t p_wall_inc[s] = {0, 2, 5}; // array with wall numbers to move
/// Wall_Operation::setWallMove(cham_i, 1, p_wall_inc, 3); // move walls in p_wall_inc up
/// Wall_Operation::setWallMove(cham_i, 0, p_wall_inc, 3); // move walls in p_wall_inc down
/// @endcode
uint8_t Wall_Operation::setWallMove(uint8_t cham_i, uint8_t pos_state_set, uint8_t p_wall_inc[], uint8_t s)
{
	// Handle array inputs
	if (s > 8)
		return -1;
	if (cham_i > nCham)
		return -1;

	// Set bit in wall byte to bit_val
	uint8_t byte_wall_inc;
	for (size_t i = 0; i < s; i++)
		bitWrite(byte_wall_inc, p_wall_inc[i], pos_state_set);

	// Run private version of the method
	return _setWallMove(cham_i, pos_state_set, byte_wall_inc);
}

/// @overload: Option for including byte mask for walls to move
///
/// @details This function and its versions updates the byte mask specifying which walls should be up.
/// It can be run more than once to setup multiple chambers with different wall configurations.
/// Run @ref Wall_Operation::changeWallPositions() after all settings complete.
///
/// @param byte_wall_inc Byte mask with bits set to one specifying which walls to move up
///
/// @details Here's an example of how to use this overload:
/// @code
/// Wall_Operation::WallOper.setWallMove(cham_i, 0, B00000110); // move walls 2 aind 3 down
/// Wall_Operation::WallOper.setWallMove(cham_i, 1, B00000110); // move walls 2 aind 3 up
/// @endcode
uint8_t Wall_Operation::setWallMove(uint8_t cham_i, uint8_t pos_state_set, uint8_t byte_wall_inc)
{
	// Run private version of the method
	return _setWallMove(cham_i, pos_state_set, byte_wall_inc);
}

/// @brief: Private workhorse version of the method
uint8_t Wall_Operation::_setWallMove(uint8_t cham_i, uint8_t pos_state_set, uint8_t byte_wall_inc)
{

	// Bail if chamber is flagged with I2C error
	if (C[cham_i].i2cStatus != 0)
	{
		_Dbg.printMsg(_Dbg.MT::WARNING, "\t SKIPPED: WALL MOVE SETUP: I2C Error: chamber[%d] i2c_status[%d]", cham_i, C[cham_i].i2cStatus);
		return 0;
	}

	// Get byte masks for walls to move up and down
	uint8_t wall_up_byte_mask = ~C[cham_i].bitWallPosition & byte_wall_inc;	  // get walls to move up
	uint8_t wall_down_byte_mask = C[cham_i].bitWallPosition & ~byte_wall_inc; // get walls to move down

	// Update/Modify chamber/wall info
	C[cham_i].bitWallRaiseFlag = wall_up_byte_mask | wall_down_byte_mask; // store values in bit flag

	// Bail if nothing to move
	if (C[cham_i].bitWallRaiseFlag == 0)
	{
		_Dbg.printMsg(_Dbg.MT::WARNING, "\t SKIPPED: WALL MOVE SETUP: No Walls to Move: chamber[%d]", cham_i);
		return 0;
	}
	else
	{
		_Dbg.printMsg("\t FINISHED: WALL MOVE SETUP: chamber[%d]", cham_i);
		if (wall_up_byte_mask > 0)
			_Dbg.printMsg("\t\t Up%s", _Dbg.bitIndStr(wall_up_byte_mask));
		if (wall_down_byte_mask > 0)
			_Dbg.printMsg("\t\t Down%s", _Dbg.bitIndStr(wall_down_byte_mask));

		return 1;
	}
}

uint8_t Wall_Operation::moveWallsStaged()
{
	uint8_t run_status = 0;

	// Create and array of chambers with walls flagged for movement
	uint8_t cham_arr_all[nCham];
	uint8_t n_cham_all = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		if (C[cham_i].bitWallRaiseFlag != 0)
			cham_arr_all[n_cham_all++] = cham_i;

	// Store chambers to move next
	uint8_t cham_arr_next[nChambMove];
	size_t cham_set_cnt = 0;

	// Move sets of chambers in stages
	for (size_t cham_i = 0; cham_i < n_cham_all; cham_i++)
	{
		// Store chambers to move next
		cham_arr_next[cham_set_cnt++] = cham_arr_all[cham_i];

		// Run walls once max reached
		if (cham_set_cnt == nChambMove || cham_set_cnt == n_cham_all)
		{
			uint8_t resp = moveWalls(cham_arr_next, cham_set_cnt);
			run_status = run_status <= 1 ? resp : run_status; // update run status

			// Reset counter
			cham_set_cnt = 0;
		}
	}

	return run_status;
}

/// @brief Change wall positions for all chambers set to move
uint8_t Wall_Operation::moveWalls()
{
	// Create and array of all chambers with walls flagged for movement
	uint8_t cham_arr[nCham];
	uint8_t n_cham = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		if (C[cham_i].bitWallRaiseFlag != 0)
			cham_arr[n_cham++] = cham_i;

	// Run main version of method
	return _moveWalls(cham_arr, n_cham);
}

/// @overload: Option verion to move walls for a subset of chambers
///
/// @param cham_arr: Array of chamber indexes/numbers to move
/// @param n_cham: Length of "cham_arr"
uint8_t Wall_Operation::moveWalls(uint8_t cham_arr[], uint8_t n_cham)
{
	// Run private version of method
	return _moveWalls(cham_arr, n_cham);
}

/// @brief: Private version of the method which handles moving walls for all specified
/// chambers and managing reattempts for any walls that did not move.
///
/// @return Status/error codes from @ref Wall_Operation::_moveConductor()
uint8_t Wall_Operation::_moveWalls(uint8_t cham_arr[], uint8_t n_cham)
{
	uint8_t run_status = 0;
	uint8_t attempt = 0;

	// Check if anything to move
	if (n_cham == 0)
	{
		_Dbg.printMsg("\t SKIPPED: MOVE WALLS: No Walls to Move");
		return 0;
	}

	// Store a copy of the wall byte for move wall and error flags
	uint8_t byte_wall_raise_flag[n_cham];
	uint8_t byte_wall_error_flag[n_cham];
	for (size_t cham_i = 0; cham_i < n_cham; cham_i++)
	{
		byte_wall_raise_flag[cham_i] = C[cham_i].bitWallRaiseFlag;
		byte_wall_error_flag[cham_i] = C[cham_i].bitWallErrorFlag;
	}

	// Make multiple attempts to move walls based on nMoveAttempt
	for (size_t att_i = 0; att_i < nMoveAttempt; att_i++)
	{
		attempt++;
		_Dbg.printMsg("\t RUNNING: MOVE WALLS ATTEMPT #%d: chambers%s", attempt, _Dbg.arrayStr(cham_arr, n_cham));

		// Copy back saved error flag to ensure reattempts only for failed walls
		for (size_t cham_i = 0; cham_i < n_cham; cham_i++)
			C[cham_i].bitWallErrorFlag = byte_wall_error_flag[cham_i];

		// Run wall movement
		uint8_t resp = _moveConductor(cham_arr, n_cham, attempt);
		run_status = run_status <= 1 ? resp : run_status; // update run status
		if (resp <= 1)
			break; // break if all move completed without errors

		// Update raise wall flag to include any walls with errors so these will be run again
		for (size_t cham_i = 0; cham_i < n_cham; cham_i++)
			C[cham_i].bitWallRaiseFlag = byte_wall_raise_flag[cham_i] & C[cham_i].bitWallErrorFlag;
	}

	// Pring success/warning message
	_Dbg.printMsg(attempt == nMoveAttempt ? _Dbg.MT::INFO : attempt < nMoveAttempt ? _Dbg.MT::WARNING
																				   : _Dbg.MT::ERROR,
				  "\t %s: MOVE WALLS AFTER #%d ATTEMPT: chambers%s",
				  attempt == nMoveAttempt ? "FINISHED" : attempt < nMoveAttempt ? "INCOMPLETE"
																				: "FAILED",
				  attempt, _Dbg.arrayStr(cham_arr, n_cham));

	return run_status;
}

/// @brief Private workhorse of the class, which mannages initiating and compleating
/// the wall movement for a given set of chambers
///
/// @details
/// This method depends on the `bitWallRaiseFlag` variable being set by the `setWallMove()` method.
/// @see setWallMove()
///
/// @param attempt: Move attempt number
///
/// @return Status/error codes [0:no move, 1:success, 2:error]
uint8_t Wall_Operation::_moveConductor(uint8_t cham_arr[], uint8_t n_cham, uint8_t attempt)
{

	// Set timeout timer
	uint32_t ts_timeout = millis() + dtMoveTimeout; // set timout
	_Dbg.dtTrack(1);

	//............... Start Wall Move ...............

	for (size_t i = 0; i < n_cham; i++)
	{
		size_t cham_i = cham_arr[i]; // get chamber index

		// Start move
		uint8_t resp = _moveStart(cham_i);
		C[cham_i].runStatus = C[cham_i].runStatus <= 1 ? resp : C[cham_i].runStatus; // update run status
	}

	//............... Check/Track Wall Move ...............

	// Initialize flags
	bool is_timedout = false;  // flag timeout
	uint8_t do_move_check = 1; // will track if all chamber movement done

	// Track wall movement
	while (!is_timedout && do_move_check != 0) // loop till finished or timed out
	{
		do_move_check = 0; // reset check flag

		for (size_t i = 0; i < n_cham; i++)
		{
			size_t cham_i = cham_arr[i];

			// Skip if no walls still flagged to move
			if (C[cham_i].bitWallRaiseFlag == 0)
				continue;

			// Check wall movement status based on IO readings
			uint8_t run_status = _moveCheck(cham_i);
			C[cham_i].runStatus = C[cham_i].runStatus <= 1 ? run_status : C[cham_i].runStatus; // update run status

			// Update check flag and and timeout flag
			do_move_check += C[cham_i].bitWallRaiseFlag;
			is_timedout = millis() >= ts_timeout; // check for timeout

			// Check for timeout
			C[cham_i].runStatus = is_timedout ? 3 : C[cham_i].runStatus;

			// // TEMP
			// _Dbg.printMsg(_Dbg.MT::DEBUG, "\t\t TEST_______________chamber[%d] status[%d] do_move_check[%d] is_timedout[%d]",
			// 			  cham_i, C[cham_i].statusRun, do_move_check, is_timedout);
		}
	}

	//............... Catch Any Errors ...............

	// Check for any unifinished moves
	for (size_t i = 0; i < n_cham; i++)
	{
		size_t cham_i = cham_arr[i];

		// Skip if no longer flagged for updating
		if (C[cham_i].bitWallRaiseFlag == 0)
			continue;

		// Update temp error flag bitwise, set bit in error flag to 1 if it or the corresponding bit in bitWallRaiseFlag is equal to 1
		C[cham_i].bitWallErrorFlag = C[cham_i].bitWallErrorFlag | C[cham_i].bitWallRaiseFlag;

		// Stop all PWM output for active walls in chamber
		/// @todo need to handle potential i2c error with this call
		_CypCom.ioWriteReg(C[cham_i].addr, pmsAllPWM.byteMaskAll, 6, 0); // stop all pwm output

		// Identify walls that were not moved
		for (size_t prt_i = 0; prt_i < C[cham_i].pmsActvIO.nPortsInc; prt_i++) // loop ports
		{
			for (size_t pin_i = 0; pin_i < C[cham_i].pmsActvIO.nPinsInc[prt_i]; pin_i++) // loop pins
			{

				// Check if pin still flagged for movement
				uint8_t pin_n = C[cham_i].pmsActvIO.pinInc[prt_i][pin_i]; // get pin number
				if (!bitRead(C[cham_i].pmsActvIO.byteMaskInc[prt_i], pin_n))
					continue;

				// Print error message
				uint8_t wall_n = C[cham_i].pmsActvIO.wallInc[prt_i][pin_i]; // get wall number
				_Dbg.printMsg(attempt < nMoveAttempt ? _Dbg.MT::WARNING : _Dbg.MT::ERROR,
							  "\t\t FAILED: ATTEMPT #%d: chamber[%d] wall[%d] cause[%s] dt[%s]",
							  attempt, cham_i, wall_n,
							  C[cham_i].runStatus == 2	 ? "i2c"
							  : C[cham_i].runStatus == 3 ? "timedout"
														 : "unknown",
							  _Dbg.dtTrack());
			}
		}
	}

	// Update overal run status
	uint8_t run_status = 0;
	for (size_t i = 0; i < n_cham; i++)
	{
		size_t cham_i = cham_arr[i];
		run_status = C[cham_i].runStatus > 1 ? 2 : run_status; // set error flag
	}

	return run_status;
}

/// @brief Used to start wall movement through PWM output
///
/// @param cham_i Index/number of the chamber to set [0-48]
///
///  @return Status/error codes [0:no move, 1:success, 2:i2c error]
uint8_t Wall_Operation::_moveStart(uint8_t cham_i)
{
	// Local vars
	uint8_t resp = 0;
	uint8_t run_status = 0; // track run status
	uint8_t i2c_status = 0; // track i2c status

	// Reset/Modify in dynamic PinMapStruct
	_resetPMS(C[cham_i].pmsActvPWM);
	_resetPMS(C[cham_i].pmsActvIO);

	// Get byte mask for walls that should be moved up/down excluding walls flagged for errors
	/// @note: This will cause any walls that are up and not flagged to be moved down
	uint8_t wall_up_byte_mask = (~C[cham_i].bitWallPosition & C[cham_i].bitWallRaiseFlag) & ~C[cham_i].bitWallErrorFlag;  // bitwise comparison, up (wall_state == 0 & wall_active == 1) & (wall_error = 0)
	uint8_t wall_down_byte_mask = (C[cham_i].bitWallPosition & C[cham_i].bitWallRaiseFlag) & ~C[cham_i].bitWallErrorFlag; // bitwise comparison, down (wall_state == 1 & wall_active == 0) & (wall_error = 0)

	// Update/Modify in dynamic PinMapStruct based on walls set to move
	_updateDynamicPMS(pmsDownIO, C[cham_i].pmsActvIO, wall_down_byte_mask);	  // pwm down
	_updateDynamicPMS(pmsUpIO, C[cham_i].pmsActvIO, wall_up_byte_mask);		  // pwm up
	_updateDynamicPMS(pmsDownPWM, C[cham_i].pmsActvPWM, wall_down_byte_mask); // io down
	_updateDynamicPMS(pmsUpPWM, C[cham_i].pmsActvPWM, wall_up_byte_mask);	  // io up

	// Move walls up/down
	resp = _CypCom.ioWriteReg(C[cham_i].addr, C[cham_i].pmsActvPWM.byteMaskAll, 6, 1);
	i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status

	// Check if anything to move
	if (wall_up_byte_mask > 0 || wall_down_byte_mask > 0)
	{
		run_status = 1; // update flag

		// Print walls being moved
		if (wall_up_byte_mask > 0 || wall_down_byte_mask > 0)
			_Dbg.printMsg("\t START: Wall Move: chamber[%d]", cham_i);
		if (wall_up_byte_mask > 0)
			_Dbg.printMsg("\t\t Up%s", _Dbg.bitIndStr(wall_up_byte_mask));
		if (wall_down_byte_mask > 0)
			_Dbg.printMsg("\t\t Down%s", _Dbg.bitIndStr(wall_down_byte_mask));
	}
	else
		run_status = 0; // update flag

	// Update run status
	run_status = i2c_status == 0 ? run_status : i2c_status; // update flag based on i2c status

	return run_status;
}

/// @brief Used to track the wall movement based on IO pins
///
/// @note both the input and output registry (up to the 6th pin) are read on each
/// call to avoid an additional read if we need to write the pwm output later
///
/// @param cham_i Index/number of the chamber to set [0-48]
///
/// @return Status/error codes [0:no move, 1:success, 2:i2c error]
uint8_t Wall_Operation::_moveCheck(uint8_t cham_i)
{
	// Local vars
	uint8_t resp = 0;
	uint8_t run_status = 0; // track run status
	uint8_t i2c_status = 0; // track i2c status

	// Get io input and output registry bytes.
	uint8_t io_all_reg[14];
	resp = _CypCom.ioReadReg(C[cham_i].addr, REG_GI0, io_all_reg, 14); // read through all input registers (6 active, 2 unused) and the 6 active output registers
	i2c_status = i2c_status == 0 ? resp : i2c_status;				   // update i2c status

	// Copy out input and output registry values seperately
	uint8_t io_in_reg[6] = {io_all_reg[0], io_all_reg[1], io_all_reg[2], io_all_reg[3], io_all_reg[4], io_all_reg[5]};		// copy out values
	uint8_t io_out_reg[6] = {io_all_reg[8], io_all_reg[9], io_all_reg[10], io_all_reg[11], io_all_reg[12], io_all_reg[13]}; // copy out values

	// Initialize output registry mask used for later if we want to turn off pwms
	uint8_t io_out_mask[6] = {0};

	// Initalize flag to track if pwm registry should be updated
	bool do_pwm_update = false;

	// Compare check ifcurrent registry io to saved registry for each port
	for (size_t prt_i = 0; prt_i < C[cham_i].pmsActvIO.nPortsInc; prt_i++) // loop ports
	{
		/// @todo bail if no io left to track for this port based on pmsDynIO.bitMask
		uint8_t port_n = C[cham_i].pmsActvIO.portInc[prt_i];

		// Compare current io registry to saved mask
		uint8_t comp_byte = C[cham_i].pmsActvIO.byteMaskInc[prt_i] & io_in_reg[port_n]; // triggered pin/bit matching mask will be 1

		// Check each bit in comparison byte
		for (size_t pin_i = 0; pin_i < C[cham_i].pmsActvIO.nPinsInc[prt_i]; pin_i++) // loop pins
		{
			uint8_t pin_n = C[cham_i].pmsActvIO.pinInc[prt_i][pin_i];

			// Skip if pin no longer flagged for movement
			/// @todo check if I actually need this
			if (!bitRead(C[cham_i].pmsActvIO.byteMaskInc[prt_i], pin_n))
				continue;

			// Skip if pin no longer flagged in reg byte
			if (!bitRead(comp_byte, pin_n))
				continue;

			// Modify in dynamic PinMapStruct to remove pin from bit mask
			bitWrite(C[cham_i].pmsActvIO.byteMaskInc[prt_i], pin_n, 0); // remove wall/pin from byte reg

			// Update pwm registry array
			/// @note: sets both up and down pwm reg entries to be turned off as this makes the code easier
			uint8_t wall_n = C[cham_i].pmsActvIO.wallInc[prt_i][pin_i]; // get wall number
			bitWrite(io_out_mask[wms.pwmDown[0][wall_n]], wms.pwmDown[1][wall_n], 1);
			bitWrite(io_out_mask[wms.pwmUp[0][wall_n]], wms.pwmUp[1][wall_n], 1);

			// Get triggered switch [1:io_down, 2:io_up]
			uint8_t swtch_fun = wms.funMap[port_n][pin_n];

			// Update state [0,1] [down,up] based on the triggered switch
			bitWrite(C[cham_i].bitWallPosition, wall_n, swtch_fun == 1 ? 0 : 1);

			// Set flags
			bitWrite(C[cham_i].bitWallRaiseFlag, wall_n, 0); // reset wall bit in flag
			do_pwm_update = true;							 // flag to update pwm

			_Dbg.printMsg("\t\t FINISHED: Wall Move %s: chamber[%d] wall[%d] dt[%s]", swtch_fun == 1 ? "down" : "up", cham_i, wall_n, _Dbg.dtTrack());
		}
	}

	// Send pwm off command
	if (do_pwm_update)
	{																			  // check for update flag
		resp = _CypCom.ioWriteReg(C[cham_i].addr, io_out_mask, 6, 0, io_out_reg); // include last reg read and turn off pwms
		i2c_status = i2c_status == 0 ? resp : i2c_status;						  // update i2c status
	}

	// Update run status
	run_status = i2c_status == 0 ? 1 : i2c_status; // update flag based on i2c status

	return run_status; // return error
}

/// @brief Used to get the current wall state/position based on limit switch IO
///
/// @param cham_i Index/number of the chamber to set [0-nCham].
/// @param pos_state_get Value specifying the wall position to get [0:down, 1:up].
/// @param byte_state_out Byte reference  to store state of up or down switches by wall (used as output).
///
/// @return Wire::method output [0-4] or [-1=255: input argument error].
uint8_t Wall_Operation::getWallState(uint8_t cham_i, uint8_t pos_state_get, uint8_t &byte_state_out)
{
	if (cham_i > nCham)
		return -1;

	// Get io input registry bytes.
	uint8_t io_in_reg[6];
	uint8_t resp = _CypCom.ioReadReg(C[cham_i].addr, REG_GI0, io_in_reg, 6);
	if (resp != 0)
		return resp;

	// Get wall io state
	for (size_t wall_i = 0; wall_i < 8; wall_i++)
	{
		// Get down io state
		if (pos_state_get == 0)
			bitWrite(byte_state_out, wall_i, bitRead(io_in_reg[wms.ioDown[0][wall_i]], wms.ioDown[1][wall_i]));
		// Get up io state
		else
			bitWrite(byte_state_out, wall_i, bitRead(io_in_reg[wms.ioUp[0][wall_i]], wms.ioUp[1][wall_i]));
	}

	return resp;
}

/// @brief Option to change the PWM duty cycle for a given wall.
/// @note This is basically a wrapper for @ref Cypress_Com::setSourceDutyPWM.
/// @note Could be useful if some walls are running at different speeds.
///
/// @param cham_i: Index of the chamber to set [0-48]
/// @param source: Specifies one of 8 sources to set. See @ref Wall_Operation::wms.pwmSrc.
/// @param duty: PWM duty cycle [0-255].
///
/// @return Wire::method output [0-4] or [-1=255:input argument error].
uint8_t Wall_Operation::changeWallDutyPWM(uint8_t cham_i, uint8_t wall_i, uint8_t duty)
{
	if (cham_i > nCham || wall_i > 7 || duty > 255)
		return -1;
	uint8_t resp = _CypCom.setSourceDutyPWM(C[cham_i].addr, wms.pwmSrc[wall_i], duty); // set duty cycle to duty
	return resp;
}

//------------------------ TESTING AND DEBUGGING METHODS ------------------------

/// @brief Used for testing the limit switch IO pins on a given Cypress chip.
/// @details This will loop indefinitely. Best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULTall walls
/// @param s OPTIONAL: Length of "p_wall_inc" array. DEFAULT: 8
///
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
	if (cham_i > nCham || s > 8)
		return -1;

	// initialize array to handle null array argument
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

	// Test input pins
	uint8_t r_bit_out;
	uint8_t resp = 0;
	_Dbg.printMsg(_Dbg.MT::ATTN, "RUNNING: Test IO switches: chamber[%d] walls[%s]", cham_i, _Dbg.arrayStr(p_wi, s));
	while (true)
	{ // loop indefinitely
		for (size_t i = 0; i < s; i++)
		{ // loop walls
			uint8_t wall_n = p_wi[i];

			// Check down pins
			resp = _CypCom.ioReadPin(C[cham_i].addr, wms.ioDown[0][wall_n], wms.ioDown[1][wall_n], r_bit_out);
			if (resp != 0) // break out of loop if error returned
				break;
			if (r_bit_out == 1)
				_Dbg.printMsg("\t Wall %d: down", wall_n);

			// Check up pins
			resp = _CypCom.ioReadPin(C[cham_i].addr, wms.ioUp[0][wall_n], wms.ioUp[1][wall_n], r_bit_out);
			if (resp != 0) // break out of loop if error returned
				break;
			if (r_bit_out == 1)
				_Dbg.printMsg("\t Wall %d: up", wall_n);

			// Add small delay
			delay(10);
		}
	}
	// Print failure message if while loop is broken out of because of I2C coms issues
	_Dbg.printMsg(_Dbg.MT::ERROR, "Test IO switches: chamber[%d] walls[%s]", cham_i, _Dbg.arrayStr(p_wi, s));
	return resp;
}

/// @brief Used for testing the motor PWM outputs for a given Cypress chip.
/// @details Note, best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULT: all walls
/// @param s OPTIONAL: Length of "p_wall_inc" array. DEFAULT: 8
///
/// @return Wire::method output [0-4] or [-1=255: input argument error].
///
/// @see Wall_Operation::testWallIO()
/// @example Refer to the example provided in @ref Wall_Operation::testWallIO().
uint8_t Wall_Operation::testWallPWM(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s, uint16_t dt_run)
{
	if (cham_i > nCham || s > 8)
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
	_Dbg.printMsg(_Dbg.MT::ATTN, "RUNNING: Test PWM: chamber[%d] walls[%s]", cham_i, _Dbg.arrayStr(p_wi, s));
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++)
	{ // loop walls
		uint8_t wall_n = p_wi[i];
		_Dbg.printMsg("\t Wall %d: Up", wall_n);
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 1); // run wall up
		if (resp != 0)
			return resp;
		delay(dt_run);
		_Dbg.printMsg("\t Wall %d: Down", wall_n);
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 1); // run wall down (run before so motoro hard stops)
		if (resp != 0)
			return resp;
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 0); // stop wall up pwm
		if (resp != 0)
			return resp;
		delay(dt_run);
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 0); // stop wall down pwm
		if (resp != 0)
			return resp;
	}
	return resp;
}

/// @brief Used for testing the overall wall module function for a given Cypress chip.
/// @details Note, best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULT: all walls
/// @param s OPTIONAL: Length of "p_wall_inc" array. DEFAULT: 8
///
/// @return Wire::method output [0-4] or [-1=255: input argument error].
///
/// @see Wall_Operation::testWallIO()
/// @example Refer to the example provided in @ref Wall_Operation::testWallIO().
uint8_t Wall_Operation::testWallOperation(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s)
{
	if (cham_i > nCham || s > 8)
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
	_Dbg.printMsg(_Dbg.MT::ATTN, "RUNNING: Test move opperation: chamber[%d] walls[%s]", cham_i, _Dbg.arrayStr(p_wi, s));
	uint8_t r_bit_out = 1;
	uint16_t dt = 2000;
	uint16_t ts;
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++)
	{ // loop walls
		uint8_t wall_n = p_wi[i];
		_Dbg.printMsg("\t Moving wall %d", wall_n);

		// Run up
		_Dbg.printMsg("\t\t up start");
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 1);
		if (resp != 0)
			return resp;
		ts = millis() + dt; // set timeout
		_Dbg.dtTrack(1);	// start timer
		while (true)
		{ // check up switch
			resp = _CypCom.ioReadPin(C[cham_i].addr, wms.ioUp[0][wall_n], wms.ioUp[1][wall_n], r_bit_out);
			if (resp != 0)
				return resp;
			if (r_bit_out == 1)
			{
				_Dbg.printMsg("\t\t up end [%s]", _Dbg.dtTrack());
				break;
			}
			else if (millis() >= ts)
			{
				_Dbg.printMsg("\t\t !!up timedout [%s]", _Dbg.dtTrack());
				break;
			}
			delay(10);
		}

		// Run down
		_Dbg.printMsg("\t\t down start");
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 1);
		if (resp != 0)
			return resp;
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 0);
		if (resp != 0)
			return resp;
		ts = millis() + dt; // set timeout
		_Dbg.dtTrack(1);	// start timer
		while (true)
		{ // check up switch
			resp = _CypCom.ioReadPin(C[cham_i].addr, wms.ioDown[0][wall_n], wms.ioDown[1][wall_n], r_bit_out);
			if (resp != 0)
				return resp;
			if (r_bit_out == 1)
			{
				_Dbg.printMsg("\t\t down end [%s]", _Dbg.dtTrack());
				break;
			}
			else if (millis() >= ts)
			{
				_Dbg.printMsg("\t\t !!down timedout [%s]", _Dbg.dtTrack());
				break;
			}
			delay(10);
		}
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 0);
		if (resp != 0)
			return resp;

		// Pause
		delay(500);
	}
	return resp;
}

/// @brief Used for debugging to print out all fields of a PMS struct.
///
/// @param p_wall_inc: OPTIONAL: [0-7] max 8 entries. DEFAULT: all walls
/// @param s: OPTIONAL: length of @param p_wall_inc array. DEFAULT: 8
void Wall_Operation::_printPMS(PinMapStruct pms)
{
	_Dbg.printMsg(_Dbg.MT::DEBUG, "IO/PWM nPorts[%d]_____________________", pms.nPortsInc);
	for (size_t prt_i = 0; prt_i < pms.nPortsInc; prt_i++)
	{
		_Dbg.printMsg(_Dbg.MT::DEBUG, "port[%d] nPins[%d] bitMask[%s]", pms.portInc[prt_i], pms.nPinsInc[prt_i], _Dbg.binStr(pms.byteMaskInc[prt_i]));
		for (size_t pin_i = 0; pin_i < pms.nPinsInc[prt_i]; pin_i++)
		{
			_Dbg.printMsg(_Dbg.MT::DEBUG, "\t wall[%d] pin[%d]", pms.wallInc[prt_i][pin_i], pms.pinInc[prt_i][pin_i]);
		}
	}
}