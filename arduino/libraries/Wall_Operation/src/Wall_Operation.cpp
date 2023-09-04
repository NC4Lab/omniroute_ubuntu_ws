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
Wall_Operation::Wall_Operation(uint8_t _nCham, uint8_t _pwmDuty, uint8_t _nWallAttempt)
{
	// Store input variables
	nCham = _nCham;				  // store number of chambers
	nWallAttempt = _nWallAttempt; // store number of wall attempts
	pwmDuty = _pwmDuty;			  // store pwm duty cycle

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
	uint8_t msg_arg_arr[9]; // store message arguments
	uint8_t arg_len = 0;	// store argument length
	uint8_t i2c_status = 0; // track i2c status
	uint8_t run_status = 0; // track run status

	// Check for new message
	if (!EsmaCom.rcvEM.isNew)
		return;

	_Dbg.printMsg(_Dbg.MT::INFO, "(%d)ECAT PROCESSING: %s", EsmaCom.rcvEM.msgID, EsmaCom.rcvEM.msg_tp_str);

	//........................ Process and Execute Messages ........................

	// HANDSHAKE
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::HANDSHAKE)
	{
		// Get software setup variables
		uint8_t n_chambers = EsmaCom.rcvEM.ArgU.ui8[0];
		uint8_t n_wall_attempt = EsmaCom.rcvEM.ArgU.ui8[1];
		uint8_t pwm_duty = EsmaCom.rcvEM.ArgU.ui8[2];

		// Set ethercat flag
		_Dbg.printMsg(_Dbg.MT::ATTN, "ECAT COMMS CONNECTED");
		EsmaCom.isEcatConnected = true;

		// Initialize software
		initSoftware(n_chambers, n_wall_attempt, pwm_duty);
	}

	// INITIALIZE_CYPRESS
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::INITIALIZE_CYPRESS)
	{
		i2c_status = initCypress();
	}

	// INITIALIZE_WALLS
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::INITIALIZE_WALLS)
	{
		run_status = initWalls(1); // move walls up and down
	}

	// REINITIALIZE_ALL
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::REINITIALIZE_ALL)
	{
		run_status = initWalls(0);	// move walls down
		i2c_status = initCypress(); // reinitialize cypress
		initSoftware();				// lastly, reinitialize software
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

	//........................ Format Ecat Ack Data ........................

	// Check for errors and store error type

	// INITIALIZE_CYPRESS
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::INITIALIZE_CYPRESS)
	{
		// Set arg length to number of chambers
		arg_len = nCham;

		// Send i2c status
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			msg_arg_arr[cham_i] = C[cham_i].statusI2C;
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

	//........................ Send Ecat Ack ........................

	// I2C_FAILED
	if (i2c_status != 0)
		EsmaCom.writeEcatAck(EsmaCom.ErrorType::I2C_FAILED, msg_arg_arr, arg_len);

	// WALL_MOVE_FAILED
	else if (run_status != 1)
		EsmaCom.writeEcatAck(EsmaCom.ErrorType::WALL_MOVE_FAILED, msg_arg_arr, arg_len);

	// NO ERROR
	else
		EsmaCom.writeEcatAck(EsmaCom.ErrorType::ERR_NULL, msg_arg_arr, arg_len);

	//........................ Reinitialize Ecat ........................

	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::REINITIALIZE_ALL)
	{
		// Initialize/reset ecat message variables after final ack is sent
		EsmaCom.initEcat();
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

/// @overload: option for additional @ref Wall_Operation::WallMapStruct entries used
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

/// @brief Used within @ref Wall_Operation::_makePMS to do the actual work of adding the port entries to the PMS structs.
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

/// @brief Resets most entries in a dynamic PMS struct to there default values.
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

/// @brief Updates dynamic PMS structs based on new wall configuration input.
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

//------------------------ SETUP METHODS ------------------------

/// @brief Initialize/reset all relivant runtime variables to prepare for new session
///
/// @param n_chambers: Specify number of chambers. DEFAULT: 255[ignore]
void Wall_Operation::initSoftware(uint8_t n_chambers, uint8_t n_wall_attempt, uint8_t pwm_duty)
{
	// flag if this is a full initilization
	bool is_full_init = n_chambers != 255 && n_wall_attempt != 255 && pwm_duty != 255;

	// Update wall opperation variables
	if (is_full_init)
	{
		nCham = n_chambers;
		nWallAttempt = n_wall_attempt;
		pwmDuty = pwm_duty;
	}

	// Update chamber address and number
	for (size_t cham_i = 0; cham_i < nCham; cham_i++) // update chamber address
		C[cham_i].addr = _CypCom.ADDR_LIST[cham_i];

	// Reset all status tracking chamber variables
	for (size_t cham_i = 0; cham_i < nCham; cham_i++) // update chamber struct entries
	{
		C[cham_i].statusI2C = 0;
		C[cham_i].statusRun = 0;
		C[cham_i].bitWallErrorFlag = 0;
		C[cham_i].bitWallMoveFlag = 0;
		C[cham_i].bitWallPosition = 0;
		C[cham_i].bitWallUpdateFlag = 0;
	}

	if (n_chambers != 255 && n_chambers != 255 && n_chambers != 255)
		_Dbg.printMsg(_Dbg.MT::ATTN, "SOFTWARE INITIALIZED FOR %d CHAMBERS", nCham);
	else
		_Dbg.printMsg(_Dbg.MT::ATTN, "SOFTWARE REINITIALIZED");
}

/// @brief Initialize/reset Cypress hardware
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t Wall_Operation::initCypress()
{
	uint8_t resp = 0;

	_Dbg.printMsg(_Dbg.MT::ATTN_START, "RUNNING: CYPRESS INITIALIZATION");

	// Loop through all chambers
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{
		// Get current cypress/chamber address
		uint8_t address = C[cham_i].addr;
		_Dbg.printMsg("INITIALIZATING: Chamber[%d] Cypress Chip[%s]", cham_i, _Dbg.hexStr(address));

		//........................ Initialize Cypress Chip ........................

		// Setup Cypress chips and check I2C
		uint8_t resp = _CypCom.setupCypress(address);
		C[cham_i].statusI2C = C[cham_i].statusI2C == 0 ? resp : C[cham_i].statusI2C; // update i2c status
		if (C[cham_i].statusI2C != 0)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "\t Cypress Chip Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg("\t FINISHED: Cypress Chip Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), resp);

		//........................ Initialize Cypress IO ........................

		// Setup IO pins for each chamber
		resp = _setupCypressIO(address);
		C[cham_i].statusI2C = C[cham_i].statusI2C == 0 ? resp : C[cham_i].statusI2C; // update i2c status
		if (C[cham_i].statusI2C != 0)												 // print error if failed
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "\t Cypress IO Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg("\t FINISHED: Cypress IO Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), resp);

		//........................ Initialize Cypress PWM ........................

		// Setup PWM pins for each chamber
		resp = _setupCypressPWM(address);
		C[cham_i].statusI2C = C[cham_i].statusI2C == 0 ? resp : C[cham_i].statusI2C; // update i2c status
		if (C[cham_i].statusI2C != 0)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "\t Cypress PWM Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg("\t FINISHED: Cypress PWM Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), resp);
	}

	// Set return flag to value of any non zero chamber flag
	uint8_t i2c_status = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		i2c_status = i2c_status == 0 ? C[cham_i].statusI2C : i2c_status; // update status

	_Dbg.printMsg(_Dbg.MT::ATTN_END, "FINISHED: CYPRESS INITIALIZATION");
	return i2c_status;
}

/// @brief Initialize/reset Wall states
///
/// @param init_walls: Specify wall iniialization [0=down, 1=up/down].
/// @note Running walls in both directions (init_walls=1) will allow the system to
/// identify and flag any walls that are not moving properly.
///
/// @return Success/error codes from @ref Wall_Operation::moveWalls())
uint8_t Wall_Operation::initWalls(uint8_t init_walls)
{
	uint8_t run_status = 0;

	_Dbg.printMsg(_Dbg.MT::ATTN_START, "RUNNING: WALL INITIALIZATION");

	// Loop through all chambers
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{
		// Get current cypress/chamber address
		uint8_t address = C[cham_i].addr;

		// Run walls up
		if (init_walls == 1)
		{
			// Set walls to move up
			if (setWallMove(cham_i, 1) ==  1)
			{
				// Run move walls operation
				moveWalls();
				run_status = C[cham_i].statusRun > 1 ? C[cham_i].statusRun : run_status; // update run status
			}
		}

		// Run walls Down
		if (init_walls == 0 || init_walls == 1)
		{
			// Set walls to move down
			if (setWallMove(cham_i, 0) ==  1)
			{
				// Run move walls operation
				moveWalls();
				run_status = C[cham_i].statusRun > 1 ? C[cham_i].statusRun : run_status; // update run status
			}
		}

		if (C[cham_i].statusRun > 1)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "\t Wall Initialization failed: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), C[cham_i].statusRun);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg("\t FINISHED: Wall Initialization: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(address), C[cham_i].statusRun);
	}

	_Dbg.printMsg(_Dbg.MT::ATTN_END, "FINISHED: WALL INITIALIZATION");
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
		for (size_t prt_i = 0; prt_i < pmsAllIO.nPorts; prt_i++)
		{ // loop through port list
			// Set input pins as input
			resp = _CypCom.setPortRegister(address, REG_PIN_DIR, pmsAllIO.port[prt_i], pmsAllIO.bitMask[prt_i], 1);
			i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status
			if (resp == 0)
			{
				// Set pins as pull down
				resp = _CypCom.setPortRegister(address, DRIVE_PULLDOWN, pmsAllIO.port[prt_i], pmsAllIO.bitMask[prt_i], 1);
				i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status
				if (resp == 0)
				{
					// Set corrisponding output register entries to 1 as per datasheet
					resp = _CypCom.ioWritePort(address, pmsAllIO.port[prt_i], pmsAllIO.bitMask[prt_i], 1);
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
	for (size_t prt_i = 0; prt_i < pmsAllPWM.nPorts; prt_i++)
	{ // loop through port list
		// Set pwm pins as pwm output
		resp = _CypCom.setPortRegister(address, REG_SEL_PWM_PORT_OUT, pmsAllPWM.port[prt_i], pmsAllPWM.bitMask[prt_i], 1);
		if (resp == 0)
		{
			// Set pins as strong drive
			resp = _CypCom.setPortRegister(address, DRIVE_STRONG, pmsAllPWM.port[prt_i], pmsAllPWM.bitMask[prt_i], 1);
		}
	}

	return resp;
}

//------------------------ RUNTIME METHODS ------------------------

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
	uint8_t resp = _CypCom.setSourceDutyPWM(C[cham_i].addr, wms.pwmSrc[wall_i], duty); // set duty cycle to duty
	return resp;
}

/// @brief Specify a given set of walls to move programmatically
/// @details This function updates the byte mask specifying which walls should be up.
/// It can be run more than once to setup multiple chambers with different wall configurations.
/// Run @ref Wall_Operation::moveWalls() after all settings complete.
///
/// @param cham_i Index of the chamber to set [0-48].
/// @param bit_val_set Value to set the bits to specifying the position they should be in [0:down, 1:up].
///
/// @return Success/error codes [0:success, -1=255:input argument error].
///
/// @details Here's an example of how to use this overload:
/// @code
/// Wall_Operation::WallOper.setWallMove(cham_i, 0); // lower all walls in chamber 0 up
/// Wall_Operation::WallOper.setWallMove(cham_i, 0); // raise all walls in chamber 0 down
/// @endcode
uint8_t Wall_Operation::setWallMove(uint8_t cham_i, uint8_t bit_val_set)
{
	// set all bits to 0 or 1
	uint8_t byte_wall_inc = bit_val_set == 0 ? 0 : 255;

	// Run main method
	return setWallMove(cham_i, bit_val_set, byte_wall_inc);
}
/// @overload: option for including byte mask for walls to move
///
/// @param byte_wall_inc Byte mask with bits set to one specifying which walls to move up
///
/// @return Success/error codes [0:no move, 1:do move].
///
/// @details Here's an example of how to use this overload:
/// @code
/// Wall_Operation::WallOper.setWallMove(cham_i, 0, B00000110); // move walls 2 aind 3 down
/// Wall_Operation::WallOper.setWallMove(cham_i, 1, B00000110); // move walls 2 aind 3 up
/// @endcode
uint8_t Wall_Operation::setWallMove(uint8_t cham_i, uint8_t bit_val_set, uint8_t byte_wall_inc)
{
	uint32_t run_status = 0; // track run status

	// Bail if chamber is with I2C error
	if (C[cham_i].statusI2C != 0)
		return 0;

	// Update chamber/wall info
	uint8_t wall_up_byte_mask = ~C[cham_i].bitWallPosition & byte_wall_inc;	  // get walls to move up
	uint8_t wall_down_byte_mask = C[cham_i].bitWallPosition & ~byte_wall_inc; // get walls to move down
	C[cham_i].bitWallMoveFlag = wall_up_byte_mask | wall_down_byte_mask;	  // store values in bit flag

	// Bail if nothing to move
	if (C[cham_i].bitWallMoveFlag == 0)
		return 0;

	_Dbg.printMsg("\t FINISHED: Move Wall Setup: chamber[%d]", cham_i);
	if (wall_up_byte_mask > 0)
		_Dbg.printMsg("\t\t Up%s", _Dbg.bitIndStr(wall_up_byte_mask));
	if (wall_down_byte_mask > 0)
		_Dbg.printMsg("\t\t Down%s", _Dbg.bitIndStr(wall_down_byte_mask));

	return 1;
}
/// @overload: option for including an array of wall numbers to move
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
uint8_t Wall_Operation::setWallMove(uint8_t cham_i, uint8_t bit_val_set, uint8_t p_wall_inc[], uint8_t s)
{
	// Handle array inputs
	if (s > 8)
		return -1;
	if (cham_i > nCham)
		return -1;

	// Set bit in wall byte ot bit_val
	uint8_t byte_wall_inc;
	for (size_t i = 0; i < s; i++)
		bitWrite(byte_wall_inc, p_wall_inc[i], bit_val_set);

	// Run main method
	return setWallMove(cham_i, bit_val_set, byte_wall_inc);
}

/// @brief The main workhorse of the class, which mannages initiating and compleating
/// the wall movement for all active chambers based on either
///
/// @param dt_timout: Time to attemp movement (ms) DEFAULT: 1000
///
/// @return Status/error codes [0:no move, 1:success, 2:error]
uint8_t Wall_Operation::moveWalls(uint32_t dt_timout)
{
	bool is_err = false; // track if any errors
	uint8_t cham_inc_arr[nCham];
	uint8_t n_cham_inc = 0;

	// Check for chambers that need to be moved
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		if (C[cham_i].bitWallMoveFlag == 0) // check if any walls flagged for updating
			continue;
		else // Update chamber array
			cham_inc_arr[n_cham_inc++] = cham_i;

	// Check if anything to move
	if (n_cham_inc == 0)
	{
		_Dbg.printMsg(_Dbg.MT::ATTN, "WALL MOVE: No Walls to Move");
		return 0;
	}

	// Begin wall move
	for (size_t i = 0; i < n_cham_inc; i++)
	{
		size_t cham_i = cham_inc_arr[i]; // get chamber index

		// Start move
		C[cham_i].statusRun = _moveRunPWM(cham_i);

		// Flag errors
		is_err = is_err || C[cham_i].statusRun > 1;
	}

	// Skip move if error
	if (!is_err)
	{

		// Update timer
		uint32_t ts_timeout = millis() + dt_timout; // set timout
		_Dbg.dtTrack(1);

		// Initialize flags
		bool is_timedout = false;  // flag timeout
		uint8_t do_move_check = 1; // will track if all chamber movement done

		// Track wall movement
		while (!is_timedout && do_move_check != 0) // loop till finished or timed out
		{
			for (size_t i = 0; i < n_cham_inc; i++)
			{
				size_t cham_i = cham_inc_arr[i];

				// Skip if no walls still flagged to move
				if (C[cham_i].bitWallMoveFlag == 0)
					continue;

				// Check wall movement status based on IO readings
				C[cham_i].statusRun = _moveCheckIO(cham_i);

				// Update check flag and and timeout flag
				do_move_check += C[cham_i].bitWallMoveFlag;
				is_timedout = millis() >= ts_timeout; // check for timeout

				// Check for timeout
				C[cham_i].statusRun = ts_timeout ? 3 : C[cham_i].statusRun;
			}
		}
	}

	// Check for any unifinished moves
	for (size_t i = 0; i < n_cham_inc; i++)
	{
		size_t cham_i = cham_inc_arr[i];

		// Skip if no longer flagged for updating
		if (C[cham_i].bitWallMoveFlag == 0)
			continue;

		// Update error flag bitwise, set bit in bitWallErrorFlag to 1 if it or the corresponding bit in bitWallMoveFlag is equal to 1
		C[cham_i].bitWallErrorFlag = C[cham_i].bitWallErrorFlag | C[cham_i].bitWallMoveFlag;

		// Stop all PWM output for active walls in chamber
		uint8_t resp = _CypCom.ioWriteReg(C[cham_i].addr, pmsAllPWM.bitMaskLong, 6, 0); // stop all pwm output

		// Identify walls that were not moved
		for (size_t prt_i = 0; prt_i < C[cham_i].pmsDynIO.nPorts; prt_i++) // loop ports
		{
			for (size_t pin_i = 0; pin_i < C[cham_i].pmsDynIO.nPins[prt_i]; pin_i++) // loop pins
			{

				// Check if pin still flagged for movement
				uint8_t pin_n = C[cham_i].pmsDynIO.pin[prt_i][pin_i]; // get pin number
				if (!bitRead(C[cham_i].pmsDynIO.bitMask[prt_i], pin_n))
					continue;

				// Print error message
				uint8_t wall_n = C[cham_i].pmsDynIO.wall[prt_i][pin_i]; // get wall number
				_Dbg.printMsg(_Dbg.MT::WARNING, "\t\t Movement failed: chamber[%d] wall{%d} cause[%s] dt[%s]", cham_i, wall_n,
							  C[cham_i].statusRun == 2	 ? "i2c"
							  : C[cham_i].statusRun == 3 ? "timedout"
														 : "unknown",
							  _Dbg.dtTrack());
			}
		}
	}

	// Update overal run status
	uint32_t run_status = 0;
	for (size_t i = 0; i < n_cham_inc; i++)
	{
		size_t cham_i = cham_inc_arr[i];
		run_status = C[cham_i].statusRun > 1 ? 2 : run_status; // set error flag
	}

	// Pring success/warning message
	if (run_status > 1)
		_Dbg.printMsg(_Dbg.MT::WARNING, "\t Wall Movement Failed");
	else
		_Dbg.printMsg("\t FINISHED: All Wall Movement");
}

/// @brief Used to start wall movement through PWM output
///
/// @param cham_i Index/number of the chamber to set [0-48]
///
///  @return Status/error codes [0:no move, 1:success, 2:i2c error]
uint8_t Wall_Operation::_moveRunPWM(uint8_t cham_i)
{
	// Local vars
	uint8_t resp = 0;
	uint8_t run_status = 0; // track run status
	uint8_t i2c_status = 0; // track i2c status

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
	resp = _CypCom.ioWriteReg(C[cham_i].addr, C[cham_i].pmsDynPWM.bitMaskLong, 6, 1);
	i2c_status = i2c_status == 0 ? resp : i2c_status; // update i2c status

	// Check if anything to move
	if (wall_up_byte_mask > 0 || wall_down_byte_mask > 0)
	{
		run_status = 1; // update flag

		// Print walls being moved
		if (wall_up_byte_mask > 0 || wall_down_byte_mask > 0)
			_Dbg.printMsg("\t RUNNING: Moving Walls: chamber[%d]", cham_i);
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
/// @param cham_i Index/number of the chamber to set [0-48]
///
/// @return Status/error codes [0:no move, 1:success, 2:i2c error]
uint8_t Wall_Operation::_moveCheckIO(uint8_t cham_i)
{
	// Local vars
	uint8_t resp = 0;
	uint8_t run_status = 0; // track run status
	uint8_t i2c_status = 0; // track i2c status

	// Get io registry bytes. Note we are reading out the output registry as well to save an additional read if we write the pwm output later
	uint8_t io_all_reg[14];											   // zero out union values
	resp = _CypCom.ioReadReg(C[cham_i].addr, REG_GI0, io_all_reg, 14); // read through all input registers (6 active, 2 unused) and the 6 active output registers
	i2c_status = i2c_status == 0 ? resp : i2c_status;				   // update i2c status

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

			_Dbg.printMsg("\t\t End move %s: chamber[%d] wall{%d} dt[%s]", swtch_fun == 1 ? "down" : "up", cham_i, wall_n, _Dbg.dtTrack());
		}
	}

	// Send pwm off command
	if (f_do_pwm_update)
	{																			  // check for update flag
		resp = _CypCom.ioWriteReg(C[cham_i].addr, io_out_mask, 6, 0, io_out_reg); // include last reg read and turn off pwms
		i2c_status = i2c_status == 0 ? resp : i2c_status;						  // update i2c status
	}

	// Update run status
	run_status = i2c_status == 0 ? 1 : i2c_status; // update flag based on i2c status

	return run_status; // return error
}

/// @brief  a command to stop all walls by setting all PWM outputs to zero.
///
/// @return Wire::method output [0-4].
uint8_t Wall_Operation::_forceStopWalls()
{
	uint8_t resp = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{ // loop chambers
		_Dbg.printMsg(_Dbg.MT::WARNING, "Running forse stop: i2c[%s] chamber[%d]", _Dbg.hexStr(C[cham_i].addr), cham_i);
		resp = _CypCom.ioWriteReg(C[cham_i].addr, pmsAllPWM.bitMaskLong, 6, 0); // stop all pwm output
		if (resp != 0)
			return resp;
	}
	return resp;
}

//------------------------ TESTING AND DEBUGGING METHODS ------------------------

/// @brief Used for testing the limit switch IO pins on a given Cypress chip.
/// @details This will loop indefinitely. Best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULTall walls
/// @param s OPTIONAL: Length of "p_wall_inc" array. DEFAULT: 8
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
void Wall_Operation::printPMS(PinMapStruct pms)
{
	char buff[250];
	sprintf(buff, "\nIO/PWM nPorts[%d]_____________________", pms.nPorts);
	_Dbg.printMsg(buff);
	for (size_t prt_i = 0; prt_i < pms.nPorts; prt_i++)
	{
		sprintf(buff, "port[%d] nPins[%d] bitMask[%s]", pms.port[prt_i], pms.nPins[prt_i], _Dbg.binStr(pms.bitMask[prt_i]));
		_Dbg.printMsg(buff);
		for (size_t pin_i = 0; pin_i < pms.nPins[prt_i]; pin_i++)
		{
			sprintf(buff, "   wall[%d]   pin[%d]", pms.wall[prt_i][pin_i], pms.pin[prt_i][pin_i]);
			_Dbg.printMsg(buff);
		}
	}
}