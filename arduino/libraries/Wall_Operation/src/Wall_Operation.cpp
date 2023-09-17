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
Wall_Operation::Wall_Operation(uint8_t _nCham, uint8_t _nChamPerBlock, uint8_t _nWallAttempt, uint8_t _pwmDuty, uint16_t _dtMoveTimeout)
{
	// Store input variables
	nCham = _nCham;
	nChamPerBlock = _nChamPerBlock;
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
	_Dbg.printMsg(_Dbg.MT::HEAD1A, "START: SOFTWARE %s", init_level == 0 ? "INITIALIZATION" : init_level == 1 ? "REINITIALIZATION"
																											  : "RESET");
	// Update wall opperation variables for initialization or reinitialization
	if (init_level == 0 || init_level == 1)
	{
		if (setup_arg_arr != nullptr)
		{
			// Keep n chambers if reinitializing
			if (init_level != 1)
			{
				nCham = setup_arg_arr[0];
			}
			nChamPerBlock = setup_arg_arr[1];
			nMoveAttempt = setup_arg_arr[2];
			pwmDuty = setup_arg_arr[3];
			dtMoveTimeout = setup_arg_arr[4] * 10; // convert uint8_t (cs) to uint16_t (ms)
		}

		// Update chamber address and existing walls map
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		{
			C[cham_i].addr = _CypCom.ADDR_LIST[cham_i];
			C[cham_i].bitWallExist = bitWallExistMap[cham_i];
		}

		// Print software setup variables
		_Dbg.printMsg(_Dbg.MT::INFO, "SETTINGS: CHAM INIT[%d] CHAM PER BLOCK[%d] ATTEMPTS MOVE[%d] PWM[%d] TIMEOUT[%d]",
					  nCham, nChamPerBlock, nMoveAttempt, pwmDuty, dtMoveTimeout);
	}

	// Change wall PWM duty cycle for reinitialize
	/// @note: This is the only Cypress setting that needs to be changed
	/// @todo: Need to add I2C error handling
	if (init_level == 1)
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			if (C[cham_i].i2cStatus == 0)
				for (size_t wall_i = 0; wall_i < 8; wall_i++)
					_CypCom.setupSourcePWM(C[cham_i].addr, wms.pwmSrc[wall_i], pwmDuty);

	// Reset all status tracking chamber variables
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{
		// Keep i2c status if reinitializing
		if (init_level != 1)
		{
			C[cham_i].i2cStatus = 0;
		}
		C[cham_i].bitWallPosition = 0;
		C[cham_i].bitWallMoveUpFlag = 0;
		C[cham_i].bitWallMoveDownFlag = 0;
		C[cham_i].bitWallErrorFlag = 0;
	}

	// Initialize and connect or disconnect Ecat
	if (init_level == 0)
		EsmaCom.initEcat(true); // connect to ethercat
	else if (init_level == 2)
		EsmaCom.initEcat(false); // reset/disconnect ethercat

	// Log/print initialization status
	_Dbg.printMsg(_Dbg.MT::HEAD1B, "FINISHED: SOFTWARE %s", init_level == 0 ? "INITIALIZATION" : init_level == 1 ? "REINITIALIZATION"
																												 : "RESET");
}

/// @brief Initialize/reset Cypress hardware
///
/// @return Output from @ref Wire::endTransmission() [0-4] or [-1=255:input argument error].
uint8_t Wall_Operation::initCypress()
{
	_Dbg.printMsg(_Dbg.MT::HEAD1A, "START: CYPRESS INITIALIZATION");

	// Scan connected I2C devices
	// TEMP _CypCom.i2cScan();

	// Loop through all chambers
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
	{
		uint8_t resp = 0;
		_Dbg.printMsg(_Dbg.MT::INFO, "INITIALIZATING: Chamber[%d] Cypress Chip[%s]", cham_i, _Dbg.hexStr(C[cham_i].addr));

		//............... Initialize Cypress Chip ...............

		// Setup Cypress chips and check I2C
		C[cham_i].i2cStatus = C[cham_i].i2cStatus > 0 ? C[cham_i].i2cStatus : _CypCom.setupCypress(C[cham_i].addr);
		if (C[cham_i].i2cStatus != 0)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "Cypress Chip Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg(_Dbg.MT::INFO, "FINISHED: Cypress Chip Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);

		//............... Initialize Cypress IO ...............

		// Setup IO pins for each chamber
		C[cham_i].i2cStatus = C[cham_i].i2cStatus > 0 ? C[cham_i].i2cStatus : _setupCypressIO(C[cham_i].addr);
		if (C[cham_i].i2cStatus != 0) // print error if failed
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "Cypress IO Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg(_Dbg.MT::INFO, "FINISHED: Cypress IO Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);

		//............... Get Starting Wall Position ...............

		/// @note: We want to get this before we setup the pwm, as this will cause the walls to move slightly
		getWallState(cham_i, 1, C[cham_i].bitWallPosition); // get walls in up position

		// Print warning if walls initialized in up position
		if (C[cham_i].bitWallPosition != 0)
			_Dbg.printMsg(_Dbg.MT::WARNING, "WALLS DETECTED IN UP STATE: chamber[%d] walls%s", cham_i, _Dbg.bitIndStr(C[cham_i].bitWallPosition));

		//............... Initialize Cypress PWM ...............

		// Setup PWM pins for each chamber
		C[cham_i].i2cStatus = C[cham_i].i2cStatus > 0 ? C[cham_i].i2cStatus : _setupCypressPWM(C[cham_i].addr);
		if (C[cham_i].i2cStatus != 0)
		{
			_Dbg.printMsg(_Dbg.MT::ERROR, "Cypress PWM Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
			continue; // skip chamber if failed
		}
		else
			_Dbg.printMsg(_Dbg.MT::INFO, "FINISHED: Cypress PWM Setup: chamber=[%d|%s] status[%d]", cham_i, _Dbg.hexStr(C[cham_i].addr), resp);
	}

	//............... Check Status ...............

	// Set status return to any error
	uint8_t i2c_status = 0;
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		i2c_status = i2c_status == 0 ? C[cham_i].i2cStatus : i2c_status; // update status

	// Print status
	_Dbg.printMsg(i2c_status == 0 ? _Dbg.MT::HEAD1B : _Dbg.MT::ERROR,
				  "%s: CYPRESS INITIALIZATION: STATUS[%d]",
				  i2c_status == 0 <= 1 ? "FINISHED" : "FAILED",
				  i2c_status);

	return i2c_status;
}

/// @brief Initialize/reset wall position
///
/// @note Running walls in both directions (init_level=0) will allow the system to
/// identify and flag any walls that are not moving properly.
///
/// @param init_level: Specify level of initialization [0:initialize/reinitialize, 1:reset]
/// @return Success/error codes from @ref Wall_Operation::moveWallsByChamberBlocks()
uint8_t Wall_Operation::initWalls(uint8_t init_level)
{
	uint8_t run_status = 0;
	_Dbg.printMsg(_Dbg.MT::HEAD1A, "START: WALL %s INITIALIZATION",
				  init_level == 0 ? "UP/DOWN" : "DWON");

	//............... Run Walls Up for initialize/reinitialize ...............

	if (init_level == 0)
	{
		// Set wall movef for all chambers
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			setWallsToMove(cham_i, 1);

		// Move walls and inlude chambers with errors
		uint8_t resp = moveWallsByChamberBlocks(true);
		run_status = run_status <= 1 ? resp : run_status; // update run status

		// Set posiion to up even for failed walls to ensure they are rerun
		/// @todo: Find a less hacky way to achive this
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			C[cham_i].bitWallPosition = 255;
	}

	//............... Run Walls Down for initialize/reinitialize or reset ...............

	if (init_level == 0 || init_level == 1)
	{
		// Set wall movef for all chambers
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			setWallsToMove(cham_i, 0);

		// Move walls and inlude chambers with errors
		uint8_t resp = moveWallsByChamberBlocks(true);
		run_status = run_status <= 1 ? resp : run_status; // update run status
	}

	//............... Check Status ...............

	// Print status
	_Dbg.printMsg(run_status <= 1 ? _Dbg.MT::HEAD1B : _Dbg.MT::ERROR,
				  "%s: WALL %s INITIALIZATION: STATUS[%d]",
				  run_status <= 1 ? "FINISHED" : "FAILED",
				  init_level == 0 ? "UP/DOWN" : "DOWN", run_status);

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
/// @return method output from @ref Wire::endTransmission().
uint8_t Wall_Operation::_setupCypressIO(uint8_t address)
{
	uint8_t i2c_status = 0;

	// Set entire output register to off
	uint8_t p_byte_mask_in[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	i2c_status = _CypCom.ioWriteReg(address, p_byte_mask_in, 6, 0);
	if (i2c_status != 0)
		return i2c_status;

	for (size_t prt_i = 0; prt_i < pmsAllIO.nPortsInc; prt_i++)
	{

		// Set input pins as input
		i2c_status = _CypCom.setPortRegister(address, REG_PIN_DIR, pmsAllIO.portInc[prt_i], pmsAllIO.byteMaskInc[prt_i], 1);
		if (i2c_status != 0)
			return i2c_status;

		// Set pins as pull down
		i2c_status = _CypCom.setPortRegister(address, DRIVE_PULLDOWN, pmsAllIO.portInc[prt_i], pmsAllIO.byteMaskInc[prt_i], 1);
		if (i2c_status != 0)
			return i2c_status;

		// Set corrisponding output register entries to 1 as per datasheet
		i2c_status = _CypCom.ioWritePort(address, pmsAllIO.portInc[prt_i], pmsAllIO.byteMaskInc[prt_i], 1);
		if (i2c_status != 0)
			return i2c_status;
	}

	return i2c_status;
}

/// @brief Setup PWM pins for each chamber including the specifying them as PWM outputs
/// and also detting the drive mode to "Strong Drive". This also sets up the PWM
/// Source duty cycle
///
/// @param address: I2C address of Cypress chip to setup.
/// @return Wire::method output from @ref Wire::endTransmission() or [-1=255:input argument error].
uint8_t Wall_Operation::_setupCypressPWM(uint8_t address)
{
	uint8_t i2c_status = 0;

	// Setup PWM sources
	for (size_t src_i = 0; src_i < 8; src_i++)
	{
		i2c_status = _CypCom.setupSourcePWM(address, wms.pwmSrc[src_i], pwmDuty);
		if (i2c_status != 0)
			return i2c_status;
	}

	// Setup wall pwm pins
	for (size_t prt_i = 0; prt_i < pmsAllPWM.nPortsInc; prt_i++)
	{ // loop through port list

		// Set pwm pins as pwm output
		i2c_status = _CypCom.setPortRegister(address, REG_SEL_PWM_PORT_OUT, pmsAllPWM.portInc[prt_i], pmsAllPWM.byteMaskInc[prt_i], 1);
		if (i2c_status != 0)
			return i2c_status;

		// Set pins as strong drive
		i2c_status = _CypCom.setPortRegister(address, DRIVE_STRONG, pmsAllPWM.portInc[prt_i], pmsAllPWM.byteMaskInc[prt_i], 1);
		if (i2c_status != 0)
			return i2c_status;
	}

	return i2c_status;
}

//------------------------ RUNTIME METHODS ------------------------

/// @brief: Base method that set all walls for movement for a given chamber
///
/// @param cham_i Index of the chamber to set [0-nCham].
/// @param pos_state_set Value specifying the new wall position state [0:down, 1:up].
/// @return Status codes [0:no move, 1:success].@return
///
/// @details Here's an example of how to use this overload:
/// @code
/// Wall_Operation::WallOper.setWallsToMove(cham_i, 0); // lower all walls in chamber 0 up
/// Wall_Operation::WallOper.setWallsToMove(cham_i, 0); // raise all walls in chamber 0 down
/// @endcode
uint8_t Wall_Operation::setWallsToMove(uint8_t cham_i, uint8_t pos_state_set)
{
	// set all bits to 0 or 1
	uint8_t byte_wall_state_new = pos_state_set == 0 ? 0 : 255;

	// Run private version of the method
	return _setWallsToMove(cham_i, pos_state_set, byte_wall_state_new);
}
/// @overload: Option for including an array of wall numbers to move
///
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall numbers for walls to move [0-7] max 8 entries. .
/// @param s OPTIONAL: Length of "p_wall_inc".
///
/// @details Here's an example of how to use this overload
/// @code
/// uint8_t p_wall_inc[s] = {0, 2, 5}; // array with wall numbers to move
/// Wall_Operation::setWallsToMove(cham_i, 1, p_wall_inc, 3); // move walls in p_wall_inc up
/// Wall_Operation::setWallsToMove(cham_i, 0, p_wall_inc, 3); // move walls in p_wall_inc down
/// @endcode
uint8_t Wall_Operation::setWallsToMove(uint8_t cham_i, uint8_t pos_state_set, uint8_t p_wall_inc[], uint8_t s)
{
	// Handle array inputs
	if (s > 8)
		return -1;
	if (cham_i > nCham)
		return -1;

	// Set bit in wall byte to bit_val
	uint8_t byte_wall_state_new;
	for (size_t i = 0; i < s; i++)
		bitWrite(byte_wall_state_new, p_wall_inc[i], pos_state_set);

	// Run private version of the method
	return _setWallsToMove(cham_i, pos_state_set, byte_wall_state_new);
} /// @overload: Option for including byte mask for walls to move
///
/// @details This function and its versions updates the byte mask specifying which walls should be up.
/// It can be run more than once to setup multiple chambers with different wall configurations.
/// Run @ref Wall_Operation::moveWallsByChamberBlocks() after all settings complete.
///
/// @param byte_wall_state_new Byte mask with bits specifying the new wall position state [0:down, 1:up]
///
/// @details Here's an example of how to use this overload:
/// @code
/// Wall_Operation::WallOper.setWallsToMove(cham_i, 0, B00000110); // move walls 2 aind 3 down
/// Wall_Operation::WallOper.setWallsToMove(cham_i, 1, B00000110); // move walls 2 aind 3 up
/// @endcode
uint8_t Wall_Operation::setWallsToMove(uint8_t cham_i, uint8_t pos_state_set, uint8_t byte_wall_state_new)
{
	// Run private version of the method
	return _setWallsToMove(cham_i, pos_state_set, byte_wall_state_new);
}

/// @brief: Private workhorse version of the method
///
/// @return Status codes @ref Wall_Operation::setWallsToMove().
uint8_t Wall_Operation::_setWallsToMove(uint8_t cham_i, uint8_t pos_state_set, uint8_t byte_wall_state_new)
{

	// Bail if chamber is flagged with I2C error
	if (C[cham_i].i2cStatus != 0)
	{
		_Dbg.printMsg(_Dbg.MT::WARNING, "SKIPPED: WALL MOVE SETUP: I2C Flagged: chamber[%d] i2c_status[%d]", cham_i, C[cham_i].i2cStatus);
		return 0;
	}

	// Set up/down move flags using bitwise comparison and exclude any walls with errors
	C[cham_i].bitWallMoveUpFlag = C[cham_i].bitWallExist &
								  (~C[cham_i].bitWallPosition & byte_wall_state_new);
	C[cham_i].bitWallMoveDownFlag = C[cham_i].bitWallExist &
									(C[cham_i].bitWallPosition & ~byte_wall_state_new);

	// Bail if nothing to move
	if (C[cham_i].bitWallMoveUpFlag == 0 && C[cham_i].bitWallMoveDownFlag == 0)
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "FINISHED: WALL MOVE SETUP: No Walls to Move: chamber[%d]", cham_i);
		return 0;
	}
	else
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "FINISHED: WALL MOVE SETUP: chamber[%d] up%s down%s", cham_i,
					  C[cham_i].bitWallMoveUpFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallMoveUpFlag) : "[none]",
					  C[cham_i].bitWallMoveDownFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallMoveDownFlag) : "[none]");
		return 1;
	}
}

/// @brief: Main public method to change wall positions for all chambers set to move
/// either in blocks of all at once
///
/// @param do_inc_err_walls: Flag to include chambers with errors in the move.
/// @return Status codes [0:no move, 1:success, 2:error]
uint8_t Wall_Operation::moveWallsByChamberBlocks(bool do_inc_err_cham)
{
	uint8_t run_status = 0;
	uint8_t block_cnt = 0;	 // counter for number of stages
	uint8_t attempt_cnt = 0; // counter for number of attempts

	// Create and array of all chambers set to move
	uint8_t cham_all_arr[nCham];
	uint8_t n_cham_all = 0;

	// Find and store all chambers flagged for movement
	for (size_t cham_i = 0; cham_i < nCham; cham_i++)
		if ((C[cham_i].bitWallMoveUpFlag != 0 || C[cham_i].bitWallMoveDownFlag != 0) &&
			(do_inc_err_cham || C[cham_i].bitWallErrorFlag == 0)) // check for error flag
			cham_all_arr[n_cham_all++] = cham_i;

	// Bail if no chambers set to move
	if (n_cham_all == 0)
	{
		_Dbg.printMsg(_Dbg.MT::INFO, "SKIPPED: STAGED MOVE WALL: No Walls to Move");
		return 0;
	}

	// Print starting message
	_Dbg.printMsg(_Dbg.MT::HEAD2, "START: STAGED MOVE WALL: chambers%s", _Dbg.arrayStr(cham_all_arr, n_cham_all));

	// Store chambers to move next
	uint8_t cham_queued_arr[nChamPerBlock];
	size_t n_cham_queued = 0;

	// Move sets of chambers in stages
	for (size_t i = 0; i < n_cham_all; i++)
	{
		uint8_t cham_i = cham_all_arr[i];

		// Store chambers to move next
		cham_queued_arr[n_cham_queued++] = cham_i;

		// Run walls once max reached
		if (n_cham_queued == nChamPerBlock || (i + 1) == n_cham_all)
		{
			block_cnt++; // increment block counter

			uint8_t resp = _moveWallsByChamberBlocksWithRetry(cham_queued_arr, n_cham_queued, block_cnt, attempt_cnt);
			run_status = run_status <= 1 ? resp : run_status;

			// Reset counter
			n_cham_queued = 0;
		}
	}

	// Check final run status
	if (run_status <= 1)
		_Dbg.printMsg(_Dbg.MT::HEAD2, "FINISHED: STAGED MOVE WALL: status[%d] blocks[%d] attempts[%d] chambers%s",
					  run_status, block_cnt, attempt_cnt, _Dbg.arrayStr(cham_all_arr, n_cham_all));
	else
	{
		// Check each chamber
		uint8_t cham_err_arr[nCham];
		uint8_t n_cham_err = 0;
		for (size_t cham_i = 0; cham_i < nCham; cham_i++)
			if (C[cham_i].bitWallMoveUpFlag != 0 || C[cham_i].bitWallMoveDownFlag != 0)
			{
				// Store chamber index for error message
				cham_err_arr[n_cham_err++] = cham_i;

				// Reset move flags
				C[cham_i].bitWallMoveUpFlag = 0;
				C[cham_i].bitWallMoveDownFlag = 0;
			}
		_Dbg.printMsg(_Dbg.MT::ERROR, "FAILED: STAGED MOVE WALL: status[%d] blocks[%d] attempts[%d] chambers%s",
					  run_status, block_cnt, attempt_cnt, _Dbg.arrayStr(cham_err_arr, n_cham_err));
	}

	return run_status;
}

/// @brief: Private method which handles moving walls for all specified
/// chambers and managing reattempts for any walls that did not move.
///
/// @param cham_arr: Pointer array of chamber indexes to move.
/// @param n_cham: Number of chambers in "cham_arr".
/// @param block_cnt: Current chamber block count.
//// @return Status codes [0:no move, 1:success, 2:error] or [-1=255:input argument error].
uint8_t Wall_Operation::_moveWallsByChamberBlocksWithRetry(uint8_t cham_arr[], uint8_t n_cham, uint8_t block_cnt, uint8_t &r_attempt_cnt)
{
	// Handle array inputs
	if (n_cham > nCham)
		return -1;

	uint8_t run_status = 0;
	r_attempt_cnt = 0; // reset attempt counter reference

	// Make multiple attempts to move walls based on nMoveAttempt
	for (size_t att_i = 0; att_i < nMoveAttempt; att_i++)
	{
		r_attempt_cnt++; // increment attempt counter
		run_status = 0;	 // reset run status

		// Print attempt message
		_Dbg.printMsg(r_attempt_cnt == 1 ? _Dbg.MT::INFO : _Dbg.MT::WARNING,
					  "\t %s: MOVE WALL BLOCK #%d ATTEMPT #%d: chambers%s",
					  r_attempt_cnt == 1 ? "RUNNING" : "RE-RUNNING",
					  block_cnt, r_attempt_cnt, _Dbg.arrayStr(cham_arr, n_cham));

		// Run wall movement
		uint8_t resp = _moveWallsConductor(cham_arr, n_cham);
		run_status = run_status <= 1 ? resp : run_status; // update run status

		// Break if all move completed without errors
		if (resp <= 1)
			break;

		// Remove any finished chambers from the chamber array
		uint8_t cnt_cham = 0; // reset chamber counter
		for (size_t i = 0; i < n_cham; i++)
		{
			size_t cham_i = cham_arr[i]; // get chamber index

			// Add back to list if still flagged to move
			if (C[cham_i].bitWallMoveUpFlag > 0 || C[cham_i].bitWallMoveDownFlag > 0)
				cham_arr[cnt_cham++] = cham_i;
		}
		n_cham = cnt_cham; // update chamber count
	}

	return run_status;
}

/// @brief Private workhorse of the class, which mannages initiating and compleating
/// the wall movement for each block of chambers
///
/// @details
/// This method depends on the `bitWallRaiseFlag` variable being set by the `setWallsToMove()` method.
/// @see setWallsToMove()
///
/// @param cham_arr: Pointer array of chamber indexes to move.
/// @param n_cham: Number of chambers in "cham_arr".
/// @return Status/error codes [0:no move, 1:success, 2:i2c error, 3:timeout] or [-1=255:input argument error].
uint8_t Wall_Operation::_moveWallsConductor(uint8_t cham_arr[], uint8_t n_cham)
{
	// Handle array inputs
	if (n_cham > nCham)
		return -1;

	uint8_t run_status = 0;

	// Set timeout variables
	uint32_t ts_start = millis();
	_Dbg.dtTrack(1);

	//............... Start Wall Move ...............

	for (size_t i = 0; i < n_cham; i++)
	{
		size_t cham_i = cham_arr[i]; // get chamber index

		// Start move
		uint8_t resp = _initWallsMove(cham_i);
		run_status = run_status <= 1 ? resp : run_status; // update overal run status

		// Print walls being moved
		_Dbg.printMsg(_Dbg.MT::INFO, "\t START: Walls Move: chamber[%d] up%s down%s error%s status[%d]",
					  cham_i,
					  C[cham_i].bitWallMoveUpFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallMoveUpFlag) : "[none]",
					  C[cham_i].bitWallMoveDownFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallMoveDownFlag) : "[none]",
					  C[cham_i].bitWallErrorFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallErrorFlag) : "[none]",
					  resp);
	}

	//............... Monitor Wall Move ...............

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
			if (C[cham_i].bitWallMoveUpFlag == 0 && C[cham_i].bitWallMoveDownFlag == 0)
				continue;

			// Check wall movement status based on IO readings
			uint8_t resp = _monitorWallsMove(cham_i);
			run_status = run_status <= 1 ? resp : run_status; // update overal run status

			// Check for timeout
			is_timedout = millis() >= ts_start + dtMoveTimeout; // check for timeout

			// Update check flag and and timeout flag
			do_move_check += C[cham_i].bitWallMoveUpFlag;
			do_move_check += C[cham_i].bitWallMoveDownFlag;
		}
	}

	//............... Check/Track Final Move Status ...............

	// Check for timeout
	run_status = is_timedout ? 3 : run_status; // update overal run status

	// Check final status
	for (size_t i = 0; i < n_cham; i++)
	{
		size_t cham_i = cham_arr[i];

		// Check status for this chamber
		bool is_err = C[cham_i].bitWallMoveUpFlag != 0 || C[cham_i].bitWallMoveDownFlag != 0;
		_Dbg.printMsg(is_err ? _Dbg.MT::ERROR : _Dbg.MT::INFO,
					  "%s%s: Walls Move: chamber[%d] up%s down%s error%s status[%d]",
					  is_err ? " " : "\t",
					  is_err ? "FAILED" : "FINISHED",
					  cham_i,
					  C[cham_i].bitWallMoveUpFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallMoveUpFlag) : "[done]",
					  C[cham_i].bitWallMoveDownFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallMoveDownFlag) : "[done]",
					  C[cham_i].bitWallErrorFlag > 0 ? _Dbg.bitIndStr(C[cham_i].bitWallErrorFlag) : "[none]",
					  run_status);

		// Handle chamber failure
		if (C[cham_i].bitWallMoveUpFlag != 0 || C[cham_i].bitWallMoveDownFlag != 0)
		{
			// Update error flag bitwise, set bit in error flag to 1 if it or the corresponding bit in move up or down flag is equal to 1
			C[cham_i].bitWallErrorFlag = C[cham_i].bitWallErrorFlag | (C[cham_i].bitWallMoveUpFlag | C[cham_i].bitWallMoveDownFlag);

			// Turn off all pwm for this chamber
			uint8_t resp = _CypCom.ioWriteReg(C[cham_i].addr, pmsAllPWM.byteMaskAll, 6, 0); // stop all pwm output
			run_status = run_status <= 1 ? resp : run_status;								// update overal run status
		}
	}

	return run_status;
}

/// @brief Used to start wall movement through PWM output
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @return Status/error codes [1:move started, 2:i2c error] or [-1=255:input argument error].
uint8_t Wall_Operation::_initWallsMove(uint8_t cham_i)
{
	// Handle array inputs
	if (cham_i > nCham)
		return -1;

	// Reset/Modify in dynamic PinMapStruct
	_resetPMS(C[cham_i].pmsActvPWM);
	_resetPMS(C[cham_i].pmsActvIO);

	// Update/Modify in dynamic PinMapStruct based on walls set to move
	_updateDynamicPMS(pmsDownIO, C[cham_i].pmsActvIO, C[cham_i].bitWallMoveDownFlag);	// pwm down
	_updateDynamicPMS(pmsUpIO, C[cham_i].pmsActvIO, C[cham_i].bitWallMoveUpFlag);		// pwm up
	_updateDynamicPMS(pmsDownPWM, C[cham_i].pmsActvPWM, C[cham_i].bitWallMoveDownFlag); // io down
	_updateDynamicPMS(pmsUpPWM, C[cham_i].pmsActvPWM, C[cham_i].bitWallMoveUpFlag);		// io up

	// Move walls up/down
	uint8_t i2c_status = _CypCom.ioWriteReg(C[cham_i].addr, C[cham_i].pmsActvPWM.byteMaskAll, 6, 1);

	// Return run status
	return i2c_status != 0 ? 2 : 1;
}

/// @brief Used to track the wall movement based on IO pins
///
/// @note both the input and output registry (up to the 6th pin) are read on each
/// call to avoid an additional read if we need to write the pwm output later
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @return Status/error codes [0:still_waiting 1:all_move_down, 2:i2c error, 3:temeout] or [-1=255:input argument error].
uint8_t Wall_Operation::_monitorWallsMove(uint8_t cham_i)
{
	// Handle array inputs
	if (cham_i > nCham)
		return -1;

	// Local vars
	uint8_t i2c_status = 0; // track i2c status

	// Get io input and output registry bytes.
	uint8_t io_all_reg[14];
	i2c_status = _CypCom.ioReadReg(C[cham_i].addr, REG_GI0, io_all_reg, 14); // read through all input registers (6 active, 2 unused) and the 6 active output registers

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

			// Set both flags to false for convenience
			bitWrite(C[cham_i].bitWallMoveUpFlag, wall_n, 0);	// reset wall bit in flag
			bitWrite(C[cham_i].bitWallMoveDownFlag, wall_n, 0); // reset wall bit in flag

			/// Unset error flag
			/// @todo: Consider if error flag should be reset here
			bitWrite(C[cham_i].bitWallErrorFlag, wall_n, 0);

			// Flag to update pwm
			do_pwm_update = true;

			_Dbg.printMsg(_Dbg.MT::INFO, "\t\t FINISHED: Wall Move: chamber[%d] wall[%d][%s] dt[%s]",
						  cham_i, wall_n, swtch_fun == 1 ? "down" : "up", _Dbg.dtTrack());
		}
	}

	// Send pwm off command if move complete or timed out
	if (do_pwm_update)
	{																					  // check for update flag
		uint8_t resp = _CypCom.ioWriteReg(C[cham_i].addr, io_out_mask, 6, 0, io_out_reg); // include last reg read and turn off pwms
		i2c_status = i2c_status == 0 ? resp : i2c_status;								  // update i2c status
	}

	// Return run status
	return i2c_status != 0 ? 2 : (C[cham_i].bitWallMoveUpFlag == 0 && C[cham_i].bitWallMoveDownFlag == 0);
}

/// @brief Used to get the current wall state/position based on limit switch IO
///
/// @param cham_i Index/number of the chamber to set [0-nCham].
/// @param pos_state_get Value specifying the wall position to get [0:down, 1:up].
/// @param byte_state_out Byte reference  to store state of up or down switches by wall (used as output).
/// @return Wire::method output [0-4] or [-1=255: input argument error] or [-1=255:input argument error].
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

/// @brief Used to process new ROS ethercat msg argument data.
void Wall_Operation::procEcatMessage()
{
	uint8_t msg_arg_arr[9];	  // store message arguments
	uint8_t arg_len = 0;	  // store argument length
	uint8_t i2c_status = 0; // track i2c status
	uint8_t run_status = 0; // track run status

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
		run_status = initWalls(0);
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

		initSoftware(1, setup_arr); // reinitialize software
	}

	// RESET_SYSTEM
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::RESET_SYSTEM)
	{
		run_status = initWalls(1);	// move walls down
		i2c_status = initCypress(); // reinitialize cypress
	}

	// MOVE_WALLS
	else if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::MOVE_WALLS)
	{
		// Loop through arguments and update walls to move
		for (size_t cham_i = 0; cham_i < EsmaCom.rcvEM.argLen; cham_i++)
		{
			// Get wall byte mask data
			uint8_t byte_wall_state_new = EsmaCom.rcvEM.ArgU.ui8[cham_i];

			// Set walls to move up for this chamber
			setWallsToMove(cham_i, 1, byte_wall_state_new);
		}

		// Run move walls operation
		run_status = moveWallsByChamberBlocks();
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
			/// @note: Wall position is unused in the current ROS code
			if (run_status == 1) // send wall position byte mask
				msg_arg_arr[cham_i] = C[cham_i].bitWallPosition;
			else // send wall error byte mask
				msg_arg_arr[cham_i] = C[cham_i].bitWallErrorFlag;
	}

	//............... Send Ecat Ack ...............

	// I2C_FAILED
	if (i2c_status != 0)
		EsmaCom.writeEcatAck(EsmaCom.ErrorType::I2C_FAILED, msg_arg_arr, arg_len);

	// WALL_MOVE_FAILED
	else if (run_status > 1)
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

//------------------------ TESTING AND DEBUGGING METHODS ------------------------

/// @brief Used for testing the limit switch IO pins on a given Cypress chip.
/// @details This will loop indefinitely. Best to put this in the Arduino Setup() function.
///
/// @param cham_i Index/number of the chamber to set [0-48]
/// @param p_wall_inc OPTIONAL: Pointer array specifying wall index/number for wall(s) to test [0-7], max 8 entries. DEFAULT: all walls
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
	_Dbg.printMsg(_Dbg.MT::HEAD1, "RUNNING: Test IO switches: chamber[%d] walls%s", cham_i, _Dbg.arrayStr(p_wi, s));
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
				_Dbg.printMsg(_Dbg.MT::INFO, "\t Wall %d: down", wall_n);

			// Check up pins
			resp = _CypCom.ioReadPin(C[cham_i].addr, wms.ioUp[0][wall_n], wms.ioUp[1][wall_n], r_bit_out);
			if (resp != 0) // break out of loop if error returned
				break;
			if (r_bit_out == 1)
				_Dbg.printMsg(_Dbg.MT::INFO, "\t Wall %d: up", wall_n);

			// Add small delay
			delay(10);
		}
	}
	// Print failure message if while loop is broken out of because of I2C coms issues
	_Dbg.printMsg(_Dbg.MT::ERROR, "Test IO switches: chamber[%d] walls%s", cham_i, _Dbg.arrayStr(p_wi, s));
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
	_Dbg.printMsg(_Dbg.MT::HEAD1, "RUNNING: Test PWM: chamber[%d] walls%s", cham_i, _Dbg.arrayStr(p_wi, s));
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++)
	{ // loop walls
		uint8_t wall_n = p_wi[i];
		_Dbg.printMsg(_Dbg.MT::INFO, "\t Wall %d: Up", wall_n);
		resp = _CypCom.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 1); // run wall up
		if (resp != 0)
			return resp;
		delay(dt_run);
		_Dbg.printMsg(_Dbg.MT::INFO, "\t Wall %d: Down", wall_n);
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
	_Dbg.printMsg(_Dbg.MT::HEAD1, "RUNNING: Test move opperation: chamber[%d] walls%s", cham_i, _Dbg.arrayStr(p_wi, s));
	uint8_t r_bit_out = 1;
	uint16_t dt = 2000;
	uint16_t ts;
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++)
	{ // loop walls
		uint8_t wall_n = p_wi[i];
		_Dbg.printMsg(_Dbg.MT::INFO, "\t Moving wall %d", wall_n);

		// Run up
		_Dbg.printMsg(_Dbg.MT::INFO, "\t\t up start");
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
				_Dbg.printMsg(_Dbg.MT::INFO, "\t\t up end [%s]", _Dbg.dtTrack());
				break;
			}
			else if (millis() >= ts)
			{
				_Dbg.printMsg(_Dbg.MT::INFO, "\t\t !!up timedout [%s]", _Dbg.dtTrack());
				break;
			}
			delay(10);
		}

		// Run down
		_Dbg.printMsg(_Dbg.MT::INFO, "\t\t down start");
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
				_Dbg.printMsg(_Dbg.MT::INFO, "\t\t down end [%s]", _Dbg.dtTrack());
				break;
			}
			else if (millis() >= ts)
			{
				_Dbg.printMsg(_Dbg.MT::INFO, "\t\t !!down timedout [%s]", _Dbg.dtTrack());
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