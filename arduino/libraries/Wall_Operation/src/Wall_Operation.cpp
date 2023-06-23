//######################################

//======== Wall_Operation.cpp ==========

//######################################

/// <file>
/// Used for the Wall_Operation class
/// <file>

//============= INCLUDE ================
#include "Wall_Operation.h"

//========CLASS: Wall_Operation==========

/// <summary>
/// Constructor
/// </summary>
/// <param name="_nCham">Spcify number of chambers to track [1-49]</param>
Wall_Operation::Wall_Operation(uint8_t _nCham) {
	nCham = _nCham; //store number of chambers
	for (size_t ch_i = 0; ch_i < nCham; ch_i++) // update chamber struct entries
	{
		C[ch_i].num = ch_i;
		C[ch_i].addr = _C_COM.ADDR_LIST[ch_i];
	}
	// Create WallMapStruct lists for each function
	_makePMS(pmsAllIO, wms.ioDown[0], wms.ioDown[1], wms.ioUp[0], wms.ioUp[1]); //all io pins
	_makePMS(pmsAllPWM, wms.pwmDown[0], wms.pwmDown[1], wms.pwmUp[0], wms.pwmUp[1]); //all pwm pins
	_makePMS(pmsUpIO, wms.ioUp[0], wms.ioUp[1]); //io up pins
	_makePMS(pmsDownIO, wms.ioDown[0], wms.ioDown[1]); //io down pins
	_makePMS(pmsUpPWM, wms.pwmUp[0], wms.pwmUp[1]); //pwm up pins
	_makePMS(pmsDownPWM, wms.pwmDown[0], wms.pwmDown[1]); //pwm down pins
	// Update pin function map [0,1,2,3] [io down, io up, pwm down, pwm up]
	for (size_t i = 0; i < 8; i++) { //loop wall map entries
		wms.funMap[wms.ioDown[0][i]][wms.ioDown[1][i]] = 1; //label io down
		wms.funMap[wms.ioUp[0][i]][wms.ioUp[1][i]] = 2; //label io up
		wms.funMap[wms.pwmDown[0][i]][wms.pwmDown[1][i]] = 3; //label pwm down
		wms.funMap[wms.pwmUp[0][i]][wms.pwmUp[1][i]] = 4; //label pwm up
	}
}

//++++++++++++ Setup Methods +++++++++++++

/// <summary>
/// Build @ref Wall_Operation::PinMapStruct (PMS) structs, which are used to store information 
/// related to pin/port and wall mapping for specified functions (i.e., pwm, io, up, down)
/// Note, these methods are only used in the construtor
/// </summary>
/// <param name="r_pms">Reference to PMS to be updated</param>
/// <param name="p_port_1">Array of port values from an @ref Wall_Operation::WallMapStruct </param>
/// <param name="p_pin_1">Array of pin values from an @ref Wall_Operation::WallMapStruct</param>
void Wall_Operation::_makePMS(PinMapStruct& r_pms, uint8_t p_port_1[], uint8_t p_pin_1[]) {
	_resetPMS(r_pms);
	_addPortPMS(r_pms, p_port_1, p_pin_1);
	_addPinPMS(r_pms, p_port_1, p_pin_1);
}
/// <summary>
/// Overload with option for additional @ref Wall_Operation::WallMapStruct entries used
/// for creating PMS structs that include pins both up and down (e.g., all IO or all PWM pins)
/// </summary>
/// <param name="p_port_2">Array of port values from an @ref Wall_Operation::WallMapStruct </param>
/// <param name="p_pin_2">Array of pin values from an @ref Wall_Operation::WallMapStruct</param>
void Wall_Operation::_makePMS(PinMapStruct& r_pms, uint8_t p_port_1[], uint8_t p_pin_1[], uint8_t p_port_2[], uint8_t p_pin_2[]) {
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
void Wall_Operation::_addPortPMS(PinMapStruct& r_pms, uint8_t p_port[], uint8_t p_pin[]) {

	for (size_t wal_i = 0; wal_i < 8; wal_i++) { //loop port list by wall
		for (size_t prt_i = 0; prt_i < 6; prt_i++) { //loop port array in struct
			if (r_pms.port[prt_i] != p_port[wal_i] && r_pms.port[prt_i] != 255) continue; //find first emtpy  or existing entry and store there
			r_pms.nPorts = r_pms.port[prt_i] == 255 ? prt_i + 1 : r_pms.nPorts;//update length
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
void Wall_Operation::_addPinPMS(PinMapStruct& r_pms, uint8_t p_port[], uint8_t p_pin[]) {
	for (size_t prt_i = 0; prt_i < 6; prt_i++) { //loop ports in struct arr
		if (r_pms.port[prt_i] == 255) break;  //bail if reached end of list
		for (size_t wal_i = 0; wal_i < 8; wal_i++) { //loop wall list
			if (p_port[wal_i] != r_pms.port[prt_i]) continue; //check port match
			for (size_t pin_ii = 0; pin_ii < 8; pin_ii++) { //loop pin struct
				if (r_pms.pin[prt_i][pin_ii] != 255) continue; //find first emtpy entry and store there
				r_pms.nPins[prt_i] = pin_ii + 1;//update length
				r_pms.pin[prt_i][pin_ii] = p_pin[wal_i];
				r_pms.wall[prt_i][pin_ii] = wal_i;
				break;
			}
		}
		// Sort pin and wall array based on pin number
		_sortArr(r_pms.pin[prt_i], 8, r_pms.wall[prt_i]);

		// Update registry byte
		for (size_t pin_i = 0; pin_i < r_pms.nPins[prt_i]; pin_i++) {
			bitWrite(r_pms.byteMask[prt_i], r_pms.pin[prt_i][pin_i], 1); //update short/truncated version
		}
		r_pms.byteMaskLong[r_pms.port[prt_i]] = r_pms.byteMask[prt_i]; //update long/complete version
	}
}

/// <summary>
/// Sorts array values in assending order
/// Note, this is used exclusively by the above methods when creating the PMS structs.
/// </summary>
/// <param name="p_arr">Array to be sorted</param>
/// <param name="s">Length of array</param>
/// <param name="p_co_arr">OPTIONAL: array to be sorted in the same order as "p_arr"</param>
void Wall_Operation::_sortArr(uint8_t p_arr[], size_t s, uint8_t p_co_arr[]) {
	bool is_sorted = false;
	while (!is_sorted) {
		is_sorted = true;

		// Iterate over the array, swapping adjacent elements if they are out of order
		for (size_t i = 0; i < s - 1; ++i) {
			if (p_arr[i] > p_arr[i + 1]) {
				// Swap the elements without using the std::swap function
				uint8_t tmp1 = p_arr[i];
				p_arr[i] = p_arr[i + 1];
				p_arr[i + 1] = tmp1;
				// Update sort ind
				if (p_co_arr) {
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
/// Option to change the PWM duty cycle for a given wall.
/// Note, this is basically a wrapper for @ref Cypress_Com::setSourceDutyPWM.
/// Could be useful if some walls are running at different speeds.
/// </summary>
/// <param name="cham_i">Index of the chamber to set [0-48]</param>
/// <param name="source">Specifies one of 8 sources to set. See @ref Wall_Operation::wms.pwmSrc.</param>
/// <param name="duty">PWM duty cycle [0-255].</param>
/// <returns>Wire::method output [0-4] or [-1=255:input argument error].</returns>
uint8_t Wall_Operation::changeWallDutyPWM(uint8_t cham_i, uint8_t wall_i, uint8_t duty) {
	if (cham_i > 48 || wall_i > 7 || duty > 255) return -1;
	uint8_t resp = _C_COM.setSourceDutyPWM(C[cham_i].addr, wms.pwmSrc[wall_i], duty); //set duty cycle to duty
	return resp;
}

/// /// <summary>
/// Setup IO pins for each chamber including the pin direction (input) a
/// and the type of drive mode (high impedance).
/// Note, have to do some silly stuff with the output pins as well based on
/// page 11 of the Cypress datasheet "To  allow  input  operations  without
/// reconfiguration, these registers [output] have to store �1�s."
/// </summary>
/// <returns>Wire::method output from @ref Cypress_Com::method.</returns>
uint8_t Wall_Operation::setupWallIO() {
	uint8_t resp = 0;

	// Setup wall io pins
	for (size_t ch_i = 0; ch_i < nCham; ch_i++) {

		// Set entire output register to off
		uint8_t p_byte_mask_in[6] = { 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF };
		resp = _C_COM.ioWriteReg(C[ch_i].addr, p_byte_mask_in, 6, 0);
		if (resp != 0) return resp;

		for (size_t prt_i = 0; prt_i < pmsAllIO.nPorts; prt_i++) { //loop through port list

			// Set input pins as input
			resp = _C_COM.setPortRegister(C[ch_i].addr, REG_PIN_DIR, pmsAllIO.port[prt_i], pmsAllIO.byteMask[prt_i], 1);
			if (resp != 0) return resp;

			// Set pins as high impedance
			resp = _C_COM.setPortRegister(C[ch_i].addr, DRIVE_HIZ, pmsAllIO.port[prt_i], pmsAllIO.byteMask[prt_i], 1);
			if (resp != 0) return resp;

			// Set corrisponding output register entries to 1 as per datasheet
			resp = _C_COM.ioWritePort(C[ch_i].addr, pmsAllIO.port[prt_i], pmsAllIO.byteMask[prt_i], 1);
			if (resp != 0) return resp;
		}
	}
	return resp;
}

/// <summary>
/// Setup PWM pins for each chamber including the specifying them as PWM outputs 
/// and also detting the drive mode to "Strong Drive". This also sets up the PWM
/// Source using @ref Cypress_Com::setupSourcePWM()
/// </summary>
/// <param name="pwm_duty">Duty cycle for the pwm [0-255]</param>
/// <returns>Wire::method output from @ref Cypress_Com::method.</returns>
uint8_t Wall_Operation::setupWallPWM(uint8_t pwm_duty) {
	uint8_t resp = 0;

	// Setup pwm
	for (size_t ch_i = 0; ch_i < nCham; ch_i++) {

		// Setup source
		for (size_t src_i = 0; src_i < 8; src_i++) {
			resp = _C_COM.setupSourcePWM(C[ch_i].addr, wms.pwmSrc[src_i], pwm_duty);
			if (resp != 0) return resp;
		}

		// Setup wall pwm pins
		for (size_t prt_i = 0; prt_i < pmsAllPWM.nPorts; prt_i++) { //loop through port list

			// Set pwm pins as pwm output
			resp = _C_COM.setPortRegister(C[ch_i].addr, REG_SEL_PWM_PORT_OUT, pmsAllPWM.port[prt_i], pmsAllPWM.byteMask[prt_i], 1);
			if (resp != 0) return resp;

			// Set pins as strong drive
			resp = _C_COM.setPortRegister(C[ch_i].addr, DRIVE_STRONG, pmsAllPWM.port[prt_i], pmsAllPWM.byteMask[prt_i], 1);
			if (resp != 0) return resp;
		}
	}
	return resp;
}

/// <summary>
/// Reset all walls to their down state using @ref Wall_Operation::setWallCmdManual() 
/// and @ref Wall_Operation::runWalls()
/// </summary>
uint8_t Wall_Operation::resetAllWalls() {

	// Run all walls to down position
	uint8_t resp = 0;
	for (size_t ch_i = 0; ch_i < nCham; ch_i++) {
		C[ch_i].byteWallPosition = 0xFF; //change wall setting so all walls are considered up
		resp = setWallCmdManual(ch_i, 0); //use command to set all walls to be lowered
		if (resp != 0) return resp;
		resp = runWalls(); //move walls
		if (resp != 0) return resp;
	}
	return resp;
}

//++++++++++++ Runtime Methods +++++++++++++

/// <summary>
/// Resets most entries in a dynamic PMS struct to there default values.
/// </summary>
/// <param name="r_pms">Reference to PMS struct to be reset</param>
void Wall_Operation::_resetPMS(PinMapStruct& r_pms) {
	r_pms.nPorts = 0;
	for (size_t prt_i = 0; prt_i < 6; prt_i++) {
		r_pms.port[prt_i] = 255;
		r_pms.byteMask[prt_i] = 0;
		r_pms.byteMaskLong[prt_i] = 0;
		r_pms.nPins[prt_i] = 0;
		for (size_t pin_i = 0; pin_i < 8; pin_i++) {
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
void Wall_Operation::_updateDynamicPMS(PinMapStruct r_pms1, PinMapStruct& r_pms2, uint8_t wall_byte_mask) {
	for (size_t prt_i = 0; prt_i < r_pms1.nPorts; prt_i++) { //loop ports
		for (size_t pin_i = 0; pin_i < r_pms1.nPins[prt_i]; pin_i++) { //loop pins
			if (bitRead(wall_byte_mask, r_pms1.wall[prt_i][pin_i]) == 1) { //check if wall is on this port

				//Copy all struct fields
				for (size_t prt_ii = 0; prt_ii < 6; prt_ii++) { //loop ports
					if (r_pms2.port[prt_ii] != r_pms1.port[prt_i] && r_pms2.port[prt_ii] != 255) continue; //find first emtpy  or existing entry and store there
					r_pms2.nPorts = r_pms2.port[prt_ii] == 255 ? r_pms2.nPorts + 1 : r_pms2.nPorts; //update port count for new entry
					r_pms2.nPins[prt_ii]++; //update pin count
					r_pms2.port[prt_ii] = r_pms1.port[prt_i]; //update port number
					for (size_t pin_ii = 0; pin_ii < 8; pin_ii++) { //loop pins
						if (r_pms2.pin[prt_ii][pin_ii] != 255) continue; //find first emtpy entry and store there
						r_pms2.pin[prt_ii][pin_ii] = r_pms1.pin[prt_i][pin_i]; //update pin number
						r_pms2.wall[prt_ii][pin_ii] = r_pms1.wall[prt_i][pin_i]; //update wall number
						bitWrite(r_pms2.byteMask[prt_ii], r_pms2.pin[prt_ii][pin_ii], 1); //update short/truncated registry
						r_pms2.byteMaskLong[r_pms2.port[prt_ii]] = r_pms2.byteMask[prt_ii]; //update long/complete version
						break;
					}
					break;
				}
			}
		}
	}
}

/// <summary>
/// Used to get incoming serial data signalling which walls to raise.
/// Note, this can be used in conjunction with another microcontroller using
/// MazeWallOperation\arduino\SendWallMessage\SendWallMessage.ino
/// </summary>
/// <returns>Success/error codes [0:success, 1-4:error] associated with serial data</returns>
uint8_t Wall_Operation::getWallCmdSerial() {
	static const uint16_t buff_max = 500;
	uint16_t buff[500] = { 0 }; buff[0] = '\0';
	uint16_t byte_in_ind = 0;
	uint16_t byte_store_ind = 0;
	uint32_t ts_read = millis() + 500;

	//// Bail if no message
	if (Serial1.available() == 0) { return 1; }

	// Read data stream
	while (millis() < ts_read) {
		if (Serial1.available() == 0) continue; //wait for new serial
		buff[byte_in_ind] = Serial1.read();
		byte_in_ind++;
		//delay(1);
		if (byte_in_ind >= buff_max) {
			_DB.printMsg("!!com buffer overflow: bytes > %d!!", buff_max);
			return 1;
		}
		if (buff[byte_in_ind - 1] == 255) break; //check for footer
	}
	if (buff[byte_in_ind - 1] != 255) {
		_DB.printMsg("!!timed out before footer!!");
		return 2;
	}

	// Parse message
	if (buff[byte_store_ind++] != 254) { //check for header
		_DB.printMsg("!!missing serial header!!");
		return 3;
	}
	uint8_t n_cham = buff[byte_store_ind++]; //get number of chambers to set
	uint8_t cham[n_cham];
	uint8_t wall_byte[n_cham];
	for (size_t ch_i = 0; ch_i < n_cham; ch_i++) { //loop through each chamber message
		cham[ch_i] = buff[byte_store_ind++];
		wall_byte[ch_i] = buff[byte_store_ind++];
	}
	if (buff[byte_store_ind++] != 255) { //check for footer
		_DB.printMsg("!!missing serial footer!!");
		return 4;
	}

	// Store values and update the update flag
	for (size_t ch_i = 0; ch_i < n_cham; ch_i++) { //loop through each chamber message
		C[cham[ch_i]].byteWallActive = wall_byte[ch_i]; //store values
		C[cham[ch_i]].byteUpdateFlag = 1; //set update flag
	}

	// Return new message flag
	return 0;
}

/// <summary>
/// Option to speicify a given set of walls to move programmatically as an
/// alternative to @ref Wall_Operation::getWallCmdSerial.
/// Note, this function updates the byte mask specifying which walls should be up
/// </summary>
/// <param name="cham_i">Index of the chamber to set [0-48]</param>
/// <param name="bit_val_set">Value to set the bits to [0,1]. DEFAULT[1].</param>
/// <param name="p_wall_inc">OPTIONAL: pointer array specifying wall numbers for walls to move [0-7] max 8 entries. DEFAULT:[all walls]</param>
/// <param name="s">OPTIONAL: length of "p_wall_inc". DEFAULT:[8] </param>
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
/// Wall_Operation::runWalls(); //note that @ref Wall_Operation::runWalls() must be run after all settings complete
/// </code>
/// </example>
uint8_t Wall_Operation::setWallCmdManual(uint8_t cham_i, uint8_t bit_val_set, uint8_t p_wall_inc[], uint8_t s) {
	if (s > 8) return -1;
	uint8_t p_wi[s]; //initialize array to handle null array argument
	if (p_wall_inc == nullptr) { //set default 8 walls
		for (size_t i = 0; i < s; i++) p_wi[i] = i;
	}
	else { // copy over input
		for (size_t i = 0; i < s; i++) p_wi[i] = p_wall_inc[i];
	}
	for (size_t i = 0; i < s; i++) { //loop through each entry
		bitWrite(C[cham_i].byteWallActive, p_wi[i], bit_val_set); //set bit for newly active wall
		C[cham_i].byteUpdateFlag = 1; //flag update
	}
	return 0;
}

/// <summary>
/// The main workhorse of the class, which mannages initiating and compleating 
/// the wall movement for all active chambers based on either 
/// @ref Wall_Operation::getWallCmdSerial or @ref Wall_Operation::getWallCmdSerial.
/// </summary>
/// <param name="dt_timout">Time to attemp movement (ms) DEFAULT:[1500]</param>
/// <returns>Success/error codes [0:success, 1:fail:unspecified, 2:fail:i2c, 3:fail:timeout]</returns>
uint8_t Wall_Operation::runWalls(uint32_t dt_timout) {
	// Local vars
	uint8_t resp = 0;
	uint8_t run_error = 0; //store run errors
	uint8_t is_timedout = 0; //flag timeout
	uint8_t do_move_check = 1; //will track if all chamber movement done
	uint32_t ts_timeout = millis() + dt_timout; //set timout 

	// Update dynamic port/pin structs
	_DB.dtTrack(1); //start timer
	for (size_t ch_i = 0; ch_i < nCham; ch_i++) { //loop chambers
		if (C[ch_i].byteUpdateFlag != 1) continue; //check if chamber flagged for updating

		// Reset structs
		_resetPMS(C[ch_i].pmsDynPWM);
		_resetPMS(C[ch_i].pmsDynIO);

		// Get byte mask for walls that should be moved and update dynamic PMS struct
		uint8_t wall_up_byte_mask = ~C[ch_i].byteWallPosition & C[ch_i].byteWallActive; //up (wall_state == 0 & wall_active == 1) 
		uint8_t wall_down_byte_mask = C[ch_i].byteWallPosition & ~C[ch_i].byteWallActive; //down (wall_state == 1 & wall_active == 0)
		_updateDynamicPMS(pmsDownIO, C[ch_i].pmsDynIO, wall_down_byte_mask); //pwm down
		_updateDynamicPMS(pmsUpIO, C[ch_i].pmsDynIO, wall_up_byte_mask); //pwm up
		_updateDynamicPMS(pmsDownPWM, C[ch_i].pmsDynPWM, wall_down_byte_mask); //io down
		_updateDynamicPMS(pmsUpPWM, C[ch_i].pmsDynPWM, wall_up_byte_mask); //io up

		// Move walls up/down
		resp = _C_COM.ioWriteReg(C[ch_i].addr, C[ch_i].pmsDynPWM.byteMaskLong, 6, 1);

		// Print walls being moved
		if (wall_up_byte_mask > 0) _DB.printMsgTime("Start move up: chamber=%d walls=%s", ch_i, _DB.bitIndStr(wall_up_byte_mask));
		if (wall_down_byte_mask > 0) _DB.printMsgTime("Start move down: chamber=%d walls=%s", ch_i, _DB.bitIndStr(wall_down_byte_mask));
	}

	// Check IO pins
	while (!is_timedout && do_move_check) { //loop till finished or timed out
		do_move_check = 0; //reset
		is_timedout = millis() >= ts_timeout; //check for timeout
		for (size_t ch_i = 0; ch_i < nCham; ch_i++) { //loop chambers
			do_move_check += C[ch_i].byteUpdateFlag; //update flag
			//_DB.printMsg("ch_i=%d do_move_check=%d is_timedout=%d", ch_i, do_move_check, is_timedout); //TEMP
			if (C[ch_i].byteUpdateFlag != 1) continue; //check if chamber flagged for updating

			// Get io registry bytes. Note we are reading out the output registry as well to save an additional read if we write the pwm output later
			U.i64[0] = 0; U.i64[1] = 0; //zero out union values
			resp = _C_COM.ioReadReg(C[ch_i].addr, REG_GI0, U.b, 14); //read through all input registers (6 active, 2 unused) and the 6 active output registers
			if (resp != 0) { forceStopWalls(); run_error = 1; } //force stop and set flag
			uint8_t io_in_reg[6] = { U.b[0], U.b[1] , U.b[2] , U.b[3] , U.b[4] , U.b[5] }; //copy out values
			uint8_t io_out_reg[6] = { U.b[8], U.b[9] , U.b[10] , U.b[11] , U.b[12] , U.b[13] }; //copy out values
			uint8_t io_out_mask[6] = { 0 }; //will store new output pwm reg mask 

			// Compare check ifcurrent registry io to saved registry 
			uint8_t f_track_reg = 0; //will track if all io pins have been been triggered 
			uint8_t f_do_pwm_update = 0; //will track if pwm registry should be updated
			for (size_t prt_i = 0; prt_i < C[ch_i].pmsDynIO.nPorts; prt_i++) { //loop ports

				// Compare current io registry to saved mask
				uint8_t port_n = C[ch_i].pmsDynIO.port[prt_i]; //get port number
				uint8_t comp_byte = C[ch_i].pmsDynIO.byteMask[prt_i] & io_in_reg[port_n]; //triggered pin/bit matching mask will be 1
				f_track_reg += C[ch_i].pmsDynIO.byteMask[prt_i]; //update flag

				// Check each bit in comparison byte
				for (size_t pin_i = 0; pin_i < C[ch_i].pmsDynIO.nPins[prt_i]; pin_i++) { //loop pins
					uint8_t pin_n = C[ch_i].pmsDynIO.pin[prt_i][pin_i]; //get pin number
					if (!bitRead(C[ch_i].pmsDynIO.byteMask[prt_i], pin_n)) continue; //check if pin still flagged in reg byte
					if (!bitRead(comp_byte, pin_n)) continue; //check io pin
					bitWrite(C[ch_i].pmsDynIO.byteMask[prt_i], pin_n, 0); //remove from byte reg

					// Update pwm registry array (note, sets both up and down pwm reg entries as this makes the code easier)
					uint8_t wall_n = C[ch_i].pmsDynIO.wall[prt_i][pin_i]; //get wall number
					bitWrite(io_out_mask[wms.pwmDown[0][wall_n]], wms.pwmDown[1][wall_n], 1);
					bitWrite(io_out_mask[wms.pwmUp[0][wall_n]], wms.pwmUp[1][wall_n], 1);
					f_do_pwm_update = 1; // flag to update pwm

					// Update state [0,1] [down,up] based on active switch function [1:io_down, 2:io_up]
					uint8_t swtch_fun = wms.funMap[port_n][pin_n]; // get wall state
					bitWrite(C[ch_i].byteWallPosition, wall_n, swtch_fun == 1 ? 0 : 1);

					_DB.printMsgTime("End move %s: chamber=%d wall=%d dt=%s", swtch_fun == 1 ? "down" : "up", ch_i, wall_n, _DB.dtTrack());
				}
			}

			// Send pwm off command
			if (f_do_pwm_update) { //check for update flag
				resp = _C_COM.ioWriteReg(C[ch_i].addr, io_out_mask, 6, 0, io_out_reg); //include last reg read and turn off pwms
				if (resp != 0) { forceStopWalls(); run_error = 1; } //force stop and set flag
			}
			C[ch_i].byteUpdateFlag = f_track_reg == 0 ? 0 : C[ch_i].byteUpdateFlag; //reset flag

		}
	}

	// Check for any unifinished moves
	for (size_t ch_i = 0; ch_i < nCham; ch_i++) { //loop chambers
		if (C[ch_i].byteUpdateFlag != 1) { //check if chamber still flagged for updating
			C[ch_i].byteErrorFlag = 0; //reset error flags
			continue;
		}
		for (size_t prt_i = 0; prt_i < C[ch_i].pmsDynIO.nPorts; prt_i++) { //loop ports
			for (size_t pin_i = 0; pin_i < C[ch_i].pmsDynIO.nPins[prt_i]; pin_i++) { //loop pins

				// Check if pin still flagged for movement
				uint8_t pin_n = C[ch_i].pmsDynIO.pin[prt_i][pin_i]; //get pin number
				if (!bitRead(C[ch_i].pmsDynIO.byteMask[prt_i], pin_n)) continue; //check if pin still flagged in reg byte

				// Flag error for debugging and to stop all wall movement
				uint8_t wall_n = C[ch_i].pmsDynIO.wall[prt_i][pin_i]; //get wall number
				run_error = is_timedout && run_error != 1 ? 2 : 3; //set error flag
				_DB.printMsg("!!Movement failed: chamber=%d wall=%d cause=%s dt=%s!!", ch_i, wall_n,
					run_error == 1 ? "i2c" : run_error == 2 ? "timedout" : "unknown", _DB.dtTrack());
				bitWrite(C[ch_i].byteErrorFlag, wall_n, 1); //set chamber/wall specific error flag
			}
		}
	}

	// Force stop all walls if error encountered
	if (run_error != 0) resp = forceStopWalls();
	else _DB.printMsgTime("Finished all movement");

	return run_error; //return error
}

/// <summary>
/// Sends a command to stop all walls by setting all PWM outputs to zero.
/// </summary>
/// <returns>Wire::method output [0-4].</returns>
uint8_t Wall_Operation::forceStopWalls() {
	uint8_t resp = 0;
	_DB.printMsgTime("!!Running forse stop!!");
	for (size_t ch_i = 0; ch_i < nCham; ch_i++) { //loop chambers
		resp = _C_COM.ioWriteReg(C[ch_i].addr, pmsAllPWM.byteMaskLong, 6, 0); //stop all pwm output
		if (resp != 0) return resp;
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
uint8_t Wall_Operation::testWallIO(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s) {
	if (cham_i > 48 || s > 8) return -1;
	uint8_t p_wi[s]; //initialize array to handle null array argument
	if (p_wall_inc == nullptr) { //set default 8 walls
		for (size_t i = 0; i < s; i++) p_wi[i] = i;
	}
	else { // copy over input
		for (size_t i = 0; i < s; i++) p_wi[i] = p_wall_inc[i];
	}

	// Test input pins
	uint8_t r_bit_out;
	uint8_t resp = 0;
	_DB.printMsg("Testing limit switches for wall number %s", _DB.arrayStr(p_wi, s));
	while (true) { // loop indefinitely
		for (size_t i = 0; i < s; i++) { //loop walls
			uint8_t wall_n = p_wi[i];
			// Check down pins
			uint8_t resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioDown[0][wall_n], wms.ioDown[1][wall_n], r_bit_out);
			if (resp != 0) return resp;
			if (r_bit_out == 1) {
				_DB.printMsg("\tWall %d: Down", wall_n);
			}
			// Check up pins
			resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioUp[0][wall_n], wms.ioUp[1][wall_n], r_bit_out);
			if (resp != 0) return resp; // TEMP
			if (r_bit_out == 1) {
				_DB.printMsg("\tWall %d: Up", wall_n);
			}
			delay(10); //add small delay
		}
	}
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
uint8_t Wall_Operation::testWallPWM(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s, uint32_t dt_run) {
	if (cham_i > 48 || s > 8) return -1;
	uint8_t p_wi[s];
	if (p_wall_inc == nullptr) { //set default 8 walls
		for (size_t i = 0; i < s; i++) p_wi[i] = i;
	}
	else { // copy over input
		for (size_t i = 0; i < s; i++) p_wi[i] = p_wall_inc[i];
	}

	// Run each wall up then down for dt_run ms
	_DB.printMsg("Testing PWM for wall number %s", _DB.arrayStr(p_wi, s));
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++) { //loop walls
		uint8_t wall_n = p_wi[i];
		_DB.printMsg("\tWall %d: Up", wall_n);
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 1); //run wall up
		if (resp != 0) return resp;
		delay(dt_run);
		_DB.printMsg("\tWall %d: Down", wall_n);
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 1); //run wall down (run before so motoro hard stops)
		if (resp != 0) return resp;
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 0); //stop wall up pwm
		if (resp != 0) return resp;
		delay(dt_run);
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 0); //stop wall down pwm
		if (resp != 0) return resp;
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
uint8_t Wall_Operation::testWallOperation(uint8_t cham_i, uint8_t p_wall_inc[], uint8_t s) {
	if (cham_i > 48 || s > 8) return -1;
	uint8_t p_wi[s];
	if (p_wall_inc == nullptr) { //set default 8 walls
		for (size_t i = 0; i < s; i++) p_wi[i] = i;
	}
	else { // copy over input
		for (size_t i = 0; i < s; i++) p_wi[i] = p_wall_inc[i];
	}

	// Test all walls
	_DB.printMsg("Testing move opperation for wall number %s", _DB.arrayStr(p_wi, s));
	uint8_t r_bit_out = 1;
	uint16_t dt = 2000;
	uint16_t ts;
	uint8_t resp = 0;
	for (size_t i = 0; i < s; i++) { //loop walls
		uint8_t wall_n = p_wi[i];
		_DB.printMsg("Running wall %d", wall_n);

		// Run up
		_DB.printMsg("\tUp start");
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 1);
		if (resp != 0) return resp;
		ts = millis() + dt; //set timeout
		_DB.dtTrack(1); //start timer
		while (true) { //check up switch
			resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioUp[0][wall_n], wms.ioUp[1][wall_n], r_bit_out);
			if (resp != 0) return resp;
			if (r_bit_out == 1) {
				_DB.printMsg("\tUp end [%s]", _DB.dtTrack());
				break;
			}
			else if (millis() >= ts) {
				_DB.printMsg("\t!!Up timedout [%s]!!", _DB.dtTrack());
				break;
			}
			delay(10);
		}

		// Run down
		_DB.printMsg("\tDown start");
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 1);
		if (resp != 0) return resp;
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmUp[0][wall_n], wms.pwmUp[1][wall_n], 0);
		if (resp != 0) return resp;
		ts = millis() + dt; //set timeout
		_DB.dtTrack(1); //start timer
		while (true) { //check up switch
			resp = _C_COM.ioReadPin(C[cham_i].addr, wms.ioDown[0][wall_n], wms.ioDown[1][wall_n], r_bit_out);
			if (resp != 0) return resp;
			if (r_bit_out == 1) {
				_DB.printMsg("\tDown end [%s]", _DB.dtTrack());
				break;
			}
			else if (millis() >= ts) {
				_DB.printMsg("\t!!Down timedout [%s]!!", _DB.dtTrack());
				break;
			}
			delay(10);
		}
		resp = _C_COM.ioWritePin(C[cham_i].addr, wms.pwmDown[0][wall_n], wms.pwmDown[1][wall_n], 0);
		if (resp != 0) return resp;

		// Pause
		delay(500);
	}
	return resp;
}

/// <summary>
/// Used for debugging to print out all fields of a PMS struct.
/// </summary>
/// <param name="pms">PMS struct to print</param>
void Wall_Operation::_printPMS(PinMapStruct pms) {
	char buff[250];
	sprintf(buff, "\nIO/PWM nPorts=%d__________________", pms.nPorts);
	_DB.printMsg(buff);
	for (size_t prt_i = 0; prt_i < pms.nPorts; prt_i++) {
		sprintf(buff, "port[%d] nPins=%d byteMask=%s", pms.port[prt_i], pms.nPins[prt_i], _DB.binStr(pms.byteMask[prt_i]));
		_DB.printMsg(buff);
		for (size_t pin_i = 0; pin_i < pms.nPins[prt_i]; pin_i++) {
			sprintf(buff, "   wall=%d   pin=%d", pms.wall[prt_i][pin_i], pms.pin[prt_i][pin_i]);
			_DB.printMsg(buff);
		}
	}
}
