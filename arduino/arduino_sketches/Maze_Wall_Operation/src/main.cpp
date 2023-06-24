// ######################################

//====== Maze_Wall_Operation.ino ======

// ######################################

/// <file>
/// Main Arduino INO file for running the maze.
/// <file>

// BUILT IN
#include "Arduino.h"
#include <Wire.h>

// CUSTOM
#include <Safe_Vector.h>
#include <Maze_Debug.h>
#include <Cypress_Com.h>
#include <Cypress_Com_Base.h>
#include <Wall_Operation.h>

//============ VARIABLES ===============

// Global
extern bool DB_VERBOSE = 1; //<set to control debugging behavior [0:silent, 1:verbose]

// Local
uint8_t resp = 0;	   ///< capture I2C comm flags from Wire::method calls [0:success, 1-4:errors]
uint8_t nCham = 3;	   ///< number of chambers being used [1-49]
uint8_t pwmDuty = 180; ///< PWM duty for all walls [0-255]

// Initialize struct and class instances
Maze_Debug DB;
Cypress_Com C_COM;
Wall_Operation W_OPR(nCham);

//=============== SETUP =================
void setup()
{

	// Setup serial coms
	Serial.begin(115200);
	Serial1.begin(115200);
	Wire.begin(); // join I2C bus
	delay(100);
	Serial.print('\n');
	pinMode(LED_BUILTIN, OUTPUT);
	DB.printMsgTime("SETUP START");

	// TEMP
	// C_COM.i2cScan();
	// return;

	// TEMP
	// const uint8_t _a_arr[5] = { 25,22,28, 29, 20 };
	// const S_VEC<uint8_t> v_arr(5, _a_arr);
	// v_arr.sort(true);
	// DB.TEMP_foo(v_arr);
	// return;

	// Print which microcontroller is active
#ifdef ARDUINO_AVR_UNO
	DB.printMsg("Uploading to Arduno Uno");
#endif
#ifdef __AVR_ATmega2560__
	DB.printMsg("Uploading to Arduno Mega");
#endif
#ifdef ARDUINO_SAM_DUE
	DB.printMsg("Uploading to Arduno Due");
#endif

	// Setup cypress chips
	for (size_t ch_i = 0; ch_i < W_OPR.nCham; ch_i++)
	{
		// setup cypress chip
		resp = C_COM.setupCypress(W_OPR.C[ch_i].addr);
		if (resp != 0)
			DB.printMsgTime("!!Failed Cypress setup: chamber=%d address=%s!!", ch_i, DB.hexStr(W_OPR.C[ch_i].addr));
		if (resp == 0 && ch_i == W_OPR.nCham - 1)
			;
		{ // print success for last itteration
			DB.printMsgTime("Finished Cypress setup: chamber=%d address=%s", ch_i, DB.hexStr(W_OPR.C[ch_i].addr));
		}
	}

	// Setup IO pins for each chamber
	resp = W_OPR.setupWallIO();

	// Setup PWM pins for each chamber
	resp = W_OPR.setupWallPWM(pwmDuty);

	// // Test and reset all walls
	// resp = W_OPR.initializeWalls();
	// while(true); // TEMP

	// // TEMP
	// while (true)
	// {
	// 	uint8_t r_bit_out;
	// 	uint8_t wall = 2;
	// 	C_COM.ioReadPin(W_OPR.C[0].addr, W_OPR.wms.ioDown[0][wall], W_OPR.wms.ioDown[1][wall], r_bit_out);
	// 	if (r_bit_out != 0)
	// 	{
	// 		Serial.println(r_bit_out);
	// 	}
	// }

	// // Test input pins
	// uint8_t a_wall[1] = { 1 };
	// resp = W_OPR.testWallIO(0, a_wall, 1);

	// // Test PWM output
	// uint8_t a_wall[1] = { 1 };
	// W_OPR.testWallPWM(0, a_wall, 1);
	// while (true);

	// // Test wall opperation
	// uint8_t a_wall[2] = { 1, 3 };
	// W_OPR.testWallOperation(0, a_wall, 2);
	// while (true);//TEMP

	// Print done
	DB.printMsgTime("SETUP DONE");

	// // Move chambers
	// bool do_cham_arr[3] = {true, false, false};
	// uint32_t dt_timout = 1500;
	// uint8_t c1 = 0;
	// uint8_t a_c1_wall[2] = {1, 2};
	// uint8_t c2 = 1;
	// uint8_t a_c2_wall[1] = {5};
	// uint8_t c3 = 2;
	// uint8_t a_c3_wall[1] = {5};
	// if (do_cham_arr[0])
	// 	W_OPR.setWallCmdManual(c1, 1, a_c1_wall, 2);
	// if (do_cham_arr[1])
	// 	W_OPR.setWallCmdManual(c2, 1, a_c2_wall, 1);
	// if (do_cham_arr[2])
	// 	W_OPR.setWallCmdManual(c3, 1, a_c3_wall, 1);
	// resp = W_OPR.runWalls(dt_timout); // move walls up
	// // return;
	// if (do_cham_arr[0])
	// 	W_OPR.setWallCmdManual(c1, 0, a_c1_wall, 1);
	// if (do_cham_arr[1])
	// 	W_OPR.setWallCmdManual(c2, 0, a_c2_wall, 1);
	// if (do_cham_arr[2])
	// 	W_OPR.setWallCmdManual(c3, 0, a_c3_wall, 1);
	// resp = W_OPR.runWalls(dt_timout); // move walls down

}

//=============== LOOP ==================
void loop()
{

	// // Check for new message
	// if (W_OPR.getWallCmdEthercat() == 0)
	// {
	// 	W_OPR.runWalls(); // move walls
	// }
}
