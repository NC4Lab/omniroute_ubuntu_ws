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
#include <Wall_Operation.h>

// TEMP COMMIT TEST

//============ VARIABLES ===============

// Global
extern bool DB_VERBOSE = 1; //<set to control debugging behavior [0:silent, 1:verbose]

// Local
uint8_t resp = 0;	   ///< capture I2C comm flags from Wire::method calls [0:success, 1-4:errors]
uint8_t nCham = 1;	   ///< number of chambers being used [1-49]
uint8_t pwmDuty = 200; ///< PWM duty for all walls [0-255]

// Initialize class instances for local libraries
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
		resp = C_COM.setupCypress(W_OPR.C[ch_i].addr);
		if (resp != 0)
			DB.printMsgTime("!!Failed Cypress setup: chamber=%d address=%s!!", ch_i, DB.hexStr(W_OPR.C[ch_i].addr));
		if (resp == 0 && ch_i == W_OPR.nCham - 1)
		{ // print success for last itteration
			DB.printMsgTime("Finished Cypress setup: chamber=%d address=%s", ch_i, DB.hexStr(W_OPR.C[ch_i].addr));
		}
	}

	// Setup IO pins for each chamber
	resp = W_OPR.setupWallIO();

	// Setup PWM pins for each chamber
	resp = W_OPR.setupWallPWM(pwmDuty);

	// Test and reset all walls
	// resp = W_OPR.initializeWalls();

	// // TEMP
	// int cnt = 0;
	// const uint8_t nbyte = 6;
	// byte io_all_reg_1[nbyte];
	// byte io_all_reg_2[nbyte];
	// while ((true))
	// {
	// 	cnt++;
	// 	DB.printMsgTime("\nLOOP NUMBER %d", cnt);
	// 	C_COM.ioReadReg(0, REG_GI0, io_all_reg_1, nbyte);
	// 	delay(5000);
	// 	C_COM.ioReadReg(0, REG_GI0, io_all_reg_2, nbyte);
	// 	DB.printRegByte(io_all_reg_1, io_all_reg_2, nbyte);
	// 	delay(5000);
	// }

	// // Test input pins
	// uint8_t a_wall[1] = { 1 };
	// resp = W_OPR.testWallIO(0, a_wall, 1);
	resp = W_OPR.testWallIO(0);

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

	// TEMP Move chambers
	// bool do_cham_arr[3] = {true, true, true};
	// uint32_t dt_timout = 1500;
	// const uint8_t s1 = 8;
	// uint8_t a_c1_wall[s1] = {0,1,2,3,4,5,6,7};
	// const uint8_t s2 = 1;
	// uint8_t a_c2_wall[s2] = {5};
	// const uint8_t s3 = 1;
	// uint8_t a_c3_wall[s3] = {5};
	// if (do_cham_arr[0])
	// 	W_OPR.setWallCmdManual(0, 1, a_c1_wall, s1);
	// if (do_cham_arr[1])
	// 	W_OPR.setWallCmdManual(1, 1, a_c2_wall, s2);
	// if (do_cham_arr[2])
	// 	W_OPR.setWallCmdManual(2, 1, a_c3_wall, s3);
	// resp = W_OPR.runWalls(dt_timout); // move walls up
	// // return;
	// if (do_cham_arr[0])
	// 	W_OPR.setWallCmdManual(0, 0, a_c1_wall, s1);
	// if (do_cham_arr[1])
	// 	W_OPR.setWallCmdManual(1, 0, a_c2_wall, s2);
	// if (do_cham_arr[2])
	// 	W_OPR.setWallCmdManual(2, 0, a_c3_wall, s3);
	// resp = W_OPR.runWalls(dt_timout); // move walls down
}

//=============== LOOP ==================
void loop()
{

	// Check ethercat coms
	resp = W_OPR.getWallCmdEthercat();

	// Wait for initialization
	if (!W_OPR.isEthercatInitialized)
		return;

	// Check for new wall move command
	if (resp == 1)
		W_OPR.runWalls(); // move walls
}
