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
uint8_t nCham = 2;	   ///< number of chambers being used [1-49]
uint8_t pwmDuty = 200; ///< PWM duty for all walls [0-255]

// Initialize class instances for local libraries
Maze_Debug DB;
Cypress_Com C_COM;
Wall_Operation W_OPR(nCham, pwmDuty);

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

	// Print which microcontroller is active
#ifdef ARDUINO_AVR_UNO
	DB.printMsg("Finished uploading to Arduno Uno");
#endif
#ifdef __AVR_ATmega2560__
	DB.printMsg("Finished uploading to Arduno Mega");
#endif
#ifdef ARDUINO_SAM_DUE
	DB.printMsg("Finished uploading to Arduno Due");
#endif

	// Run initial maze setup
	// W_OPR.resetMaze(false);

	// // TEMP
	// W_OPR.sendEthercatMessage(W_OPR.A2P_Type_ID::CONFIRM_RECEIVED);
	// while (true);

	// // TEMP
	// while(true)
	// {
	// 	W_OPR.printEtherReg(0);
	// 	delay(1000);
	// }

}

//=============== LOOP ==================
void loop()
{

	// Check ethercat coms
	resp = W_OPR.getEthercatMessage();

	// Wait for initialization message
	if (!W_OPR.isEthercatInitialized)
		return;

	// Setup maze
	if (W_OPR.p2aEtherMsgType == Wall_Operation::P2A_Type_ID::START_SESSION)
		//W_OPR.resetMaze(false);

	// Reset maze
	if (W_OPR.p2aEtherMsgType == Wall_Operation::P2A_Type_ID::END_SESSION)
		//W_OPR.resetMaze(true);

	// Check for new wall move command
	if (W_OPR.p2aEtherMsgType == Wall_Operation::P2A_Type_ID::MOVE_WALLS)
		//W_OPR.moveWalls(); // move walls

	// // Test input pins
	// uint8_t a_wall[1] = { 2 };
	// resp = W_OPR.testWallIO(0, a_wall, 1);
	// resp = W_OPR.testWallIO(0);

	// // Test PWM output
	// uint8_t a_wall[1] = { 1 };
	// W_OPR.testWallPWM(0, a_wall, 1);
	// while (true);

	// // Test wall opperation
	// uint8_t a_wall[2] = { 1, 3 };
	// W_OPR.testWallOperation(0, a_wall, 2);
	// while (true);//TEMP
}