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
#include <Esmacat_Com.h>
#include <Maze_Debug.h>
#include <Cypress_Com.h>
#include <Wall_Operation.h>

//============ VARIABLES ===============

// Global variables
bool DB_VERBOSE = 1;  //< set to control debugging behavior [0:silent, 1:verbose]
bool DO_ECAT_SPI = 1; //< set to control block SPI [0:dont start, 1:start]

// Wall opperation setup (these will be overwritten by the Ethercat message)
uint8_t nCham = 2;	   ///< number of chambers being used [1-9]
uint8_t nWallAttempt;  // number of attempts to move a walls [1-255]
uint8_t pwmDuty = 255; ///< PWM duty for all walls [0-255]

// Initialize class instances for local libraries
Maze_Debug Dbg;
Cypress_Com CypCom;
Wall_Operation WallOper(nCham, nWallAttempt, pwmDuty);

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
	Dbg.printMsg(Dbg.MT::ATTN, "FINISHED UPLOADING TO ARDUNO UNO");
#endif
#ifdef __AVR_ATmega2560__
	Dbg.printMsg(Dbg.MT::ATTN, "FINISHED UPLOADING TO ARDUNO MEGA");
#endif
#ifdef ARDUINO_SAM_DUE
	Dbg.printMsg(Dbg.MT::ATTN, "FINISHED UPLOADING TO ARDUNO DUE");
#endif

	// // Scan connected I2C devices
	// Dbg.printMsg(Dbg.MT::ATTN_START, "RUNNNING: I2C SCAN");
	// CypCom.i2cScan();
	// Dbg.printMsg(Dbg.MT::ATTN_END, "FINISHED: I2C SCAN");
}

//=============== LOOP ==================
void loop()
{

	//............... ROS Controlled ...............

	// // Check ethercat coms
	// WallOper.EsmaCom.readEcatMessage();

	// // Process and exicute ethercat arguments
	// WallOper.procEcatMessage();

	//............... Standalone Setup ...............
	static bool init = 0;
	if (!init)
	{
		init = 1;
		Dbg.printMsg(Dbg.MT::ATTN, "RUNNNING: STANDALONE SETUP");

		// Initalize Cypress Chips
		WallOper.initCypress();

		// WallOper.setWallMove(0, 1);
		// WallOper.moveWalls();
		// WallOper.setWallMove(0, 0);
		// WallOper.moveWalls();
		// while (true)
		// 	;

		// Initalize Walls
		WallOper.initWalls(1);

		Dbg.printMsg(Dbg.MT::ATTN_END, "FINISHED: STANDALONE SETUP");
	}

	//............... Cypress Testing ...............

	// // Test input pins
	// uint8_t a_wall[1] = { 2 };
	// resp = WallOper.testWallIO(0, a_wall, 1);
	// resp = WallOper.testWallIO(0);

	// // Test PWM output
	// uint8_t a_wall[1] = { 1 };
	// WallOper.testWallPWM(0, a_wall, 1);
	// while (true);

	// // Test wall opperation
	// uint8_t a_wall[2] = { 1, 3 };
	// WallOper.testWallOperation(0, a_wall, 2);
}