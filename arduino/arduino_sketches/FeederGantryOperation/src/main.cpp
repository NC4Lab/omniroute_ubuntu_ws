// ######################################

//====== FeederGantryOperation.ino ======

// ######################################

/**
* @file Main Arduino INO file for running the maze.
*/

// BUILT IN
#include "Arduino.h"
#undef max
#undef min
#undef bit
#include "grbl.h"

// CUSTOM
#include <EsmacatCom.h>
#include <MazeDebug.h>


//=============== SETUP =================
void setup()
{
	// Setup serial coms
	Serial.begin(115200);
	Serial1.begin(115200);
	delay(100);
	Serial.print('\n');
}

//=============== LOOP ==================
void loop()
{
}