// #################################

// =========== main.ino ============

// #################################

/**
 @file Arduino INO file for running MazeGantryOpperation.
*/

// BUILT IN
#include "Arduino.h"
#include <Servo.h>

// LOCAL
#include <MazeDebug.h>
#include <EsmacatCom.h>
#include <GantryOperation.h>

//============ VARIABLES ===============

// Global variables
bool DB_VERBOSE = 1;  // set to control debugging behavior [0:silent, 1:verbose]
bool DO_ECAT_SPI = 1; // set to control block SPI [0:dont start, 1:start]

// Initialize class instances for local libraries
MazeDebug Dbg;         // instance of MazeDebug class for debugging messages
GantryOperation GanOp; // instance of GantryOperation class

//=============== SETUP =================
void setup()
{
  // Setup serial coms
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(100);

  // Print setup started
  Dbg.printMsg(Dbg.MT::HEAD1, "RUNNING SETUP");

  // Initialize servos
  GanOp.servoInit();
  delay(1000);

  // Print setup complete
  Dbg.printMsg(Dbg.MT::HEAD1, "SETUP COMPLETE");

  // // TEMP
  // for (int i = 0; i < 100; i++)
  // {
  //   GanOp.gantryMove(5.0, 5.0);
  // }
}

//=============== LOOP ==================
void loop()
{
  // TEMP
  Dbg.dtTrack(1);

  // Check ethercat coms
  if (!GanOp.EsmaCom.readEcatMessage())
    return;

  // Process and execute ethercat arguments
  GanOp.procEcatMessage();
  Dbg.printMsg(Dbg.MT::INFO, "[loop] dt[%s]", Dbg.dtTrack());
}