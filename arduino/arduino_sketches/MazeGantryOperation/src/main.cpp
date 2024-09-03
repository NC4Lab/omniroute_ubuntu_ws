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

  //   // TEMP
  // GanOp.grblWrite("F30000", false);
  // GanOp.grblWrite("$110", false);
  // while (Serial1.available() > 0)
  // {
  //   Serial.write(Serial1.read());
  // }

  // // TEMP
  // GanOp.grblInit();
  // GanOp.gantryHome();
  // //GanOp.gantryMove(500.0, 500.0);
  // for (int i = 0; i < 50; i++)
  // {
  //   GanOp.gantryMove(5.0, 5.0);
  //   //delay(10);
  //   //Serial1.println("$J=G91 G21 X5.00 Y5.00 F25000"); 
  // }

  // Print setup complete
  Dbg.printMsg(Dbg.MT::HEAD1, "SETUP COMPLETE");
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