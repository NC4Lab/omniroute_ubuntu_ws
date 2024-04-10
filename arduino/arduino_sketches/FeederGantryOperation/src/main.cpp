// ######################################

//====== FeederGantryOperation.ino ======

// ######################################

/**
 * @file Main Arduino INO file for running the maze.
 */

// BUILT IN
#include "Arduino.h"
#include <Servo.h>

// LOCAL
#include <MazeDebug.h>
#include <FeederServo.h>
#include <EsmacatCom.h>

//============ VARIABLES ===============

// Initialize class instances for local libraries
MazeDebug Dbg;
FeederServo FdSrv;

//=============== SETUP =================
void setup()
{
  // Setup serial coms
  Serial.begin(115200);
  delay(100);

  // Print setup started
  Dbg.printMsg(Dbg.MT::HEAD1, "RUNNING SETUP");

  // Initialize servos
  FdSrv.initServo();
  delay(1000);

  // Print setup complete
  Dbg.printMsg(Dbg.MT::HEAD1, "SETUP COMPLETE");
}

//=============== LOOP ==================
void loop()
{
  return;

  // Test the servos
  FdSrv.runFeeder(2000);

  // // Lower the feeder
  // FdSrv.lowerFeeder();
  // delay(1000);

  // // Run the pump
  // FdSrv.startPump();
  // delay(1000);
  // FdSrv.stopPump();

  // // Raise the feeder
  // FdSrv.raiseFeeder();

  delay(5000);
}
