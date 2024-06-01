// #################################

// ====== FeederOperation.ino ======

// #################################

/**
 @file Arduino INO file for running feeeder related oppertions.
*/

// BUILT IN
#include "Arduino.h"
#include <Servo.h>

// LOCAL
#include <MazeDebug.h>
#include <FeederServo.h>
#include <EsmacatCom.h>

//============ VARIABLES ===============

// Global variables
bool DB_VERBOSE = 1;  // set to control debugging behavior [0:silent, 1:verbose]
bool DO_ECAT_SPI = 1; // set to control block SPI [0:dont start, 1:start]

// Initialize class instances for local libraries
MazeDebug Dbg;          // instance of MazeDebug class for debugging messages
FeederServo FdSrv;      // instance of FeederServo class

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
  FdSrv.servoInit();
  delay(1000);

  // Print setup complete
  Dbg.printMsg(Dbg.MT::HEAD1, "SETUP COMPLETE");
}

//=============== LOOP ==================
void loop()
{
  // Check ethercat coms
  if (!FdSrv.EsmaCom.readEcatMessage())
    return;

  // Process and execute ethercat arguments
  FdSrv.procEcatMessage();
}