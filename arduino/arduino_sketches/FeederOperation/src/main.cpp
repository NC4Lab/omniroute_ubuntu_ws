// ######################################

//====== FeederOperation.ino ======

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

// Global variables
bool DB_VERBOSE = 1;  // set to control debugging behavior [0:silent, 1:verbose]
bool DO_ECAT_SPI = 1; // set to control block SPI [0:dont start, 1:start]

// Initialize class instances for local libraries
MazeDebug Dbg;
FeederServo FdSrv;
EsmacatCom EsmaCom;

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

  // // TEMP
  // while (true)
  // {
  //   EsmaCom.writeEcatAck(EsmaCom.ErrorType::ERR_NONE, EsmaCom.rcvEM.ArgU.ui8, EsmaCom.rcvEM.argLen);
  //   delay(1000);
  //   EsmaCom.writeEcatAck(EsmaCom.ErrorType::ECAT_ID_DISORDERED, EsmaCom.rcvEM.ArgU.ui8, EsmaCom.rcvEM.argLen);
  //   delay(1000);
  //   EsmaCom.writeEcatAck(EsmaCom.ErrorType::ECAT_NO_MSG_TYPE_MATCH, EsmaCom.rcvEM.ArgU.ui8, EsmaCom.rcvEM.argLen);
  //   delay(1000);
  // }

  // Check ethercat coms
  if (!EsmaCom.readEcatMessage())
    return;

  // Start pump
	if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::START_PUMP)
	{
		FdSrv.startPump();
	}

  // Stop pump
  if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::STOP_PUMP)
  {
    FdSrv.stopPump();
  }

  // Lower feeder
  if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::LOWER_FEEDER)
  {
    FdSrv.lowerFeeder();
  }

  // Raise feeder
  if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::RAISE_FEEDER)
  {
    FdSrv.raiseFeeder();
  }

  // Send back recieved message arguments
  EsmaCom.writeEcatAck(EsmaCom.ErrorType::ERR_NONE, EsmaCom.rcvEM.ArgU.ui8, EsmaCom.rcvEM.argLen);

  // Test the servos
  // FdSrv.runFeeder(2000);
}
