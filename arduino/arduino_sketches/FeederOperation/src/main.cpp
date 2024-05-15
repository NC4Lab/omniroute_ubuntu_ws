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
bool DB_VERBOSE = 0;  // set to control debugging behavior [0:silent, 1:verbose]
bool DO_ECAT_SPI = 1; // set to control block SPI [0:dont start, 1:start]

// Initialize class instances for local libraries
MazeDebug Dbg; // instance of MazeDebug class for debugging messages
FeederServo FdSrv; // instance of FeederServo class
EsmacatCom EsmaCom(10);  // instance of EsmacatCom class using SPI chip select pin 10

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

  // Initialize ethercat coms
  EsmaCom.initEcat(true);

  // Print setup complete
  Dbg.printMsg(Dbg.MT::HEAD1, "SETUP COMPLETE");
}

//=============== LOOP ==================
void loop()
{

  // Check ethercat coms
  if (!EsmaCom.readEcatMessage())
    return;

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

  // Run full feeder opperation
  if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::REWARD)
  {
    FdSrv.runFeeder(2000);
  }

  // Send back recieved message arguments
  EsmaCom.writeEcatAck(EsmaCom.ErrorType::ERR_NONE, EsmaCom.rcvEM.ArgU.ui8, EsmaCom.rcvEM.argLen);

}
