
// #################################

// =========== main.ino ============

// #################################

/**
 @file Arduino INO file for MazeSync handeling time stamp syncing oppertions.
*/

// BUILT IN
#include "Arduino.h"

// LOCAL
#include <EsmacatCom.h>

//============ VARIABLES ===============

// Global variables
bool DB_VERBOSE = 0;  // set to control debugging behavior [0:silent, 1:verbose]
bool DO_ECAT_SPI = 1; // set to control block SPI [0:dont start, 1:start]

// Initialize class instances for local libraries
MazeDebug Dbg;
EsmacatCom EsmaCom(10);

// Define pins
const int optiSyncPin = 7;      // Optitrack sync pin
const int spikeGadgSyncPin = 6; // Spike Gadgets sync pin

void setup()
{
  // Setup serial coms
  Serial.begin(115200);
  delay(100);

  // Print setup started
  Dbg.printMsg(Dbg.MT::HEAD1, "RUNNING SETUP");

  // Initialize ethercat coms
  EsmaCom.initEcat(true);

  // Set pin modes
  pinMode(spikeGadgSyncPin, OUTPUT);
  pinMode(optiSyncPin, OUTPUT);

  // Set pin states
  digitalWrite(spikeGadgSyncPin, LOW);
  digitalWrite(optiSyncPin, LOW);

  // Print setup complete
  Dbg.printMsg(Dbg.MT::HEAD1, "SETUP COMPLETE");
}

void loop()
{
  // Check ethercat coms
  if (!EsmaCom.readEcatMessage())
    return;

  // Get first argument
  uint8_t arg0 = EsmaCom.rcvEM.ArgU.ui8[0];

  // Set Optitrack sync pin
  if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::SET_OPTITRACK_SYNC_PIN)
  {
    digitalWrite(optiSyncPin, arg0);
    Dbg.printMsg(Dbg.MT::INFO, "Optitrack sync pin set to: %s", arg0 == 0 ? "LOW" : "HIGH");
  }

  // Set SpikeGadgets sync pin
  if (EsmaCom.rcvEM.msgTp == EsmaCom.MessageType::SET_SPIKEGADGETS_PIN)
  {
    digitalWrite(spikeGadgSyncPin, arg0);
    Dbg.printMsg(Dbg.MT::INFO, "SpikeGadgets sync pin set to: %s", arg0 == 0 ? "LOW" : "HIGH");
  }

  // Send back recieved message arguments
  EsmaCom.writeEcatAck(EsmaCom.ErrorType::ERR_NONE, EsmaCom.rcvEM.ArgU.ui8, EsmaCom.rcvEM.argLen);
}