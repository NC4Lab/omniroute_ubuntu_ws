
// ##########################

// ====== MazeSync.ino ======

// ##########################

/**
 @file Arduino INO file for handeling time stamp syncing oppertions.
*/

// BUILT IN
#include "Arduino.h"

// LOCAL
#include <EsmacatCom.h>

// Define pins
const int easePin = 9; // EASE SPI pin for ethercat communication
const int optiSyncPin = 7; // Optitrack sync pin
const int spikeGadgSyncPin = 6; // Spike Gadgets sync pin

Esmacatshield ease(easePin);

int easeRegisters[8];

void setup()
{
  ease.start_spi();

  pinMode(optiSyncPin, OUTPUT);
  digitalWrite(optiSyncPin, LOW);
}

void loop()
{
  // Get registers from EASE
  ease.get_ecat_registers(easeRegisters);

  digitalWrite(optiSyncPin, easeRegisters[0]);

  delay(10);
}