/***
   Sync EASE Sketch

   Skylar Fang
   Manu Madhav

   v1.0: 29-Mar-23
*/

#include <Esmacatshield.h>

const int easePin = 9;
const int syncPin = 7;

Esmacatshield ease(easePin);

int easeRegisters[8];

void setup() {
  ease.start_spi();
//    Serial.begin(9600);           // set up Serial library at 9600 bps

  pinMode(syncPin, OUTPUT);
  digitalWrite(syncPin,LOW);
}

void loop() {
  // Get registers from EASE
    ease.get_ecat_registers(easeRegisters);
//    Serial.print(easeRegisters[0]);

  digitalWrite(syncPin, easeRegisters[0]);
  delay(10);
  }
