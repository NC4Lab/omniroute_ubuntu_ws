// BUILT IN
#include "Arduino.h"
#include <Wire.h>

//=============== SETUP =================
void setup()
{
	// Setup serial coms
	Serial.begin(115200);
    Serial1.begin(115200);
}

//=============== LOOP ==================
void loop()
{
    // Read string from serial port
    String inputString = "";
    while (Serial1.available())
    {
        inputString += (char)Serial1.read();
    }

    // Print string to serial port
    Serial.print(inputString);

} // end loop
    
