// ######################################

//====== FeederGantryOperation.ino ======

// ######################################

/**
 * @file Main Arduino INO file for running the maze.
 */

// BUILT IN
#include "Arduino.h"
// #undef max
// #undef min
// #undef bit

// // CUSTOM
// #include <EsmacatCom.h>
// #include <MazeDebug.h>

bool awaitingOK = false;   // this is set true when we are waiting for the ok signal from the grbl board (see the sendCodeLine() void)


// void grbl_command(String command)
// {
// 	Serial2.print(F(command.c_str());
// 	// Serial2.write("\r\n");

// 	delay(1000);

// 	while(Serial2.available())
// 	{
// 		Serial.write(Serial2.read());
// 	}
// }

void checkForOk() {
  // read the receive buffer (if anything to read)
  char c,lastc;
  c=64;
  lastc=64;
  Serial.println("Checking for RX buffer");
   while (Serial2.available()) {
    c = Serial2.read();  
	Serial.print(c);
    if (lastc=='o' && c=='k') {awaitingOK=false; Serial.println("< OK");}
    lastc=c;
    delay(3);          
    }    
}

void clearRXBuffer(){
  /*
  Just a small void to clear the RX buffer.
  */
  char v;
    while (Serial2.available()) {
      v=Serial2.read();
	  Serial.print("Checking for RX buffer: ");
	  Serial.print(v);
      delay(3);
    }
  }
  

void sendCodeLine(String lineOfCode, bool waitForOk ){
  /*
    This void sends a line of code to the grbl shield, the grbl shield will respond with 'ok'
    but the response may take a while (depends on the command).
    So we immediately check for a response, if we get it, great!
    if not, we set the awaitingOK variable to true, this tells the sendfile() to stop sending code
    We continue to monitor the rx buffer for the 'ok' signal in the getStatus() procedure.
  */
  Serial.print("Send ");
  if ( waitForOk ) Serial.print("and wait, ");
  Serial.println(lineOfCode);
  
  Serial2.println(lineOfCode);
  awaitingOK = true;  
  delay(10);
  Serial.println("SendCodeLine calls for CheckForOk");
  checkForOk();  
  
  while (waitForOk && awaitingOK) {
    delay(50);
    
    checkForOk();      
    }
}



//=============== SETUP =================
void setup()
{
	// Setup serial coms
	Serial.begin(115200);
	Serial2.begin(115200);
	delay(1000);
	// Serial.flush();
	// Serial2.flush();

	clearRXBuffer();
	Serial.print('\n');
	Serial.println("Starting up...");

	Serial2.println("\r\n\r\n");
	delay(1000);
	while(Serial2.available())
	{
		Serial.write(Serial2.read());
	}

	// Set Units(does not seem to work on ender 5)
	// Serial2.print("G20\r\n") // inches
	// Serial2.print("G21"); // millimeters

	// // Absolute Mode
	// Serial2.print("G90\r\n");

	// // Relative Mode
	// // Serial2.print("G91\r\n");

	// // Feed Rate
	// Serial2.print("F25000\r\n");

	// delay(1000);
	// Serial.println("Homing gantry...");
	// Serial2.print("$H\r\n");
}

//=============== LOOP ==================
void loop()
{

	
}