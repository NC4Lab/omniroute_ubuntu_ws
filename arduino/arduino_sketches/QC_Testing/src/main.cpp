#include <Arduino.h>

// ######################################

//====== QC_Testing.ino ======

// ######################################
//  This is the primary arduino INO file for running the QC Test Jig

//============= INCLUDE ================

// GENERAL
#include <Wire.h>
#include <LiquidCrystal.h> // include the LiquidCrystal library

// CUSTOM
#include <Safe_Vector.h>
#include <Maze_Debug.h>
#include <Cypress_Com.h>
#include <Cypress_Com_Base.h>
#include <Wall_Operation.h>

//============ VARIABLES ===============

// Local
uint8_t resp = 0;      // capture I2C comm flags from Wire::method calls [0:success, 1-4:errors]
uint8_t pwmDuty = 0;   // PWM duty for all walls [0-255]
uint8_t r_bit_out = 1; // I/O value

// Initialize struct and class instances
Maze_Debug DB;
Cypress_Com C_COM;
Wall_Operation W_OPR(1);
LiquidCrystal lcd(52, 50, 46, 44, 42, 48); // initialize the LCD display with the appropriate pins

// Global
extern bool DB_VERBOSE = 1;                    //<set to control debugging behavior [0:silent, 1:verbose]
int potPin = A15;                              // potentiometer pin
int LED_DWN = 49;                              // LED for down switch pin
int LED_UP = 51;                               // LED for up switch pin
int MTR_dirPin = 53;                           // Switch for motor direction pin
byte AddPin[7] = {23, 25, 27, 29, 31, 33, 35}; // Array of DIP switch pin
byte AddBin = 0;                               // Address binary value
byte add_expect = 0x00;                        // address from DIP switches
byte add_measure = 0x00;                       // address from I2C scanner

//=============== SETUP =================
void setup()
{

  // Setup serial coms
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.print('\n');
  DB.printMsgTime("SETUP START");

  Wire.begin();     // join I2C bus
  lcd.begin(16, 2); // initialize the LCD display with 16 columns and 2 rows
  lcd.clear();      // clear the LCD display

  // Print which microcontroller is active
#ifdef ARDUINO_AVR_UNO
  DB.printMsgTime("Uploading to Arduino Uno");
#endif
#ifdef __AVR_ATmega2560__
  DB.printMsgTime("Uploading to Arduino Mega");
#endif
#ifdef ARDUINO_SAM_DUE
  DB.printMsgTime("Uploading to Arduino Due");
#endif

  // Read I2C scanner
  int nDevices = 0;
  byte errorI2C, address;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    errorI2C = Wire.endTransmission();

    lcd.setCursor(0, 0);
    lcd.print("Measured: ");

    if (errorI2C == 0)
    {
      nDevices++;
      add_measure = byte(address);
    }
  }

  if (nDevices == 0)
  {
    lcd.print("Not found"); // check I2C connection
  }
  else if (errorI2C == 4)
  {
    lcd.print("Error");
  }
  else
  {
    lcd.print("0x");
    lcd.print(add_measure, HEX); // print expected address (from I2C scanner) to the LCD display
    Serial.print("I2C error code: ");
    Serial.println(errorI2C);
    Serial.print("Measured address: 0x");
    Serial.println(add_measure, HEX); // print expected address (from I2C scanner) to the LCD display
  }

  // Manually set address
  W_OPR.C[0].addr = add_measure;

  // Setup cypress chips
  resp = C_COM.setupCypress(add_measure);
  if (resp != 0)
    DB.printMsgTime("!!Failed Cypress setup: address=%s!!", DB.hexStr(add_measure));
  else
  { // print success for last itteration
    DB.printMsgTime("Finished Cypress setup: address=%s", DB.hexStr(add_measure));
  }

  // Setup IO pins 
	resp = W_OPR.setupWallIO();

	// Setup PWM pins 
	resp = W_OPR.setupWallPWM(pwmDuty);

  // Print done
  DB.printMsgTime("SETUP DONE");
}

//=============== LOOP ==================
void loop()
{

  // Read DIP switch address
  for (int i = 0; i < 7; i++)
  {
    byte AddBit = digitalRead(AddPin[i]);
    bitWrite(AddBin, i, AddBit);
  }
  add_expect = byte(AddBin);

  // Print expected I2C address on LCD
  lcd.setCursor(0, 1); // set the cursor to the first column of the first row
  lcd.print("Expected: 0x");
  lcd.print(add_expect, HEX); // print expected address (from DIP switch) to the LCD display

  // Read UP switch and turn on  LED
  int UswitchVal[8];
  Serial.print("UP switch val: [");
  // READ FROM CYPRESS UP Switch NEED TO READ FROM ALL 8 WALLS AND IF ONE OF THEM IS HIGH -> TURN ON LED
  for (int i = 0; i < 8; i++)
  {
    C_COM.ioReadPin(add_measure, W_OPR.wms.ioUp[0][i], W_OPR.wms.ioUp[1][i], r_bit_out);
    UswitchVal[i] = r_bit_out; // read pin value and store it in array
    Serial.print(UswitchVal[i]);
    Serial.print(", ");
  }
  Serial.println("]");
  if (UswitchVal[0] == 1 | UswitchVal[1] == 1 | UswitchVal[2] == 1 | UswitchVal[3] == 1 | UswitchVal[4] == 1 | UswitchVal[5] == 1 | UswitchVal[6] == 1 | UswitchVal[7] == 1)
  {
    digitalWrite(LED_UP, HIGH);
  }
  else
  {
    digitalWrite(LED_UP, LOW);
  }

  // Read DOWN switch and turn on  LED
  int DswitchVal[8];
  Serial.print("DOWN switch val: [");
  // READ FROM CYPRESS DOWN Switch NEED TO READ FROM ALL 8 WALLS AND IF ONE OF THEM IS HIGH -> TURN ON LED
  for (int i = 0; i < 8; i++)
  {
    C_COM.ioReadPin(add_measure, W_OPR.wms.ioDown[0][i], W_OPR.wms.ioDown[1][i], r_bit_out);
    DswitchVal[i] = r_bit_out; // read pin value and store it in array
    Serial.print(DswitchVal[i]);
    Serial.print(", ");
  }
  Serial.println("]");
  if (DswitchVal[0] == 1 | DswitchVal[1] == 1 | DswitchVal[2] == 1 | DswitchVal[3] == 1 | DswitchVal[4] == 1 | DswitchVal[5] == 1 | DswitchVal[6] == 1 | DswitchVal[7] == 1)
  {
    digitalWrite(LED_DWN, HIGH);
  }
  else
  {
    digitalWrite(LED_DWN, LOW);
  }

  // Read potentiometer and set duty cycle
  pwmDuty = (byte)analogRead(potPin); // Potentiometer value is between 0-255
  // Setup source
  for (size_t src_i = 0; src_i < 8; src_i++)
  {
    resp = C_COM.setupSourcePWM(add_measure, W_OPR.wms.pwmSrc[src_i], pwmDuty);
    if (resp != 0)
    {
      DB.printMsgTime("!!Failed pwm source Drive pin setup!!");
    }
  }
  if (resp == 0)
  {
    DB.printMsgTime("Finished pwm source pin setup");
  }
  Serial.print("Potentiometer: ");
  Serial.println(pwmDuty); // print the potentiometer value to the serial monitor

  // Read motor direction (1= backward, 0=forward)
  int MTRdir = digitalRead(MTR_dirPin);
  Serial.print("Motor direction: ");
  Serial.print(MTRdir); // print the motor direction to the serial monitor

  if (MTRdir == 0)
  {
    Serial.println(" Backward");
    // WRITE TO CYPRESS to go BACKWARD
    for (int wall_n = 0; wall_n < 8; wall_n++)
    {
      C_COM.ioWritePin(add_measure, W_OPR.wms.pwmUp[0][wall_n], W_OPR.wms.pwmUp[1][wall_n], 1);
      C_COM.ioWritePin(add_measure, W_OPR.wms.pwmDown[0][wall_n], W_OPR.wms.pwmDown[1][wall_n], 0);
    }
  }
  else
  {
    Serial.println(" Forward");
    // WRITE TO CYPRESS to go FORWARD
    for (int wall_n = 0; wall_n < 8; wall_n++)
    {
      C_COM.ioWritePin(add_measure, W_OPR.wms.pwmUp[0][wall_n], W_OPR.wms.pwmUp[1][wall_n], 0);
      C_COM.ioWritePin(add_measure, W_OPR.wms.pwmDown[0][wall_n], W_OPR.wms.pwmDown[1][wall_n], 1);
    }
  }

  delay(500); // wait for a short time before reading again
}
