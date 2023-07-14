// ######################################

//====== QC_Testing.ino ======

// ######################################
//  This is the primary arduino INO file for running the QC Test Jig

//============= INCLUDE ================

// GENERAL
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal.h> // include the LiquidCrystal library

// CUSTOM
#include <Safe_Vector.h>
#include <Maze_Debug.h>
#include <Cypress_Com.h>
#include <Wall_Operation.h>

//============ VARIABLES ===============

// Local
uint8_t resp = 0;      // capture I2C comm flags from Wire::method calls [0:success, 1-4:errors]
uint8_t r_bit_out = 1; // I/O value
int pwmDuty = 0;       // track PWM duty for all walls [0-255]
uint8_t mtrDir = 0;    // track motor direction [0:backward/down, 1:forward/up]
int UswitchVal[8];     // track up switch states
int DswitchVal[8];     // track down switch states

// Initialize struct and class instances
Maze_Debug DB;
Cypress_Com C_COM;
Wall_Operation W_OPR(1, 0);
const int rs = 52, en = 50, d4 = 46, d5 = 44, d6 = 42, d7 = 48;
// const int rs = 53, en = 51, d4 = 47, d5 = 45, d6 = 43, d7 = 49;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// LiquidCrystal lcd(52, 50, 46, 44, 42, 48); // initialize the LCD display with the appropriate pins

// Global
extern bool DB_VERBOSE = 1;                       //<set to control debugging behavior [0:silent, 1:verbose]
int potPin = A15;                                 // potentiometer pin
int LED_DOWN = 37;                                // LED for down switch pin
int LED_UP = 39;                                  // LED for up switch pin
int MTR_dirPin = 41;                              // Switch for motor direction pin
uint8_t AddPin[7] = {23, 25, 27, 29, 31, 33, 35}; // Array of DIP switch pin
uint8_t AddBin = 0;                               // Address binary value
uint8_t add_expect = 0x00;                        // address from DIP switches
uint8_t add_measure = 0x00;                       // address from I2C scanner

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
  uint8_t errorI2C, address;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    errorI2C = Wire.endTransmission();

    lcd.setCursor(0, 0);
    lcd.print("Measured: ");

    if (errorI2C == 0)
    {
      nDevices++;
      add_measure = uint8_t(address);
    }
  }

  // Print I2C scan results
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
    DB.printMsgTime("I2C error code: %d", errorI2C);
    DB.printMsgTime("Measured address: %s", DB.hexStr(add_measure));
  }

  // Manually set address for Wall_Opperation methods
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

  // Read DIP switch address
  for (int i = 0; i < 7; i++)
  {
    uint8_t AddBit = digitalRead(AddPin[i]);
    bitWrite(AddBin, i, AddBit);
  }
  add_expect = uint8_t(AddBin);

  // Print expected I2C address on LCD
  lcd.setCursor(0, 1); // set the cursor to the first column of the first row
  lcd.print("Expected: 0x");
  lcd.print(add_expect, HEX); // print expected address (from DIP switch) to the LCD display
  DB.printMsgTime("Expected address: %s", DB.hexStr(add_expect));

  // Print done
  DB.printMsgTime("SETUP DONE");
}

//=============== LOOP ==================
void loop()
{

  // READ DOWN SWITCH FROM CYPRESS, NEED TO READ FROM ALL 8 WALLS AND IF ONE OF THEM IS HIGH -> TURN ON LED
  bool do_d_switch_update = false;
  bool is_d_switch_closed = false;
  for (int i = 0; i < 8; i++)
  {
    C_COM.ioReadPin(add_measure, W_OPR.wms.ioDown[0][i], W_OPR.wms.ioDown[1][i], r_bit_out);
    if (DswitchVal[i] != r_bit_out)
      do_d_switch_update = true;
    if (r_bit_out == 1)
      is_d_switch_closed = true;
    DswitchVal[i] = r_bit_out; // read pin value and store it in array
  }
  if (do_d_switch_update)
  {
    DB.printMsgTime("Down switch val: [%d,%d,%d,%d,%d,%d,%d,%d]", DswitchVal[0], DswitchVal[1], DswitchVal[2], DswitchVal[3], DswitchVal[4], DswitchVal[5], DswitchVal[6], DswitchVal[7]);
    if (is_d_switch_closed)
      digitalWrite(LED_DOWN, HIGH);
    else
      digitalWrite(LED_DOWN, LOW);
  }

  // READ UP SWITCH FROM CYPRESS, NEED TO READ FROM ALL 8 WALLS AND IF ONE OF THEM IS HIGH -> TURN ON LED
  bool do_u_switch_update = false;
  bool is_u_switch_closed = false;
  for (int i = 0; i < 8; i++)
  {
    C_COM.ioReadPin(add_measure, W_OPR.wms.ioUp[0][i], W_OPR.wms.ioUp[1][i], r_bit_out);
    if (UswitchVal[i] != r_bit_out)
      do_u_switch_update = true;
    if (r_bit_out == 1)
      is_u_switch_closed = true;
    UswitchVal[i] = r_bit_out; // read pin value and store it in array
  }
  if (do_u_switch_update)
  {
    DB.printMsgTime("Up switch val: [%d,%d,%d,%d,%d,%d,%d,%d]", UswitchVal[0], UswitchVal[1], UswitchVal[2], UswitchVal[3], UswitchVal[4], UswitchVal[5], UswitchVal[6], UswitchVal[7]);
    if (is_u_switch_closed)
      digitalWrite(LED_UP, HIGH);
    else
      digitalWrite(LED_UP, LOW);
  }

  // Read potentiometer and set duty cycle
  int sensor_val = analogRead(potPin);
  int pwm_duty = (int)((float)sensor_val * (255.00 / 1023.00)); // Potentiometer
  if (abs(pwmDuty - pwm_duty) > 5)
  {
    pwmDuty = pwm_duty;
    // Setup source
    for (size_t src_i = 0; src_i < 8; src_i++)
      resp = C_COM.setupSourcePWM(add_measure, W_OPR.wms.pwmSrc[src_i], pwmDuty);
    DB.printMsgTime("Potentiometer duty = %d", pwmDuty);
  }

  // Read motor direction [1= backward, 0=forward]
  uint8_t mtr_dir = digitalRead(MTR_dirPin);
  if (mtr_dir != mtrDir)
  {
    mtrDir = mtr_dir;
    DB.printMsgTime("Motor direction = %d(%s)", mtrDir, mtrDir == 0 ? "Backward/Down" : "Forward/Up");

    if (mtrDir == 0)
    {
      // WRITE TO CYPRESS to go BACKWARD/DOWN
      for (int wall_n = 0; wall_n < 8; wall_n++)
      {
        C_COM.ioWritePin(add_measure, W_OPR.wms.pwmUp[0][wall_n], W_OPR.wms.pwmUp[1][wall_n], 0);
        C_COM.ioWritePin(add_measure, W_OPR.wms.pwmDown[0][wall_n], W_OPR.wms.pwmDown[1][wall_n], 1);
      }
    }
    else
    {
      // WRITE TO CYPRESS to go FORWARD/UP
      for (int wall_n = 0; wall_n < 8; wall_n++)
      {
        C_COM.ioWritePin(add_measure, W_OPR.wms.pwmUp[0][wall_n], W_OPR.wms.pwmUp[1][wall_n], 1);
        C_COM.ioWritePin(add_measure, W_OPR.wms.pwmDown[0][wall_n], W_OPR.wms.pwmDown[1][wall_n], 0);
      }
    }
  }

  delay(100); // wait for a short time before reading again
}
