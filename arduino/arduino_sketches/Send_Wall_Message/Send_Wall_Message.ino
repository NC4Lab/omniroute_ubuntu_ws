#include <SoftwareSerial.h>
// Sends a test message to the maze wall control script
// Message format [header, number of messages, chamber number, wall byte, footer]

// Connect switch to pin 13 and ground
int switchPin = 13;

// Connect RX to 2 ant TX to 3
SoftwareSerial serialOut(2, 3); // RX, TX


// Include the required Wire library for I2C<br>#include <Wire.h>
void setup() {
	// Setup serial coms
	Serial.begin(115200);
	serialOut.begin(115200);
	// Join i2c bus (address optional for master)
	delay(100);
	Serial.print('\n');
	Serial.println("SETUP START");

	// Set switch pin
	pinMode(switchPin, INPUT_PULLUP);
}

void loop() {
	static bool is_on = false;
	static int dt = 500;
	static uint32_t t_check = millis();

	// Check switch
	if (millis() > t_check) {
		if (digitalRead(switchPin) == LOW && !is_on) {
			Serial.println("ON");
			t_check = millis() + dt;
			is_on = true;
			sendMessage();
		}
		else if (digitalRead(switchPin) == HIGH && is_on) {
			Serial.println("OFF");
			t_check = millis() + dt;
			is_on = false;
		}
	}
}

void sendMessage() {
	static uint8_t run_cnt = 0;
	uint8_t wall_byte = 0;
	
	switch (run_cnt) {
	case 0:
		wall_byte = B10010100;  // 2,4,7
		Serial.println("Send wall 2,4");
		break;
	case 1:
		wall_byte = B00100100; // 2,5
		Serial.println("Send wall 2,5");
		break;
	case 2:
		wall_byte = B00000000; // none
		Serial.println("Send wall none");
		break;
	default:
		break;
	}

	// Send message
	uint8_t msg[5] = {
254,
1,
0,
wall_byte,
255
	};
	serialOut.write(msg, 5);

	// Itterate
	run_cnt = run_cnt<2? run_cnt+1: 0;

}