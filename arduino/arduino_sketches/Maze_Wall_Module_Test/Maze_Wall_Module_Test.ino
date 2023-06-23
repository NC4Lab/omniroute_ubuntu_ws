
// Settings
bool DO_AUTO = false; // Set true to automatically move wall up/down repeatedly

// Specify output pins
const int pin_IA = 5; // run forward HIGH; backward LOW TEMP(3)
const int pin_IB = 3; // run forward LOW; backward HIGH TEMP(5)

// Specify input pins
const int pin_upSwitch = 9; // Upper limit switch
const int pin_downSwitch = 10; // Lower limit switch
const int pin_wallDirSwitch = 11; // Move wall up down [closed, open]
const int pin_motPowSwitch = 13; // Power motor
const int pin_PotPWM = A0; // Analog pin for pot reading 

// Declare vars
int dirSwitchState = 1; // [0,1] [down, up]
int moveState = 0; // [0,1] [down, up]
int runState = 0; // [0,1] [off, on]
int pwmFreq = 150; // [0.256]
bool upSwitchState = 0; // [0,1] [inactive, active]
bool downSwitchState = 0; // [0,1] [inactive, active]

// FUNTION setup()
void setup() {

	// Serial setup
	Serial.begin(9600);
	delay(100);
	Serial.println();

	// Set output pins
	pinMode(pin_IA, OUTPUT);
	pinMode(pin_IB, OUTPUT);

	// Set input pins
	pinMode(pin_upSwitch, INPUT);
	pinMode(pin_downSwitch, INPUT);
	pinMode(pin_motPowSwitch, INPUT);
	pinMode(pin_PotPWM, INPUT);

	// Get initial switch state
	upSwitchState = digitalRead(pin_upSwitch);
	downSwitchState = digitalRead(pin_downSwitch);

	// Setup finished
	Serial.println("SETUP DONE");
}

// FUNTION loop()
void loop() {

	// Check PWM input
	CheckInputPWM();

	// Check move switch
	CheckMoveSwitch();

	// Check upper switch
	CheckLimitSwitch();

}

// FUNCTION GET NEW PWM
void CheckInputPWM() {

	// Local vars
	static int pwm_new = 0;
	int sensor_val = 0;

	// Get sensor value and convirt range from [0, 1023] to [0, 255]
	sensor_val = analogRead(pin_PotPWM);
	pwm_new = (int)((float)sensor_val * (255.00 / 1023.00));

	// Bail if difference in value not > 5
	if (abs(pwmFreq - pwm_new) < 5) {
		return;
	}

	// Update pwm
	MoveWall(moveState);

	// Store value
	pwmFreq = pwm_new;

	// Print new pwm
	Serial.print(millis());
	Serial.print(" SET PWM TO ");
	Serial.println(pwmFreq);

}

// FUNCTION CHECK MOVE SWITCH
void CheckMoveSwitch() {

	// Local vars
	const int dt_debounce = 100;
	static uint32_t t_debounce = 0;

	// Exit if < dt has not passed
	if (millis() < t_debounce) return;

	// Get switch state if not doing automatic up down
	if (!DO_AUTO) dirSwitchState = digitalRead(pin_wallDirSwitch);

	// Bail if no change
	if (dirSwitchState == moveState) return;

	// Update move state
	moveState = dirSwitchState;

	// Move wall down or up
	MoveWall(moveState);

	// Print state
	switch (moveState)
	{
	case 0:
		PrintMessage("MOVING WALL: DOWN");
		break;
	case 1:
		PrintMessage("MOVING WALL: UP");
		break;
	default:
		break;
	}

	// Update debounce
	t_debounce = millis() + dt_debounce;

}

// FUNCTION CHECK LIMIT SWITCHES
void CheckLimitSwitch() {

	// Local vars
	const int dt_debounce = 100;
	static uint32_t t_debounce = 0;

	// Exit if < dt has not passed
	if (millis() < t_debounce) return;

	// Get both switch states
	bool up_switch_state = digitalRead(pin_upSwitch);
	bool down_switch_state = digitalRead(pin_downSwitch);

	// Print change
	upSwitchState != up_switch_state ? up_switch_state ? PrintMessage("	up switch on") : PrintMessage("	up switch off") : void();
	downSwitchState != down_switch_state ? down_switch_state ? PrintMessage("	down switch on") : PrintMessage("	down switch off") : void();

	// Update flags
	upSwitchState = up_switch_state;
	downSwitchState = down_switch_state;

	// Bail if not running
	if (runState == 0) return;

	// Get switch state
	bool swtch_state = moveState == 0 ? down_switch_state : up_switch_state;

	// Bail if nothing triggered
	if (swtch_state == 0) return;

	// Print message
	PrintMessage(moveState == 0 ? "	Down Switch Triggered" : "	Up Switch Triggered");

	// Stop wall
	StopWall();

	// Set to opposite direction
	if (DO_AUTO) {
		static int c_run = 0; c_run++;
		dirSwitchState = dirSwitchState == 0 ? 1 : 0;
		Serial.println(c_run);
	}

	// Update debounce
	t_debounce = millis() + dt_debounce;
}

// FUNTION MOVE WALL
void MoveWall(int move_state) {

	// Move down/forward
	if (move_state == 0) {
		analogWrite(pin_IA, 0);
		analogWrite(pin_IB, pwmFreq);
		PrintMessage("RUNNING MOTOR DOWN/FORWARD");
	}
	// Move up/backward
	if (move_state == 1) {
		analogWrite(pin_IA, pwmFreq);
		analogWrite(pin_IB, 0);
		PrintMessage("RUNNING MOTOR UP/BACKWARD");
	}

	// Update state
	runState = 1;

}

// FUNTION STOP WALL
void StopWall() {

	// Set pin
	analogWrite(pin_IA, 1);
	analogWrite(pin_IB, 1);
	delay(100);
	analogWrite(pin_IA, 0);
	analogWrite(pin_IB, 0);

	// Update state
	runState = 0;

	// Print
	PrintMessage("MOTOR STOPED");
}

// FUNCTION PRINT MESSAGE
void PrintMessage(char* p_msg) {
	// Dont print if running auto
	if (DO_AUTO) return;

	// Print time
	Serial.print(millis());
	Serial.print(" ");
	Serial.println(p_msg);

}

