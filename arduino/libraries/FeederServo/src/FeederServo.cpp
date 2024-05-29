// ######################################

//========== FeederServo.cpp ============

// ######################################

/// <file>
/// Used for the FeederServo class
/// <file>

//============= INCLUDE ================
#include "FeederServo.h"

//===========CLASS: FeederServo============

/// @brief Constructor
FeederServo::FeederServo() {}

/// @brief Initialize the servo objects.
void FeederServo::initServo()
{
	// Setup the port servo
	portServo.attach(portServoPin); // Attach the servo object to the pwm pin
	portServo.write(portUpAngle);  // Set the servo to the up position

	// Setup the pump servo
	pumpServo.attach(pumpServoPin); // Attach the servo object to the pwm pin
}

/// @brief Lower the feeder.
void FeederServo::lowerFeeder()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Lowering the feeder");
	portServo.write(portDownAngle);
}

/// @brief Raise the feeder.
void FeederServo::raiseFeeder()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Raising the feeder");
	portServo.write(portUpAngle);
}

/// @brief Start the pump.
void FeederServo::startPump()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Running the pump");
	pumpServo.write(pumpRunSpeed);
}

/// @brief Stop the pump.
void FeederServo::stopPump()
{
	_Dbg.printMsg(_Dbg.MT::INFO, "Stopping the pump");
	pumpServo.write(pumpStopSpeed);
}

/// @brief Run the feeder.
void FeederServo::runFeeder(int dt_run)
{
	// Lower the feeder
	lowerFeeder();
	delay(1000);

	// Run the pump
	startPump();
	delay(dt_run);
	stopPump();

	// Raise the feeder
	raiseFeeder();
}

private:
    serial::Serial ser;
    double value_X;
    double value_Y;

    void initialise(const std::string& cmd) {
        std::string full_cmd = cmd + "\n";
        Serial1.write(full_cmd);
        delay(1000);
        while (true) {
            std::string feedback = ser.readline();
            if (feedback == "ok\n") {
                // std::cout << feedback << std::endl;
                break;
            }
        }
    }

public:
    GRBLClient(const std::string& port, unsigned long baud) : ser(port, baud, serial::Timeout::simpleTimeout(1000)), value_X(0.0), value_Y(0.0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Set Units (does not seem to work on ender 5)
        initialise("G21"); // millimeters

        // Absolute Mode
        initialise("G90");

        // Relative Mode
        // initialise("G91");

        // Feed Rate
        initialise("F25000");
    }

    void realtime_command(const std::string& cmd) {
        Serial1.write(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool raw_command(const std::string& cmd) {
        try {
            std::string full_cmd = cmd + "\n";
            Serial1.write(full_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::string feedback = ser.readline();
            return feedback == "ok\n";
        } catch (const std::exception& e) {
            std::cerr << "Gcode commands must be a string" << std::endl;
            return false;
        }
    }

    void command(const std::string& cmd) {
        try {
            std::string upper_cmd = cmd;
            std::transform(upper_cmd.begin(), upper_cmd.end(), upper_cmd.begin(), ::toupper);
            std::istringstream iss(upper_cmd);
            std::string subcmd;

            while (iss >> subcmd) {
                if (subcmd[0] == 'X') {
                    value_X += std::stod(subcmd.substr(1));
                } else if (subcmd[0] == 'Y') {
                    value_Y += std::stod(subcmd.substr(1));
                }
            }

            std::string full_cmd = cmd + "\n";
            Serial1.write(full_cmd);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            while (true) {
                std::string feedback = ser.readline();
                if (feedback == "ok\n") {
                    // std::cout << feedback << std::endl;
                    break;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Gcode commands must be a string" << std::endl;
        }
    }
