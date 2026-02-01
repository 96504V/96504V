#include "main.h"
#include "lemlib/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER); // Intialize the controller
pros::adi::DigitalOut loader('A'); // Intializes loader pneumatic
pros::adi::DigitalOut wing('B');
pros::MotorGroup leftMotors({-2, -14, -13}, pros::MotorGearset::blue);    //  Creates a motor group with reverse ports 2, 14, 13 and a blue gearset
pros::MotorGroup rightMotors({5, 11, 12}, pros::MotorGearset::blue); 
pros::MotorGroup intake({4, -3});
pros::Motor score({6}); // Creates a motor on port 6
pros::Imu imu(20); //  initializes an Inertial sensor on port 20
pros::Rotation horizontalEncoder(19); //  Initializes a rotation sensor on port 19
pros::Rotation verticalEncoder(18); 
lemlib::TrackingWheel horizontal(&horizontalEncoder, 2, -5.75); // Initializes a tracking wheel object that uses the horizontal encoder from line 9, has a wheel diameter 2in, and an offset of 5.75 inches to the left
lemlib::TrackingWheel vertical(&verticalEncoder, 2, -2.5);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10, 2.75, 450, 2); // Initializes the drivetrain with the left and right motor groups, a track width of 10 in, a 2.75 in wheel diameter, 450 rpm, increases speed
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20); // creates controller settings
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu); // combines all the sensors into one
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019); // creates a drive curve
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve); // creates the chassis

bool loading = false; // tracks whether the match loader is down or up
bool clearing = false; // tracks whether the wing mech is down or up

/**
 * A callback function for LLEMU's center button.
 */

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "Oh my god did you call me baby");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	chassis.calibrate(true);
	pros::lcd::initialize();
	pros::lcd::set_text(1, "thats not okay"); // Used to ensure upload after past upload errors

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	clearing = !clearing; // Record wing mech as raised when driver control starts
	wing.set_value(clearing); // Raise wing mech
	while (true) {			
		pros::lcd::set_text(2, "Jake Phillips x Keigo Koyama drag racing"); // Changed every time to ensure successful download after issues
		chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(127); // Spin intake in
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(-127); // Spin intake out
		} else {
			intake.move(0); // Stop intake
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			score.move(127); // Spin scoring mech
			intake.move(127); // Spin intake
		} else {
			score.move(0); // stop scoring mech
            intake.move(0); // Stop intake mech
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			loading = !loading; // Flip recorded state of fmatch loader mech
			loader.set_value(loading); // Flip physical state of match loader mech
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			clearing = !clearing; // Flip recorded state of wing mech
			wing.set_value(clearing); // Flip physical state of wing mech
		}
		pros::delay(20); // Run for 20 ms then update 

	}
}