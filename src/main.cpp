#include "main.h"
#include "auto.h"
// Threads
pros::Task LadyBrownTask(LadyBrownPID);
pros::Task Positioning(Debug);

lemlib::Pose Debugposition(0,0, 0);
// lemlib::Pose Debugposition(-3, 40, -103);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate(); // calibrate sensors
	LadyBrownRotation.reset_position();
	// LadyBrownTask.resume();
	chassis.setPose(Debugposition);
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