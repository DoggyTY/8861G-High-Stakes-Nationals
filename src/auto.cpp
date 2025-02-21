#include "main.h"

double Drivekp = 18;
double Driveki = 0;
double Drivekd = 80;
double DriveSmallError = 0.5;

double DriveAngularkp = 5;
double DriveAngularki = 0;
double DriveAngularkd = 30;
double DriveAngularSmallError = 3;

lemlib::Pose Neutral(0,0,0);
lemlib::Pose GoalRush(50.873,-60.544, 90);
lemlib::Pose AWPPose(0, 0, -150);

//Autons
void BottomGoalRush();
void AWPsolo();
void Skills();

// left motor group
pros::MotorGroup left_motor_group({ -13, -14, 15 }, pros::v5::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({ 16, -17, 18 }, pros::v5::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11, // 15 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              300, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now));
);

// horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&LaderalTracking, 2, -0.5);
// vertical tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&ParrellelTracking, 2, 4.5);
lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            &horizontal_tracking_wheel,
                            nullptr,
                            &imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(Drivekp, // proportional gain (kP)
                                              Driveki, // integral gain (kI)
                                              Drivekd, // derivative gain (kD)
                                              0, // anti windup
                                              DriveSmallError, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3*DriveSmallError, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
/*
 3, // anti windup
                                              DriveSmallError, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3*DriveSmallError, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maxim
*/

// angular PID controller
lemlib::ControllerSettings angular_controller(DriveAngularkp, // proportional gain (kP)
                                              DriveAngularki, // integral gain (kI)
                                              DriveAngularkd, // derivative gain (kD)
                                              0, // anti windup
                                              DriveAngularSmallError, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3*DriveAngularSmallError, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
/*
3, // anti windup
                                              DriveAngularSmallError, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3*DriveAngularSmallError, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
*/

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


void autonomous() {
    // chassis.setPose(Neutral);
    // chassis.turnToHeading(40, 100000);
    // pros::delay(100);
    // chassis.moveToPoint(24, 24, 100000);
    // BottomGoalRush();
    // AWPsolo();
    // Skills();

}
void Debug() {
    while (true) {
        lemlib::Pose pose = chassis.getPose();
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %f, Y: %f, Heading: %f", pose.x, pose.y, pose.theta);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "%f", setpoint);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "%f", LadyBrownRotation.get_position());
        pros::delay(500);
        
    }
}

void BottomGoalRush() {
    /*
    chassis.setPose(GoalRush);
    chassis.moveToPoint(9.5, -43.183, 1000);
    Doinker.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(23.419, -38.303, 1000);
    Doinker.set_value(false);
        pros::delay(200);
    chassis.turnToHeading(180, 1000);
        pros::delay(200);

    chassis.moveToPoint(23.488, -23.807, 1000);
    GoalClamp.set_value(true);
    Intake.move(-127);
        pros::delay(200);
    chassis.moveToPoint(23.882, -58.622, 1000);
    GoalClamp.set_value(false);
        pros::delay(200);
    chassis.turnToHeading(135, 500);
    Intake.move(0);
        pros::delay(200);
    chassis.moveToPoint(11.279, -47.594, 1000);
        pros::delay(200);

    chassis.moveToPoint(32.083,-47.676, 1250);
        pros::delay(200);

    chassis.turnToHeading(30, 500);
        pros::delay(200);

    chassis.moveToPoint(43.649, -11.279, 1000);
    Doinker.set_value(true);
        pros::delay(200);

    chassis.turnToHeading(90, 1000);
    Doinker.set_value(false);
    pros::delay(200);
    chassis.turnToHeading(60, 1000);
    pros::delay(200);

    chassis.moveToPoint(61.529, 0, 1000);
    pros::delay(200);
    chassis.turnToHeading(180, 1000);
    pros::delay(200);
    chassis.moveToPoint(65.549/2, -65.789, 1000);
    Doinker.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(65.549, -65.789, 1000);
    pros::delay(200);
    chassis.turnToHeading(270, 500);
    Doinker.set_value(false);
    // Lady Brown
    pros::delay(200);
    chassis.moveToPoint(4.097, -66.316, 1000);
    */

    chassis.setPose(Neutral);
    setpoint = LADYBROWNSCORE;
    chassis.moveToPoint(0, -33, 1000, {.forwards = false}, false);
    pros::delay(200);
    chassis.turnToHeading(CURRENTTEAM*30, 500, {}, false);
    pros::delay(200);
    chassis.moveToPoint(-5, -46, 1000, {.forwards = false, .maxSpeed = 50});
    pros::delay(600);
    GoalClamp.set_value(true);
    pros::delay(100);
    Intake.move(-127);
    pros::delay(1000);
    chassis.turnToHeading(CURRENTTEAM*-11, 500, {}, false);
    pros::delay(250);
    Intake.move(0);
    pros::delay(1000);
    Intake.move(-127);
    chassis.moveToPoint(CURRENTTEAM*-8, -30, 1000, {}, false);
    Intake.move(0);
    pros::delay(1000);
    chassis.turnToHeading(CURRENTTEAM*-90, 500, {}, false);
    pros::delay(1000);
    GoalClamp.set_value(false);
    chassis.turnToHeading(CURRENTTEAM*-180, 500, {}, false);
    Intake.move(0);
    pros::delay(1000);
    // Intake.move(-127);
    // chassis.turnToHeading(270, 500);
    // pros::delay(1000);
    // chassis.moveToPoint(-26, -35, 1000, {.forwards = false}, false);
    // GoalClamp.set_value(true);
    /*
    pros::delay(1000);
    chassis.turnToHeading(360, 500);
    Intake.move(-127);
    pros::delay(1000);
    chassis.moveToPose(-6, 14, -270, 1500);
    Doinker.set_value(true);
    pros::delay(1000);
    chassis.moveToPoint(5, 13, 1000);
    GoalClamp.set_value(false);
    pros::delay(1000);
    chassis.turnToHeading(180, 500);
    pros::delay(1000);
    chassis.moveToPoint(9, -38.5, 1000);
    */
}

void AWPsolo() {
    // chassis.setPose(AWPPose);
    chassis.moveToPoint(0, 10, 500);

}

void Skills() { 
    // setpoint = LADYBROWNSCORE;
    // pros::delay(500);
    // setpoint = LADYBROWNDISABLE;
    // chassis.moveToPoint(0, 4, 500);
    // chassis.turnToHeading(180, 500);
    // chassis.resetLocalPosition();
    // chassis.moveToPoint(0, -4, 500);
    // chassis.setPose({0,0,0});
    // chassis.resetLocalPosition();
    chassis.moveToPoint(-0.7, 13, 500);
    chassis.turnToHeading(73, 500);
    chassis.moveToPoint(-60, 0, 500);
    chassis.moveToPose(-50, -8, 70, 1500);
    chassis.moveToPoint(-16.4, 5.5, 1000);
    chassis.turnToHeading(109, 500);
    chassis.moveToPoint(20, -7, 1500);
}