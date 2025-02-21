#include "main.h"

pros::MotorGroup LadyBrown({1, 2});
pros::Motor Intake(20);
pros::Rotation LadyBrownRotation(10);
pros::Rotation LaderalTracking(-9);
pros::Rotation ParrellelTracking(11);
pros::Imu imu(8);
pros::Distance BackDistance(19);
pros::Distance FrontDistance(3);
pros::Optical ColorSort(4);

pros::adi::DigitalOut GoalClamp(8);
pros::adi::DigitalOut Doinker(7);