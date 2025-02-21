#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

double LADYBROWNkp = 4;
double LADYBROWNkd = 3.5;

double setpoint = 0;
double error = 0;
double proportional;
double derivative;
double output;
double prevError;

bool usePID = false;

void LadyBrownPID() {
    LadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    while (true) {
        if (usePID) {
            // Calculate error
            error = setpoint - (float)LadyBrownRotation.get_position() / 100;

            // Calculate proportional term
            proportional = LADYBROWNkp * error;

            // Calculate derivative term
            derivative = LADYBROWNkd * (error - prevError);

            // Calculate PID output
            output = proportional + derivative;

            // Update previous error
            prevError = error;

            LadyBrown.move(-output);
        } else {
            pros::delay(20);
        }
    }
}

void opcontrol() {
    pros::MotorGroup left_mg({13, 14, 15});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
    pros::MotorGroup right_mg({-16, -17, -18});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

    while (true) {
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
        int turn = 1.5 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick multiplied by 1.5
        left_mg.move(dir + turn);                      // Sets left motor voltage
        right_mg.move(dir - turn);                     // Sets right motor voltage

        // Intake Controls
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !ChangeColor) {
            Intake.move(-127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !ChangeColor) {
            Intake.move(127);
        } else if (!ChangeColor) {
            Intake.move(0);
        }

        // LadyBrown
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            setpoint = LADYBROWNRING1;
            usePID = true;
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            // setpoint = LADYBROWNRING2;
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            setpoint = LADYBROWNSCORE;
            usePID = false;
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            setpoint = LADYBROWNDISABLE;
            usePID = false;
        }

        // Manual control for LadyBrown
        if (!usePID) {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                LadyBrown.move(127);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                LadyBrown.move(-127);
            } else {
                LadyBrown.move(0);
            }
        }

        // Goal Clamp
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            master.print(1, 1, "%i\n", (int)1);
            GoalClamp.set_value(true);
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            GoalClamp.set_value(false);
        }

        // Doinker
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            Doinker.set_value(true);
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            Doinker.set_value(false);
        }

        pros::delay(20);    // Run for 20 ms then update
    }
}