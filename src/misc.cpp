#include "main.h"



double ColorDetect = 0;
bool ChangeColor = false;

void ColorSorting() {
    while (true) {
        ColorDetect = ColorSort.get_hue();

        if (ColorDetect == OPPOSITETEAM) {
            ChangeColor = true;
            Intake.move(-127);
            pros::delay(COLORSORTDELAY);
            Intake.move(127);
            pros::delay(50);
            ChangeColor = false;
            Intake.move(0);
        }

        pros::delay(20);
    }
}