#include "globals.hpp"
#include "paths.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <math.h>

namespace Paths {
    void SAWP1() {
        chassis.cancelMotion();
        pros::delay(10);
        imu.tare_heading();
        verticalRotation.reset();
        pros::delay(20);
        chassis.setPose(-9, -63, 0, false);
        pros::delay(20);
    }

    void SAWP2() {

    }

    void shortened1() {

    }

    void shortened2() {

    }

    void skillsPath() {

    }

    void runAutonomous() {
        if(currentStartingPos == 0) {
            shortened1();
        } else if(currentStartingPos == 1) {
            shortened2();
        } else if(currentStartingPos == 2) {
            SAWP1();
        } else if(currentStartingPos == 3) {
            SAWP2();
        } else if(currentStartingPos == 4) {
            skillsPath();
        }
    }
}