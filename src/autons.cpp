#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include "autons.h"

extern lemlib::Chassis chassis;
extern pros::MotorGroup lift;
extern pros::Motor claw;
extern pros::Distance backdistance; 
extern pros::Distance frontdistance;
extern pros::Distance leftdistance;
extern pros::Distance rightdistance;

// float getBack() {
//     return (backdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
// }

// float getFront() {
//     return (frontdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
// }

// float getLeft() {
//     return (leftdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
// }

// float getRight() {
//     return (rightdistance.get()/25.4) + 0; // change these values to adjust for the distance from the sensor to the center of the robot
// }

//main 15 second auton

void mainAuton() {
    
    //start auton
    chassis.setPose(-14.5,-61.5, 135);
    chassis.moveToPoint(-24, -48, 2000, {.forwards = false});

    // lift.move_absolute(50, 70);
    // pros::delay(500);
    // lift.move_absolute(0, 70);
    // claw.move(-120);
    // chassis.swingToheading(90);
    // chassis.setPose(-72+ getLeft(), chassis.getPose().y, chassis.getPose().theta);
    // chassis.moveToPoint(0, -48, 2000);
    // chassis.turnToHeading(180, 2000);
    // chassis.setPose(-72 + getFront(), chassis.getPose().y, chassis.getPose().theta);
    // chassis.moveToPoint(0, -65, 2000);
    // chassis.moveToPoint(0, -60, 2000);
    // chassis.moveToPoint(0, -65, 2000);

}

void pidTest() {
     //insert values as x, y, sensor reading, theta
}