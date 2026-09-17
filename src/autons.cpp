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


}

void pidTest() {
     //insert values as x, y, sensor reading, theta
}