/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       junxiao                                                   */
/*    Created:      Sat Dec 25 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    11, 20, 1, 10, 13
// Controller1          controller
// arm                  motor         12
// DistanceSensor       distance      14
// topHook              motor         2
// leftHook             motor         18
// backHook             motor         19
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

#define K_STRAFE 1.2
#define WHEEL_CIRCUMFERENCE (4 * 3.145)
int drawSide() {

  return 0;
}
void drawTask(int side) {
  if (side == 0) { 
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 240, 120);
    Brain.Screen.setCursor(3, 8);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Rings");
    Brain.Screen.drawRectangle(240, 0, 240, 120);
    Brain.Screen.setCursor(3, 32);
    Brain.Screen.print("Rush 1");
    Brain.Screen.drawRectangle(0, 120, 240, 120);
    Brain.Screen.setCursor(10, 8);
    Brain.Screen.print("Rush 2");
    Brain.Screen.drawRectangle(240, 120, 240, 120);
    Brain.Screen.setCursor(10, 32);
    Brain.Screen.print("Skills");
  } else {
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 240, 120);
    Brain.Screen.setCursor(3, 8);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Rings");
    Brain.Screen.drawRectangle(240, 0, 240, 120);
    Brain.Screen.setCursor(3, 32);
    Brain.Screen.print("Solo");
    Brain.Screen.drawRectangle(0, 120, 240, 120);
    Brain.Screen.setCursor(10, 8);
    Brain.Screen.print("Rush 1");
    Brain.Screen.drawRectangle(240, 120, 240, 120);
    Brain.Screen.setCursor(10, 32);
    Brain.Screen.print("Rush 2");  
  }
}
void menu() {
  int side = drawSide();
  drawTask(side);
}
void strafe(vex::directionType myDirection, float distance, float speed) {
  float numberDeg;
  numberDeg = distance / (4 * 3.1415) * 360  * K_STRAFE;

  leftMotorA.spinFor(myDirection, numberDeg, deg, speed, velocityUnits::pct,
                     false);
  rightMotorA.spinFor(myDirection, -1 * numberDeg, deg, speed,
                      velocityUnits::pct, false);
  leftMotorB.spinFor(myDirection, -1 * numberDeg, deg, speed,
                     velocityUnits::pct, false);
  rightMotorB.spinFor(myDirection, numberDeg, deg, speed, velocityUnits::pct);
}

void turnRight90() {
  Drivetrain.turnFor(90, degrees, 50, velocityUnits::pct, true);
}

void turnLeft90() {
  Drivetrain.turnFor(-90, degrees, 50, velocityUnits::pct, true);
}


void leftHookUp() { leftHook.spinToPosition(90.0, degrees, true); }

void leftHookDown() { leftHook.spinToPosition(0.0, degrees, true); }

bool leftHookIsUp = false;

void toggleLeftHook() {
  if (leftHookIsUp) {
    leftHookDown();
  } else {
    leftHookUp();
  }
  leftHookIsUp = !leftHookIsUp;
}

void backHookUp() { backHook.spinToPosition(90.0, degrees, true); }

void backHookDown() { backHook.spinToPosition(0.0, degrees, true); }

bool backHookIsUp = false;

void toggleBackHook() {
  if (backHookIsUp) {
    backHookDown();
  } else {
    backHookUp();
  }
  backHookIsUp = !backHookIsUp;
}
void awpAuton() {
  Drivetrain.driveFor(4, distanceUnits::in ,true);
  for (int i = 0; i < 4; i++){
    arm.spinToPosition(103, rotationUnits::deg);
    topHook.spinToPosition(i * 360 - 13, rotationUnits::deg);
    wait(0.4, timeUnits::sec); 
    arm.spinToPosition(0, rotationUnits::deg);
    wait(0.4, timeUnits::sec);
    topHook.spinToPosition((i+1) * 360 - 13, rotationUnits::deg);    
  }
  arm.spinToPosition(103, rotationUnits::deg);
  topHook.spinToPosition(5 * 360 - 13, rotationUnits::deg);
  wait(0.4, timeUnits::sec);
  Drivetrain.turnFor(-90, rotationUnits::deg);  
  Drivetrain.driveFor(directionType::fwd, 32, distanceUnits::in);
  strafe(directionType::fwd, 25, 100);
  Drivetrain.driveFor(directionType::rev, 5, distanceUnits::in);
  toggleBackHook();
  Drivetrain.driveFor(30, distanceUnits::in);
}
void seesawAuton(){
  for (int i = 0; i < 4; i++){
    arm.spinToPosition(103, rotationUnits::deg);
    topHook.spinToPosition(i * 360 - 13, rotationUnits::deg);
    wait(0.4, timeUnits::sec); 
    arm.spinToPosition(0, rotationUnits::deg);
    wait(0.4, timeUnits::sec);
    topHook.spinToPosition((i+1) * 360 - 13, rotationUnits::deg);    
  }
  arm.spinToPosition(103, rotationUnits::deg);
  topHook.spinToPosition(5 * 360 - 13, rotationUnits::deg);
  wait(0.4, timeUnits::sec);  
}
void awpRushSafe() { // rush 1
  Drivetrain.driveFor(4, distanceUnits::in ,true);
  arm.spinToPosition(103, rotationUnits::deg);
  topHook.spinToPosition(-13, rotationUnits::deg);
  wait(0.4, timeUnits::sec);
  strafe(directionType::rev, 30, 100);
  Drivetrain.turnFor(180, rotationUnits::deg);
  Drivetrain.driveFor(directionType::rev, 40, distanceUnits::in);
  toggleBackHook();
  Drivetrain.driveFor(directionType::fwd, 40, distanceUnits::in);   
}
void awpRushAggressive(){ // rush 2
  arm.spinToPosition(45, rotationUnits::deg, false);
  Drivetrain.driveFor(directionType::rev, 35, distanceUnits::in);
  toggleBackHook();
  Drivetrain.driveFor(directionType::fwd, 26, distanceUnits::in);
  Drivetrain.turnFor(-90, rotationUnits::deg);
  Drivetrain.driveFor(directionType::fwd, 3, distanceUnits::in);
  arm.spinToPosition(103, rotationUnits::deg);
  topHook.spinToPosition(-13, rotationUnits::deg);
  wait(0.4, timeUnits::sec);  
}
void solo() {
  arm.spinToPosition(103, rotationUnits::deg);
  topHook.spinToPosition(-13, rotationUnits::deg);
  wait(0.4, timeUnits::sec);
  arm.spinToPosition(0, rotationUnits::deg);
  wait(0.4, timeUnits::sec);
  topHook.spinToPosition(360, rotationUnits::deg);
  strafe(directionType::rev, 19, 100);
  Drivetrain.driveFor(directionType::fwd, 94, distanceUnits::in);
  arm.spinToPosition(103, rotationUnits::deg);
  topHook.spinToPosition(347, rotationUnits::deg);      
  wait(0.4, timeUnits::sec);
  Drivetrain.driveFor(directionType::rev, 6, distanceUnits::in);
  Drivetrain.turnFor(-180, rotationUnits::deg);
  Drivetrain.driveFor(directionType::rev, 8, distanceUnits::in);
  toggleBackHook();
  Drivetrain.driveFor(directionType::fwd, 80, distanceUnits::in);  
}
void seesawRushSafe() {
  arm.spinToPosition(103, rotationUnits::deg);
  topHook.spinToPosition(-13, rotationUnits::deg);
  wait(0.4, timeUnits::sec);
  Drivetrain.driveFor(directionType::rev, 5, distanceUnits::in);
  strafe(directionType::rev, 30, 100);
  Drivetrain.driveFor(5, distanceUnits::in);
  strafe(directionType::rev, 3, 100);
  toggleLeftHook();
  strafe(directionType::fwd, 32, 100);
}
void seesawRushAggressive() {
  Drivetrain.driveFor(directionType::rev, 35, distanceUnits::in, 100, velocityUnits::pct);
  toggleBackHook();
  Drivetrain.driveFor(directionType::fwd, 35, distanceUnits::in, 100, velocityUnits::pct);  
}
void skills() {
  awpAuton();
  Drivetrain.driveFor(directionType::fwd, 63, distanceUnits::in);
  strafe(directionType::rev, 8, 100);
  Drivetrain.driveFor(directionType::fwd, 15, distanceUnits::in);
  strafe(directionType::rev, 3, 100);
  toggleLeftHook();
  strafe(directionType::fwd, 3, 100);
  Drivetrain.driveFor(directionType::rev,  75, distanceUnits::in);
  toggleLeftHook();
  Drivetrain.turnFor(-50, rotationUnits::deg);
  Drivetrain.driveFor(directionType::fwd, 60, distanceUnits::in);
  Drivetrain.turnFor(-140, rotationUnits::deg);
  Drivetrain.driveFor(directionType::fwd, 50, distanceUnits::in);
  strafe(directionType::fwd, 19, 100);
  Drivetrain.driveFor(directionType::fwd, 20, distanceUnits::in);
  strafe(directionType::rev, 23, 100);
  toggleLeftHook();
  strafe(directionType::fwd, 24, 100);
  Drivetrain.driveFor(directionType::rev, 70, distanceUnits::in);
  toggleLeftHook();
  strafe(directionType::rev, 6, 100);
  toggleBackHook();
  Drivetrain.driveFor(68, distanceUnits::in);
}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  arm.setStopping(hold);
  backHook.setStopping(hold);
  leftHook.setStopping(hold);
  topHook.setStopping(hold);

  arm.setTimeout(3, seconds);
  backHook.setTimeout(3, seconds);
  leftHook.setTimeout(3, seconds);
  topHook.setTimeout(3, seconds);

  arm.setPosition(0, degrees);
  backHook.setPosition(0, degrees);
  leftHook.setPosition(0, degrees);
  topHook.setPosition(0, degrees);

  arm.setVelocity(10, percent);
  Controller1.ButtonX.pressed(awpAuton);
  Controller1.ButtonY.pressed(toggleLeftHook);
  Controller1.ButtonB.pressed(toggleBackHook);

}
