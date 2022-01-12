#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

bool backHookOn = false;
void toggleBackHook() {
  if (backHookOn) {
    backHookMotor.rotateTo(0, degrees);
    backHookOn = false;
  } else {
    backHookMotor.rotateTo(90, degrees);
    backHookOn = true;
  }
}

bool leftHookOn = false;
void toggleLeftHook() {
  if (leftHookOn) {
    leftHookMotor.rotateTo(0, degrees);
    leftHookOn = false;
  } else {
    leftHookMotor.rotateTo(90, degrees);
    leftHookOn = true;
  }
}

void onBackBumperPressed() {
  Drivetrain.stop();
  if (!backHookOn) {
    toggleBackHook();
    Controller1.rumble(".");
  }
}

void setupRobot() {
  arm.setBrake(hold);
  arm.setRotation(0, degrees);
  arm.setTimeout(3, seconds);
  leftHookMotor.setBrake(hold);
  leftHookMotor.setRotation(0, degrees);
  leftHookMotor.setTimeout(3, seconds);
  backHookMotor.setBrake(hold);
  backHookMotor.setRotation(0, degrees);
  backHookMotor.setTimeout(3, seconds);
  topHookMotor.setBrake(hold);
  topHookMotor.setRotation(0, degrees);
  topHookMotor.setTimeout(3, seconds);

  Drivetrain.setHeading(0, degrees);
  Drivetrain.setStopping(hold);

  Controller1.ButtonY.pressed(toggleLeftHook);
  Controller1.ButtonB.pressed(toggleBackHook);
  backBumper.pressed(onBackBumperPressed);
}

void slideFor(vex::directionType myDirection, float distance, float speed) {
  float numberDeg;
  numberDeg = (distance / (4 * 3.1415)) * 360 * 1.07;

  leftMotorA.spinFor(myDirection, numberDeg, deg, speed, velocityUnits::pct,
                     false);
  leftMotorB.spinFor(myDirection, -numberDeg, deg, speed, velocityUnits::pct,
                     false);
  rightMotorA.spinFor(myDirection, -numberDeg, deg, speed, velocityUnits::pct,
                      false);
  rightMotorB.spinFor(myDirection, numberDeg, deg, speed, velocityUnits::pct);
}

const int DUMPING_ANGLE = 96;
void dumpRings() {
  arm.spinToPosition(DUMPING_ANGLE, degrees, 20, velocityUnits::pct);
  wait(1, seconds);
}

void pickupRings() {
  topHookMotor.spinToPosition(-180, degrees, 100, velocityUnits::pct);
  arm.spinToPosition(0, degrees, 30, velocityUnits::pct);
  topHookMotor.spinToPosition(0, degrees, 20, velocityUnits::pct);
}


void seesawRings() {
  dumpRings();

  for (int i = 0; i < 4; i++) {
    pickupRings();
    dumpRings();
  }
}

void seesawMogo() {
  double d = 0;
  while (d < 1000) {
    d = backDistance.objectDistance(mm);
    wait(50, msec);
  }

  Drivetrain.driveFor(reverse, d - 40, mm, 70, velocityUnits::pct, false);
  while (!backHookOn) {
    wait(50, msec);
  }
  wait(0.5, seconds);
  Drivetrain.driveFor(forward, 1.5 * 24, inches, 100, velocityUnits::pct);
}

void autonomous(void) {
 // Drivetrain.driveFor(forward, 48, inches, 100, velocityUnits::pct);

  // double drift = Drivetrain.heading();
  // Drivetrain.turnFor(-drift, degrees);

  // slideFor(forward, 24*3, 30);
   seesawRings();
  // Drivetrain.turnFor(19, deg);
  // arm.spinToPosition(45, degrees, 30, velocityUnits::pct);
  // wait(3, seconds);
  //  seesawMogo();
}

void usercontrol(void) {

  arm.setVelocity(10, percent);

  while (true) {

    wait(20, msec);
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  setupRobot();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
