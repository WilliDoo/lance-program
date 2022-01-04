#include "vex.h"

#define K_STRAFE 1.2

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT11, ratio18_1, false);
motor leftMotorB = motor(PORT20, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT1, ratio18_1, true);
motor rightMotorB = motor(PORT10, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT13);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart,
                                   DrivetrainInertial, 319.19, 320, 40, mm, 1);
controller Controller1 = controller(primary);
motor arm = motor(PORT12, ratio36_1, true);
distance DistanceSensor = distance(PORT19);
motor topHook = motor(PORT2, ratio18_1, false);
motor leftHook = motor(PORT14, ratio18_1, true);
motor backHook = motor(PORT8, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

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

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  int straight;
  int rotate;
  int drivetrainLeftSideSpeed, drivetrainRightSideSpeed;
  while (true) {
    if (RemoteControlCodeEnabled) {

      if (Controller1.ButtonA.pressing()) {
       //Drivetrain.driveFor(vex::forward, 24, distanceUnits::in, 50, velocityUnits::pct);
        strafe(forward, 24, 50);
      }

      int sideways = Controller1.Axis4.position();
      if (abs(sideways) < 20){
        sideways = 0;
      } else {
        rightMotorA.spin(vex::forward, -sideways, percent);
        leftMotorA.spin(vex::forward, sideways, percent);
        rightMotorB.spin(vex::forward, sideways, percent);
        leftMotorB.spin(vex::forward, -sideways, percent);
      }
      straight = Controller1.Axis3.position();
      rotate = Controller1.Axis1.position();
      if (abs(straight) < 50 && abs(straight) > 10) {
        straight = 10 * straight/abs(straight) ;
      } else if (abs(straight) < 100) {
        straight = 50 * straight/abs(straight);
      } else if (abs(straight) == 100) {
        straight = 100 * straight/abs(straight);  
      }
      drivetrainLeftSideSpeed = straight + rotate;   
      drivetrainRightSideSpeed =  straight - rotate;
      // calculate the drivetrain motor velocities from the controller joystick
      // axies left = Axis3 + Axis1 right = Axis3 - Axis1

      // check if the value is inside of the deadband range
      if (abs(drivetrainLeftSideSpeed) < 10) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left
        // motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (abs(drivetrainRightSideSpeed) < 10) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right
        // motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }

      // only tell the left drive motor to spin if the values are not in the
      // deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the
      // deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control topHook
      if (Controller1.ButtonL1.pressing()) {
        topHook.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        topHook.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        topHook.stop();
        // set the toggle so that we don't constantly tell the motor to stop
        // when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
    //  arm.setVelocity(5,percent);
      // check the ButtonR1/ButtonR2 status to control arm
      if (Controller1.ButtonR1.pressing()) {
        arm.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        arm.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        arm.stop();
        // set the toggle so that we don't constantly tell the motor to stop
        // when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}