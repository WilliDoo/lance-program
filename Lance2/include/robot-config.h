using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor arm;
extern motor leftHookMotor;
extern motor backHookMotor;
extern motor topHookMotor;

extern motor leftMotorA;
extern motor leftMotorB;
extern motor rightMotorA;
extern motor rightMotorB;


extern controller Controller1;
extern smartdrive Drivetrain;
extern motor  leftMotorB;

extern bumper backBumper;
extern bumper sideBumper;
extern distance sideDistance;
extern distance backDistance;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

