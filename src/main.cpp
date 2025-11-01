/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       sixseven                                                  */
/*    Created:      19/09/2025, 21:20:29                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//admin stuff

#include "vex.h"


//makes it easier to do instead of using say vex::Motor
using namespace vex; 

//--------------------------------------------------------------------------------------------------------------------------------------------

//declare everything

// A global instance of competition
competition Competition;

//declare the brain and controller 
brain Brain;
controller Controller;

//define the drivetrain motors
motor driveleftFront(PORT1, ratio18_1, false);
motor driveleftBack(PORT2, ratio18_1, false);
motor driverightFront(PORT9, ratio18_1, true);
motor driverightBack(PORT10, ratio18_1, true);

//drivetrain motor group
motor_group driveleft(driveleftFront, driveleftBack);
motor_group driveright(driverightFront, driverightBack);

//define the intake motors
motor intakerightFront(PORT8, ratio18_1, false);
motor intakerightMiddle(PORT7, ratio18_1, false);
motor intakeleftMiddle(PORT3, ratio18_1, true);
motor intakeleftBack(PORT4, ratio18_1, true);

//intake motor group
motor_group intake(intakerightFront, intakerightMiddle, intakeleftMiddle, intakeleftBack);

//--------------------------------------------------------------------------------------------------------------------------------------------

//Pre auton functions

void pre_auton(void) {

  //reset encoders
  driveleft.setPosition(0, degrees);
  driveright.setPosition(0, degrees);

  //reset brake modes
  driveleft.setStopping(brake);
  driveright.setStopping(brake);
  intake.setStopping(hold);

  //print to brain            
  Brain.Screen.printAt(10, 40, "robot pre-auton complete");

}

//--------------------------------------------------------------------------------------------------------------------------------------------

//PID drive function

void drivePID(double targetDegrees) {

  /*
    PID formula:
    output = (kP * error) + (kI * integral) + (kD * derivative)

    where:
    error = distance from the goal; target - current
    integral = sum of all past errors
    derivative = how fast error is changing
    output = power sent to motor
  */

  //PID constants
  double kP = 0.5; //kP = Proportional gain; Controls how strongsly the robot reacts to being off target, spring analogy
  double kI = 0.005; //kI = Integral gain; Fixes small leftovers error
  double kD = 0.4; //kD = Derivative gain; Smooths things, reacts to how fast error changing

  /*
    If it doesnt reach target; increase kp
    If it overshoots/ooschillates; incresae kd
    if it stops sort, add tiny ki
  */

  //values to calculate
  double error = 0;
  double integral = 0;
  double derivative = 0;
  double prevError = 0;

  //reset motor encoder
  driveleft.setPosition(0, degrees);
  driveright.setPosition(0, degrees);


  while (true) {
    //measures robot distance
    double leftPos = driveleftFront.position(degrees);
    double rightPos = driverightFront.position(degrees);
    double averagePos = (leftPos + rightPos) / 2.0;

    //calculate values
    error = targetDegrees - averagePos;
    integral += error;
    derivative = error - prevError;

    //calculate power
    double power = (kP * error) + (kI * integral) + (kD * derivative);

    //cap power to prevent overshoot
    if (power > 100) power = 100;
    if (power < -100) power = -100;

    //apply power
    driveleft.spin(fwd, power, pct);
    driveright.spin(fwd, power, pct);

    prevError = error;

    //if close to targe(within 5 degres), stop it
    if (fabs(error) < 5 && fabs(derivative) < 2) {
      break;
    }

    wait(20, msec);

    //brake
    driveleft.stop(brake);
    driveright.stop(brake);
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------

//functions for pid distance conversion in both meters and inches

double meterstodegrees(double distanceM) {
  double wheeldiameterM = 0.1016; // 4in
  double wheelcircumferenceM = M_1_PI * wheeldiameterM;
  return (360.0 * distanceM) / wheelcircumferenceM;
}

double inchestodegrees(double distanceIn) {
  double wheeldiameterIn = 4.0;
  double wheelcircumferenceIn = M_1_PI * wheeldiameterIn;
  return (360.0 * distanceIn) / wheelcircumferenceIn;
}

//--------------------------------------------------------------------------------------------------------------------------------------------

//function for turning pid for both meters and inches

double turndegreestowheelpidM(double turnangleD, double trackwidthM, double wheeldiameterM = 0.1016) {
  return (trackwidthM / wheeldiameterM) * turnangleD; 
}

double turndegreestowheelpidIn(double turnangleD, double trackwidthIn, double wheeldiameterIn = 4.0){
  return (trackwidthIn / wheeldiameterIn) * turnangleD;
}

//--------------------------------------------------------------------------------------------------------------------------------------------

//Autonomous function

void autonomous(void) {
  // ..........................................................................
  // add direct measurement because we dont have any sensors
  // ..........................................................................
  drivePID(1000);
  drivePID(-1000);

  //robot turn right 90 - if left side forward, right backward, it will turn right 
  double target = turndegreestowheelpidIn(90, 12);
  driveleft.spinFor(fwd, target, degrees, false);
  driveright.spinFor(reverse, target, degrees, true);

  Brain.Screen.printAt(10, 80, "went forward 1000 degrees and back");
 
}

//----------------------------------------------------------------------------------------------------------------------------------------------

//User control function (improved)
void usercontrol(void) {

  //left power, right power stores current motor speeds
  int leftPower = 0;
  int rightPower = 0;
  double powerchangepercent = 5; //controls how quickly the power changes every loop, make acceleration smoother

  //detects if intake is running, lastl1, lastr1 detect button presses, edge detection
  bool intakeOn = false;
  bool lastL1 = false;
  bool lastR1 = false;
  //you can now tap l1 once and r1 once to turn on and off for more comfort

  while (true) {

    //read joystick values
    int forward = Controller.Axis2.position(pct);
    int turn = Controller.Axis4.position(pct);

    //deadband ignores joystick small movements smaller than +- 5%
    int deadband = 5;
    if (fabs(forward) < deadband) forward = 0;
    if (fabs(turn) < deadband) turn = 0;

    //applies a exponential curve to joystick input, makes smaller inputs gentler and larger ones full power
    auto curve = [](int input) {
      double scaled = pow(fabs(input / 100.0), 1.5) * 100.0;
      return (input < 0 ? -scaled : scaled);
    };

    int curvedForward = curve(forward);
    int curvedTurn = curve(turn);
    //this curve makes smaller joystick movements more accurate by making it smaller

    //calculate target motor powers
    int targetLeft = curvedForward + curvedTurn;
    int targetRight = curvedForward - curvedTurn;

    //smoothly increases or decreasees power to the target instead of jumping instantly
    if (leftPower < targetLeft)
      leftPower += powerchangepercent;
    else if (leftPower > targetLeft)
      leftPower -= powerchangepercent;

    if (rightPower < targetRight)
      rightPower += powerchangepercent;
    else if (rightPower > targetRight)
      rightPower -= powerchangepercent;

    //dfit to make sure you are driving straigh using pid
    if (fabs(turn) < 5 && fabs(forward) > 10) {
      double error = driveleftFront.position(degrees) - driverightFront.position(degrees);
      double kP = 0.05; //small correction
      double correction = kP * error;
      leftPower -= correction;
      rightPower += correction;
    }

    //when robot is not moving, the brake is applied. When using, the motors coast for smoothness
    if (fabs(forward) < 5 && fabs(turn) < 5) {
      driveleft.stop(brake);
      driveright.stop(brake);
    } else {
      driveleft.setStopping(coast);
      driveright.setStopping(coast);
      driveleft.spin(fwd, leftPower, pct);
      driveright.spin(fwd, rightPower, pct);
    }

    //When l1 is pressed, it turns intake on, when r1 is pressed, it turns it off so you dong have to hold the buttons anymore
    bool L1 = Controller.ButtonL1.pressing();
    bool R1 = Controller.ButtonR1.pressing();

    if (L1 && !lastL1) { // toggle intake on
      intakeOn = true;
    }
    if (R1 && !lastR1) { // toggle intake off
      intakeOn = false;
    }
    lastL1 = L1;
    lastR1 = R1;

    if (intakeOn) {
      intake.spin(fwd, 100, pct);
    } else {
      intake.stop();
    }

    wait(20, msec); // small delay for loop stability
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------

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
