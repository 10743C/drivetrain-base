/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       sixseven                                                  */
/*    Created:      19/09/2025, 21:20:29                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

//makes it easier to do instead of using say vex::Motor
using namespace vex; 

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


void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

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


void autonomous(void) {
  // ..........................................................................
  // add direct measurement because we dont have any sensors
  // ..........................................................................
  drivePID(1000);
  drivePID(-1000);

  Brain.Screen.printAt(10, 80, "went forward 1000 degrees and back");
 
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop


  while (true) {

    //read joystick values
    int forward = Controller.Axis2.position(pct);
    int turn = Controller.Axis4.position(pct);

    //calculate motor powers
    int leftPower = forward + turn;
    int rightPower = forward - turn;

    //apply to drivetrain
    driveleft.spin(fwd, leftPower, pct);
    driveright.spin(fwd, rightPower, pct);

    //intake control from l1, r1
    if (Controller.ButtonL1.pressing()) {
      intake.spin(fwd, 100, pct);
    } else if (Controller.ButtonR1.pressing()) {
      intake.spin(reverse, 100, pct);
    } else {
      intake.stop();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
