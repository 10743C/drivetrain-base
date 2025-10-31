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
  Brain.Screen.printAt(10, 40, 'robot pre-auton complete');

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // add direct measurement because we dont have any sensors
  // ..........................................................................

 
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
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
