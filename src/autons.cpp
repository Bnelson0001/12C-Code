#include "main.h"
#include "autons.hpp"
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
chassis.pid_drive_constants_forward_set(19.0, 0.0, 85.75);
chassis.pid_drive_constants_backward_set(12.50, 0.125, 82.50);  // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(12.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(6.90, 0.05, 36.0, 15.0);     // Turn in place constants
chassis.pid_swing_constants_forward_set(6.90, 0.0, 55.0);
chassis.pid_swing_constants_backward_set(6.90, 0.0, 55.0);   // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.8);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}




void skills(){         
  chassis.drive_angle_set(45_deg);
  pros::delay(100);
  LBPID.target_set(1350);
  pros::delay(500);
  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(21_in);
  //chassis.pid_turn_set(-1_deg, 10);
  //chassis.pid_wait();
  MOGOClamp.set(true); //clamp first mogo
  auto_intake = true;
  chassis.pid_wait();
  LBPID.target_set(-20); //return lb to home
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true); //drive to first ring
  chassis.pid_wait();
  auto_intake = false;
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait();
  auto_intake = true;
  chassis.pid_drive_set(-21_in, DRIVE_SPEED, true); //used to be 23
  chassis.pid_wait();
  chassis.pid_turn_set(182_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true); //used to be 24
  chassis.pid_wait();
  chassis.pid_turn_set(205_deg, TURN_SPEED); //turn to blue/red ring
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true); //drive to red ring for lb
  chassis.pid_wait();
  //pros::delay(400);
  //LBPID.target_set(180);
  chassis.pid_turn_set(197_deg, TURN_SPEED); //turn before driving to middle of field
  chassis.pid_wait();
  chassis.pid_drive_set(41.5_in, DRIVE_SPEED, true); //drive to middle of field
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED); //turn towards wall stake
  chassis.pid_wait();
  LBPID.target_set(165); //used to be 175
  //intake.move(-100);
  //pros::delay(100);
  //LBPID.target_set(600);
  //intake.move(270);
  chassis.pid_drive_set(-19.25_in, DRIVE_SPEED, true); //drive to wall stake
  //chassis.pid_wait_until(-15_in); //wait until 15 inches into drive to stop intake
  chassis.pid_wait();
  auto_intake = false;
  //chassis.pid_drive_set(.5_in, DRIVE_SPEED, true);
  //chassis.pid_wait();
  //intake.move(-100);
  pros::delay(750);
  LBPID.target_set(1300);
  auto_intake = false;
  pros::delay(800);
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  auto_intake = true;
  chassis.pid_drive_set(-31_in, 70, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22_in, 70, true);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(245_deg, TURN_SPEED);
  chassis.pid_wait();
  auto_intake = true;
  
  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(1.5_in);
  auto_intake = false;
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  auto_intake = false;
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  MOGOClamp.set(false);
  LBPID.target_set(-20);
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(88_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-92_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, 50, true);
  chassis.pid_wait();
  MOGOClamp.set(true);
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  auto_intake = true;
  chassis.pid_turn_set(175_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(136.5_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true); //used to be -29
  chassis.pid_wait();
  LBPID.target_set(145);
  chassis.pid_turn_set(90_deg, TURN_SPEED); //turn to other wall stake
  chassis.pid_wait();
  chassis.pid_drive_set(-16.5_in, DRIVE_SPEED, true); //drive to other wall stake
  chassis.pid_wait();
  pros::delay(750);
  auto_intake = false;
  LBPID.target_set(1300);
  pros::delay(750);
  chassis.pid_drive_set(15.5_in, DRIVE_SPEED, true); //drive back from wall stake
  chassis.pid_wait();
  auto_intake = true;
  chassis.pid_turn_set(0_deg, TURN_SPEED); //turn towards red ring corner
  chassis.pid_wait();
  chassis.pid_drive_set(-60_in, 85, true); //drive to red rings
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-245_deg, TURN_SPEED);
  chassis.pid_wait();
  auto_intake = true;
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(9.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  auto_intake = false;
  auto_outtake = true;
  MOGOClamp.set(false);
  LBPID.target_set(-20);
  chassis.pid_drive_set(-9.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  auto_outtake = false;
  auto_intake = true;
  chassis.pid_turn_set(185_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-62_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait();
  LBPID.target_set(120);
  chassis.pid_drive_set(-33_in, DRIVE_SPEED, true);
  chassis.pid_wait(); 

  chassis.pid_turn_set(44_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, DRIVE_SPEED, true); //used to be 21
  //chassis.pid_wait_until(20.5_in); //used to be 20
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED, 70);
  chassis.pid_wait();
  MOGOClamp.set(true); //clamp first mog
  chassis.pid_wait();
  chassis.pid_drive_set(-3.25_in, DRIVE_SPEED, true); //used to be 2
  chassis.pid_wait();
  auto_intake = false; // intake stop before goal grab for allaince stake
  pros::delay(250);
  chassis.pid_turn_set(177_deg, TURN_SPEED); //turn towards alliance stake
  chassis.pid_wait();
  chassis.pid_drive_set(-16.75_in, DRIVE_SPEED, true); //drive to alliance stake
  chassis.pid_wait();
  chassis.pid_drive_set(6.25_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(600);
  LBPID.target_set(1300); //put ring on alliance stake
  pros::delay(750);
  chassis.pid_drive_set(12.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  LBPID.target_set(500);
  chassis.pid_turn_set(-53_deg, TURN_SPEED);
  chassis.pid_wait();
  auto_intake = true;
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(210_deg, TURN_SPEED);
  chassis.pid_wait();
  doinker.set(true);
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true); //drive to corner
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED); //turn around
  chassis.pid_wait_quick_chain();
  doinker.set(false);
  chassis.pid_drive_set(22_in, DRIVE_SPEED, true); //drive to corner, mogo facing corner
  chassis.pid_wait_quick_chain();
  MOGOClamp.set(false); //drop goal in corner
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true); //drive forward
  chassis.pid_wait();
  chassis.pid_turn_set(-85_deg, TURN_SPEED); //turn towards pushy goal
  chassis.pid_wait();
  chassis.pid_drive_set(105_in, 110, true); //push goal into other corner
  chassis.pid_wait();
  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true); //drive away from corner
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(135_deg, TURN_SPEED); //turn towards ladder
  chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

}

void solo_blue(){ 
  chassis.drive_angle_set(45_deg);
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(50); //used to be 200
  LBPID.target_set(1300);
  pros::delay(400); //used to be 400
  chassis.pid_drive_set(15.5_in, DRIVE_SPEED, true); //prob add slow here
  chassis.pid_wait();
  //chassis.pid_turn_set(-.5_deg, TURN_SPEED);
  //chassis.pid_wait();
  //LBPID.target_set(-20);
  //chassis.pid_drive_set(16.5_in, 50, true);
  //chassis.pid_wait();
  //pros::delay(150);
  //MOGOClamp.set(true);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(15_in);
  MOGOClamp.set(true);
  auto_intake = true;
  chassis.pid_turn_set(-110_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(17_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-147_deg, TURN_SPEED);
  chassis.pid_wait();
  doinker.set(true);
  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(37_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  doinker.set(false);
  chassis.pid_turn_set(78_deg, TURN_SPEED); //for regular match, make this 105
  chassis.pid_wait();
  MOGOClamp.set(false);
  //PIntake.set(true);
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  auto_intake = false;
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, 50, true);
  chassis.pid_wait_until(19);
  MOGOClamp.set(true);
  auto_intake = true;
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  LBPID.target_set(1100);
  chassis.pid_turn_set(-135_deg, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, 50, true);
  chassis.pid_wait();

}

void solo_red(){
}

void test_slot(){ //currently alternative solo awp
  chassis.drive_angle_set(60_deg);
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(200); //used to be 200
  LBPID.target_set(1300);
  pros::delay(400); //used to be 400
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); //prob add slow here
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  LBPID.target_set(-20);
  chassis.pid_drive_set(23_in, 70, true);
  chassis.pid_wait();
  pros::delay(150);
  MOGOClamp.set(true);
  auto_intake = true;
  chassis.pid_turn_set(-140_deg, TURN_SPEED);
  chassis.pid_wait();
  doinker.set(true);
  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  auto_intake = false;
  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  auto_intake = true;
  doinker.set(false);
  pros::delay(400);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  //doinker.set(false);
  chassis.pid_drive_set(-24_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  MOGOClamp.set(false);
  chassis.pid_drive_set(-20_in, 90, true);
  chassis.pid_wait_quick_chain();
  auto_intake = false;
  chassis.pid_drive_set(-20_in, 90, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-20_deg, TURN_SPEED);
  chassis.pid_wait();
}

void blue_ring(){
  //set starting angle
  chassis.drive_angle_set(-170_deg);

  //put down doinker and start intake
  doinker.set(true);
  auto_intake = true;

  //drive to 4 ring stack
  chassis.pid_drive_set(-42_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  //wait a bit and then turn off intake
  pros::delay(200);
  auto_intake = false;

  //swing to mogo and clamp
  chassis.pid_swing_set(ez::LEFT_SWING, -85_deg, SWING_SPEED, 23);
  chassis.pid_wait_until(-75_deg);
  doinker.set(false);
  chassis.pid_wait();
  pros::delay(400);

  //move back a little and clamp
  chassis.pid_drive_set(3_in, 50, true);
  chassis.pid_wait();
  MOGOClamp.set(true);

  //turn to line up with rings
  doinker.set(false);
  pros::delay(200);
  chassis.pid_turn_set(-80_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive over rings
  auto_intake = true;
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //turn towards corner
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to corner
  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //drive back
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //turn towards preload
  chassis.pid_turn_set(75_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to preload and then middle ring
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void red_ring(){
}

void blue_goal(){
  //set starting angle
  chassis.drive_angle_set(0_deg);

  //drive to mogo and clamp
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(23_in);
  MOGOClamp.set(true);

  //intake preload
  auto_intake = true;

  //turn towards ladder
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();
  auto_intake = false;

  //drive to center rings and put down first doinker
  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  doinkerL.set(true);
}

void red_goal(){
}

void blue_ring_safe(){             
}

void red_ring_safe(){            
}

void blue_goal_safe(){
  //set starting angle
  chassis.drive_angle_set(-45_deg);

  //drive to alliance stake
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //put ring on alliance stake
  pros::delay(50);
  LBPID.target_set(1300);
  pros::delay(400);

  //drive back
  chassis.pid_drive_set(15.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //turn towards mogo
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to mogo and clamp
  chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(15_in);
  MOGOClamp.set(true);

  //turn on intake and turn to rings
  auto_intake = true;
  chassis.pid_turn_set(110_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to rings
  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //drive back
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //turn towards corner rings
  chassis.pid_turn_set(39_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to corner rings
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  //wait to move back
  pros::delay(200);

  //turn towards ladder
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to ladder
  LBPID.target_set(1100);
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void red_goal_safe(){
  //set starting angle
  chassis.drive_angle_set(45_deg);

  //drive to alliance stake
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //put ring on alliance stake
  pros::delay(50);
  LBPID.target_set(1300);
  pros::delay(400);

  //drive back
  chassis.pid_drive_set(15.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //turn towards mogo
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to mogo and clamp
  chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(15_in);
  MOGOClamp.set(true);

  //turn on intake and turn to rings
  auto_intake = true;
  chassis.pid_turn_set(-110_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to rings
  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //drive back
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //turn towards corner rings
  chassis.pid_turn_set(-39_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to corner rings
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  //wait to move back
  pros::delay(200);

  //turn towards ladder
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();

  //drive to ladder
  LBPID.target_set(1100);
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}







void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  // chassis.pid_drive_set(50_in, DRIVE_SPEED, true);
  // doinker.button_toggle(true);
  // chassis.pid_wait_quick_chain();

  // doinker.button_toggle(false);

  // chassis.pid_drive_set(-30_in, DRIVE_SPEED);
  // chassis.pid_wait_quick_chain();

  // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  chassis.pid_odom_injected_pp_set({{{0_in, 24_in}, fwd, 110},
    {{24_in, 24_in}, fwd, 110}}, 
    true);

  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                   //     {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
  
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .