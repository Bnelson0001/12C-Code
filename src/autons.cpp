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




void skills(){ //notebook
  intake.move(127); //start spinning intake
    pros::delay (550); // wait to out take ring on allaince stake
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true); // drive back
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED); // turn to goal
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(20, DRIVE_SPEED, true); // drive to goal
    chassis.pid_wait_until(14_in); //wait until 14 inches, then slow down 
    chassis.pid_speed_max_set(30);
    chassis.pid_wait_until(17_in); // at 17 inches, clamp mogo
  MOGOClamp.set(true); // grab goal
    chassis.pid_wait(); //wait for drive to finish
      pros::delay (250); //wait
  chassis.pid_turn_set(0_deg, DRIVE_SPEED); // turn to first ring
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-12.5_in, DRIVE_SPEED, true); // drive to ring
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(37.5_deg, DRIVE_SPEED); // turn to 2nd ring
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true); // drive to 2nd ring
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(90_deg, DRIVE_SPEED); // turn to line up with allince stake
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_drive_set(-9.5_in, DRIVE_SPEED, true); // drive to ring
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true); // drive back
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true); // drive forward make sure ring is grabbed
  chassis.pid_drive_set(3.5_in, DRIVE_SPEED, true); // drive back
    chassis.pid_wait_quick_chain(); //wait for drive to finish
      pros::delay(250); // wait to out take ring before turning
  chassis.pid_turn_set(181_deg, DRIVE_SPEED); // turn back to other rings
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-50_in, 82, true); // drive foarward to other rings, slower to ensure rings dont jam
    chassis.pid_wait(); //wait for drive to finish
      pros::delay (250); // wait to finish intaking
  chassis.pid_drive_set(-11_in, 82, true); // grab last ring in line
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(9_in, DRIVE_SPEED, true); // drive back
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(70_deg, DRIVE_SPEED); // turn to next ring
    chassis.pid_wait_quick_chain(); // wait for turn to finish
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true); // drive to ring
    chassis.pid_wait_quick_chain(); // wait for drive to finish
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true); //drive back
    chassis.pid_wait_quick_chain(); // wait for drive to finish
  chassis.pid_turn_set(-45_deg, DRIVE_SPEED); // turn to postion goal to corner
    chassis.pid_wait(); // wait for turn to finish
  chassis.pid_drive_set(14_in, DRIVE_SPEED, true); // back goal into corner
    chassis.pid_wait(); // wait for drive to finish
  MOGOClamp.set(false); // release goal
  intake.move(-50); // outake to prevent jamming
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); // drive out of corner
    chassis.pid_wait_quick_chain(); // wait for drive to finish
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED); // turn to face next goal, intake forward
  intake.move(50); // intake slow to grab any dropped rings
    chassis.pid_wait_quick_chain(); // wait for turn to finish
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true); // drive to goal
    chassis.pid_wait_quick_chain(); // wait for drive to finish
  chassis.pid_turn_set(93_deg, DRIVE_SPEED); // turn to face goal
    chassis.pid_wait_quick_chain(); // wait for turn to finish
  chassis.pid_drive_set(25.5_in, DRIVE_SPEED, true);
    chassis.pid_wait_until(21.5_in); //wait until 21.5 inches, then slow down
    chassis.pid_speed_max_set(30);  // slow to 30 speed
    chassis.pid_wait_until(25.1_in); //wait until 25.1 inches, then clamp mogo
  MOGOClamp.set(true); // grab goal
    chassis.pid_wait(); // wait for drive to finish
      pros::delay(100); // wait to ensure clamped
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true); // drive back
    chassis.pid_wait();// wait for drive to finish
  chassis.pid_turn_set(0_deg, DRIVE_SPEED); // turn to ring
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true);  //drive to ring
  intake.move(127); //start intaking
    chassis.pid_wait();// wait for drive to finish
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED); // turn to next ring
    chassis.pid_wait_quick_chain(); // wait for turn to finish
  chassis.pid_drive_set(-23_in, DRIVE_SPEED, true); // drive to ring
    chassis.pid_wait(); // wait for drive to finishq
  chassis.pid_turn_set(180_deg, DRIVE_SPEED); // turn to ring
    chassis.pid_wait_quick_chain(); // wait for turn to finish, quick
  chassis.pid_drive_set(-34_in, 90, true); // drive to rings, slow speed to 90
    chassis.pid_wait();
  chassis.pid_drive_set(3_in, 90, true);// drive forrward to final ring of row, keep speed slow
    chassis.pid_wait();// wait for drive to finish
  chassis.pid_turn_set(-70_deg, DRIVE_SPEED); // return to normal drive speed, turn to final ring
    chassis.pid_wait_quick_chain(); // wait for turn to finish
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); //drive forward to final ring
    chassis.pid_wait_quick_chain(); // wait for drive to finish
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);// drive back to postion to corner
    chassis.pid_wait_quick_chain(); // wait for drive to finish
  chassis.pid_turn_set(60_deg, DRIVE_SPEED); // turn back to corner
    chassis.pid_wait(); //wait for turn to finish         
  chassis.pid_drive_set(7_in, DRIVE_SPEED, true); // drive into corner
    chassis.pid_wait(); // wait for drive to finish
      pros::delay(250); // wait to ensure goal is in corner
  MOGOClamp.set(false); // release goal
  intake.move(-20); //start spinning intake back, anti-jam
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); // drive out of corner
    chassis.pid_wait();
  chassis.pid_turn_set(-3.5_deg, DRIVE_SPEED); // turn to aim for next ring/cross feild
  intake.move(55); //start spinning intake, slow to grab ring
    chassis.pid_wait(); // wait for turn to finish
  chassis.pid_drive_set(-51_in, DRIVE_SPEED, true); // drive to ring
    chassis.pid_wait();// wait for drive to finish
  chassis.pid_turn_set(45_deg, DRIVE_SPEED); // turn to face goal with intake
    chassis.pid_wait(); // wait for turn to finish
  chassis.pid_drive_set(-36_in, DRIVE_SPEED, true); // drive forward to goal
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(200_deg, DRIVE_SPEED); // turn back to goal
    chassis.pid_wait(); // wait for turn to finish
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true); // drive back
  intake.move(10); // slow intake to prevent out taking with out goal
    chassis.pid_wait(); // wait for drive to finish
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED); // turn to face goal, repostion for acuarcy
    chassis.pid_wait(); // wait for turn to finish
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true); //drive to corner, clamp on way
    chassis.pid_wait_until(30_in); // at 30 clamp mogo
  MOGOClamp.set(true); // grab goal
    chassis.pid_wait(); // wait for drive to finish
      pros::delay(250); // wait to ensure clamped
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); // drive backward to corner
    chassis.pid_wait(); // wait for drive to finish
  chassis.pid_turn_set(-115_deg, DRIVE_SPEED); // postion for corner
  intake.move(127); //outake rings on goal
    chassis.pid_wait(); // wait for turn to finish
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); // drive into corner
    chassis.pid_wait(); // wait for drive to finish
  chassis.pid_turn_set(-135_deg, DRIVE_SPEED); // repostion in corner
  intake.move(-20); // outake, anti-jam
    chassis.pid_wait(); // wait for turn to finish
  MOGOClamp.set(false); // release
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true); // drive out of corner
    chassis.pid_wait();// wait for drive to finish
  MOGOClamp.set(true); // close clamp for flat pushing surface
  chassis.pid_drive_set(21_in, DRIVE_SPEED, true); // plow goal into corner to ensure it is scored
    chassis.pid_wait(); // wait for drive to finish
  MOGOClamp.set(false); // release goal, incase of clampo jamming
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true); // drive out of corner
    chassis.pid_wait();// wait for drive to finish
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); // drive back in to ensure corner
    chassis.pid_wait();// wait for drive to finish
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); // drive back in to ensure corner
    chassis.pid_wait();// wait for drive to finish
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED); // turn for final corner plow, cross corner
    chassis.pid_wait();// wait for turn to finish
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true); // drive to nex goal, intake side
    chassis.pid_wait(); // wait for drive to finish
  chassis.pid_turn_set(-65_deg, DRIVE_SPEED); // angle for goal
    chassis.pid_wait();// wait for turn to finish
  chassis.pid_drive_set(-85_in, DRIVE_SPEED, true); // plow goal into corner
}

void solo_blue(){  //notebook
  LBPID.target_set(690); //put up lady brown
    pros::delay(650); //wait to score on allaince stake
  LBPID.target_set(0); // drop down lady brown
  chassis.pid_drive_set(4.1_in, DRIVE_SPEED, true); //  back up 
     chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(-65_deg, DRIVE_SPEED); // angle for goal
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); //drive to goal
  chassis.pid_wait_until(16_in); //wait until 16 inches, then slow down
  chassis.pid_speed_max_set(35); // slow to 35 speed
     chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true); // grab goal
  chassis.pid_turn_set(-170_deg, DRIVE_SPEED); // angle for ring stack
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true); //drive to ring stack 
  intake.move(127); //start spinning intake
     chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-255_deg, DRIVE_SPEED); // turn to center stack
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-13_in, DRIVE_SPEED, true); //drive to center ring stack, grab ring 1 
     chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); //back up
     chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-7_deg, DRIVE_SPEED); // angle for cross feild
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-80_in, DRIVE_SPEED, true); //drive to cross, grab ring stack infront of allaince stake
  chassis.pid_wait_until(40_in); //wait until 40, drop goal
  MOGOClamp.set(false);//drop goal
  chassis.pid_wait_until(45); //wait untill 45, slow down
  chassis.pid_speed_max_set(35);
  chassis.pid_wait_until(70_in); //wait until 70, speed up
  chassis.pid_speed_max_set(DRIVE_SPEED);
     chassis.pid_wait_quick_chain();  //wait for drive to finish
  intake.move(0); //stop spinning intake
  chassis.pid_turn_set(-70_deg, DRIVE_SPEED); // angle for next goal
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true); //drive to goal
  chassis.pid_wait_until(32_in); // wait until 32 inches, then slow down
  MOGOClamp.set(true); // grab goal
  intake.move(127); //start spinning intake
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(20_deg, DRIVE_SPEED); // turn to next ring 
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-27_in, DRIVE_SPEED, true); //drive to ring stack 
     chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true); //drive back
     chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(180_deg, DRIVE_SPEED); // turn to tower
     chassis.pid_wait(); //wait for turn to finish
  LBPID.target_set(500); //put up lady brown to  tough
  chassis.pid_drive_set(-37_in, DRIVE_SPEED, true); //drive center to touch
     chassis.pid_wait(); //wait drive to finish
}

void solo_red(){
  LBPID.target_set(690); //put up lady brown
      pros::delay(650);
  LBPID.target_set(0);
  chassis.pid_drive_set(4.1_in, DRIVE_SPEED, true);
     chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(65_deg, DRIVE_SPEED);
     chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); //drive to goal
  chassis.pid_wait_until(16_in); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(35);
     chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true);
  chassis.pid_turn_set(170_deg, DRIVE_SPEED);
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  intake.move(127); //start spinning intake
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(255_deg, DRIVE_SPEED);
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-13_in, DRIVE_SPEED, true); //drive to ring stack //change made here
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); //drive to ring stack //change made here
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(7_deg, DRIVE_SPEED);
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-80_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  chassis.pid_wait_until(40_in); //wait until 14 inches, then slow down
  MOGOClamp.set(false);
  chassis.pid_wait_until(45); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(35);
  chassis.pid_wait_until(70_in); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(DRIVE_SPEED);
     chassis.pid_wait_quick_chain();  //wait for turn to finish
  intake.move(0); //start spinning intake
  chassis.pid_turn_set(70_deg, DRIVE_SPEED);
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  chassis.pid_wait_until(32_in);
  MOGOClamp.set(true);
  intake.move(127); //start spinning intake
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(-20_deg, DRIVE_SPEED);
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-27_in, DRIVE_SPEED, true); //drive to ring stack //change made here
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true); //drive to ring stack //change made here
     chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(180_deg, DRIVE_SPEED);
     chassis.pid_wait(); //wait for turn to finish
  LBPID.target_set(500); //put up lady brown
  chassis.pid_drive_set(-37_in, DRIVE_SPEED, true); //drive to ring stack //change made here
     chassis.pid_wait(); //wait for turn to finish
}

void rush_ring_blue(){         // notebook
  LBPID.target_set(690); //put up lady brown
    pros::delay(650); //wait to score on allaince stake
  LBPID.target_set(0); // drop down lady brown
  chassis.pid_drive_set(4.1_in, DRIVE_SPEED, true); //  back up 
     chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(-65_deg, DRIVE_SPEED); // turn to goal
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(30.5_in, DRIVE_SPEED, true); //drive to goal
    chassis.pid_wait_until(16_in); //wait until 16 inches, then slow down
  chassis.pid_speed_max_set(35);
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true); // grab goal
  chassis.pid_turn_set(-170_deg, DRIVE_SPEED); // turn to alliance side ring stack
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true); //drive to ring stack 
  intake.move(127); //start spinning intake
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-244_deg, DRIVE_SPEED); // turn to center stack
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true); //drive to center stack, ring 1
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); //back up
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-300_deg, DRIVE_SPEED); // turn to center stack, ring 2
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-6.5_in, DRIVE_SPEED, true); //drive to center stack, ring 2
    chassis.pid_wait_quick_chain();  //wait for turn to finish
  chassis.pid_turn_set(-255_deg, DRIVE_SPEED); // angle for back up
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true); //drive back
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(15.5_in, DRIVE_SPEED, true); //drive to postion for ring stack infront of allaince stake
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-8_deg, DRIVE_SPEED); //turn to ring stack
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-44_in, DRIVE_SPEED, true); //drive to rings
  PIntake.set(true); //raise intake
    chassis.pid_wait(); //wait for drive to finish
     pros::delay(100); // wait to ensure intaking of top ring
  PIntake.set(false); //drop intake
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); //drive forward
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(105_deg, DRIVE_SPEED); // turn to towerr
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-14.8_in, DRIVE_SPEED, true); // drive to tower
    chassis.pid_wait(); //wait for drive to finish
}

void rush_ring_red(){             
  LBPID.target_set(690); //put up lady brown
      pros::delay(700);
  LBPID.target_set(0);
  chassis.pid_drive_set(4.1_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(65_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(30.25_in, DRIVE_SPEED, true); //drive to goal
    chassis.pid_wait_until(16_in); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(35);
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true);
  chassis.pid_turn_set(170_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  intake.move(127); //start spinning intake
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(244_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-11_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(300_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-7.5_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait_quick_chain();  //wait for turn to finish
  chassis.pid_turn_set(255_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(15.5_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(5_deg, DRIVE_SPEED); //turn to face other ring stack
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-42.5_in, DRIVE_SPEED, true); //change made here 
  PIntake.set(true); //raise intake
    chassis.pid_wait(); //wait for drive to finish
      pros::delay(100);
  PIntake.set(false); //raise intake
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); //change made here 
    chassis.pid_wait(); //wait for drive to finish
  intake.move(0); //start spinning intake
  chassis.pid_turn_set(-125_deg, DRIVE_SPEED);
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  intake.move(127); //start spinning intake
  chassis.pid_drive_set(-14.8_in, DRIVE_SPEED, true);
    chassis.pid_wait_until(-5_in); //wait until 25 inches, then slow down
    chassis.pid_wait(); //wait for drive to finish
}

void rush_ring_blue_elims(){        //Notebook
  LBPID.target_set(690); //put up lady brown
    pros::delay(700);
  LBPID.target_set(0);
  chassis.pid_drive_set(4.1_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(-65_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); //drive to goal
    chassis.pid_wait_until(16_in); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(35);
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true);
  chassis.pid_turn_set(-170_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  intake.move(127); //start spinning intake
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(-244_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(-300_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-6_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait_quick_chain();  //wait for turn to finish
  chassis.pid_turn_set(-255_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(-8_deg, DRIVE_SPEED); //turn to face other ring stack
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-44_in, DRIVE_SPEED, true); //change made here 
  PIntake.set(true); //raise intake
    chassis.pid_wait(); //wait for drive to finish
      pros::delay(100);
  PIntake.set(false); //raise intake
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); //drive forward
    chassis.pid_wait(); //wait for drive to finish
      pros::delay(250);
  chassis.pid_turn_set(25_deg, DRIVE_SPEED);
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  intake.move(-60); //start spinning intake
  chassis.pid_drive_set(-45_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
}

void rush_ring_red_elims(){             
  LBPID.target_set(690); //put up lady brown
    pros::delay(700);
  LBPID.target_set(0);
  chassis.pid_drive_set(4.1_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(65_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); //drive to goal
    chassis.pid_wait_until(16_in); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(35);
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true);
  chassis.pid_turn_set(170_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  intake.move(127); //start spinning intake
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(244_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(300_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-6_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait_quick_chain();  //wait for turn to finish
  chassis.pid_turn_set(255_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); //drive to ring stack //change made here
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(5_deg, DRIVE_SPEED); //turn to face other ring stack
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-43_in, DRIVE_SPEED, true); //change made here 
  PIntake.set(true); //raise intake
    chassis.pid_wait(); //wait for drive to finish
      pros::delay(100);
  PIntake.set(false); //raise intake
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); //change made here 
    chassis.pid_wait(); //wait for drive to finish
      pros::delay(250);
  chassis.pid_turn_set(-25_deg, DRIVE_SPEED);
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  intake.move(-60); //start spinning intake
  chassis.pid_drive_set(-45_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
}

void safe_ring(){             
  LBPID.target_set(690); //put up lady brown
    pros::delay(700);
  LBPID.target_set(0);
  chassis.pid_drive_set(4.2_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(-65_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); //drive to goal
    chassis.pid_wait_until(16_in); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(30);
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true);
  chassis.pid_turn_set(-170_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  intake.move(127); //start spinning intake
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(-3.5_deg, DRIVE_SPEED); //turn to face other ring stack
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-46_in, DRIVE_SPEED, true); //change made here
  intake.move(127); //stop intake
  PIntake.set(true); //raise intake
    chassis.pid_wait(); //wait for drive to finish
  PIntake.set(false); //lower intake
  intake.move(127); //stop intake
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(90_deg, DRIVE_SPEED);
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-14_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
}

void mirror_safe_ring(){            
  LBPID.target_set(690); //put up lady brown
    pros::delay(700);
  LBPID.target_set(0);
  chassis.pid_drive_set(4.2_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_turn_set(65_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); //drive to goal
    chassis.pid_wait_until(16_in); //wait until 25 inches, then slow down
  chassis.pid_speed_max_set(30);
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true);
  chassis.pid_turn_set(170_deg, DRIVE_SPEED);
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true); //drive to ring stack //change made here
  intake.move(127); //start spinning intake
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(3.5_deg, DRIVE_SPEED); //turn to face other ring stack
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-46_in, DRIVE_SPEED, true); //change made here
  intake.move(127); //stop intake
  PIntake.set(true); //raise intake
    chassis.pid_wait(); //wait for drive to finish
  PIntake.set(false); //lower intake
  intake.move(127); //stop intake
  chassis.pid_drive_set(-4_in, DRIVE_SPEED, true);
    chassis.pid_wait_quick_chain(); //wait for drive to finish
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-90_deg, DRIVE_SPEED); 
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(-14_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
}

void red_goal(){
  doinker.set(true);
  chassis.pid_drive_set(-38_in, DRIVE_SPEED, true);
    chassis.pid_wait_until(-10_in);
  intake.move(127); //intake
    chassis.pid_wait_until(-38_in);
    chassis.pid_wait(); //wait for drive to finish
  doinker.set(false);
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
    chassis.pid_wait_until(2_in);
  intake.move(0); //intake
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(127_deg, DRIVE_SPEED); 
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);
    chassis.pid_wait_until(3_in);
  chassis.pid_speed_max_set(90); 
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true);
  chassis.pid_turn_set(180_deg, DRIVE_SPEED); 
    chassis.pid_wait(); //wait for drive to finish
  intake.move(127); //intake
  chassis.pid_drive_set(-38_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(115_deg, DRIVE_SPEED); 
    chassis.pid_wait(); //wait for drive to finish
  doinker.set(true);
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(0_deg, DRIVE_SPEED); 
    chassis.pid_wait(); //wait for drive to finish
  doinker.set(false);
  chassis.pid_turn_set(32_deg, DRIVE_SPEED); 
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(-23_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(false);
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(220_deg, DRIVE_SPEED); 
    chassis.pid_wait(); //wait for drive to finish
}

void blue_goal(){  //notebook
  doinker.set(true); // doinker down
  chassis.pid_drive_set(-38_in, DRIVE_SPEED, true);  // drive forward, intaking ring on way
    chassis.pid_wait_until(-20_in); // wait until 20 inches, then start intake
  intake.move(127); //intake on 
    chassis.pid_wait(); //wait for drive to finish
  doinker.set(false); // doinker up, grabs goal
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true); // back up with goal in doinker
    chassis.pid_wait_until(2_in); // wait until 2 inches, then stop intake
  intake.move(0); // stop intake
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_turn_set(-110_deg, DRIVE_SPEED);   // turn to face other goal, other goal will have dropped off doinker
    chassis.pid_wait_quick_chain(); //wait for turn to finish
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true); // drive with back to goal
    chassis.pid_wait_until(15_in); // at 15 inches, slow down
  chassis.pid_speed_max_set(90); // slow down
    chassis.pid_wait(); //wait for drive to finish
  MOGOClamp.set(true); // grab mogo
  chassis.pid_drive_set(-47_in, DRIVE_SPEED, true);// drive to corner
  intake.move(127); //intake ring on to goal
    chassis.pid_wait(); //wait for drive to finish
  doinker.set(true); // doinker down for corner clear
  chassis.pid_turn_set(-150_deg, DRIVE_SPEED);  // turn in corner to clear
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); // drive forward to clear corner
    chassis.pid_wait(); //wait for drive to finish 
  chassis.pid_turn_set(-240_deg, DRIVE_SPEED);  // turn more to clear corner
    chassis.pid_wait(); //wait for turn to finish
  chassis.pid_turn_set(-220_deg, DRIVE_SPEED);   // align to grab cleared rings
    chassis.pid_wait(); //wait for turn to finish
  doinker.set(false);      // doinker up
  chassis.pid_drive_set(-13_in, DRIVE_SPEED, true); // drive forward to grab cleared rings
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(3_in, DRIVE_SPEED, true); // back up
    chassis.pid_wait(); // wait for drive to finish
  chassis.pid_turn_set(-250_deg, DRIVE_SPEED);  // turn to grab next rings 
    chassis.pid_wait(); //wait for drive to finish
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true); // drive to grab the other dropped rings
    chassis.pid_wait();  // wait for drive to finish
  chassis.pid_turn_set(0_deg, DRIVE_SPEED);  // turn to grab more rings
    chassis.pid_wait();  // wait for turn to finish
  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true); // grab rings
    chassis.pid_wait(); // wait for drive to finish
  chassis.pid_turn_set(40_deg, DRIVE_SPEED);  // turn so goal is facing towards allaince wall
    chassis.pid_wait();  // wait for turn to finish
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true); // back up to drop goal
    chassis.pid_wait(); // wait for drive to finish
  MOGOClamp.set(false); // drop goal
  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true); // drive to center to screen goal for start of driver
    chassis.pid_wait(); // wait till drive is finished
  chassis.pid_turn_set(220_deg, DRIVE_SPEED);  // turn to face goal for faster pick up in driver
    chassis.pid_wait(); // wait for turn to finish
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
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED},
                        {{0_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
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