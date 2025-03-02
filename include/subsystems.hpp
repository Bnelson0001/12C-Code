#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor r_LB(-2);
inline pros::Motor l_LB(3);
inline pros::Rotation rot_LB (1);
inline pros::ADIDigitalIn LB_Limit ('h');
inline bool auto_intake_control = false;
inline bool auto_outtake_control = false;

inline void set_lift(int input) {
  r_LB.move(input);
  l_LB.move(input);
}
inline bool sort_active = false;
inline ez::PID LBPID{0.6, 0, .1, 0, "LB"};

inline void lift_wait() {
  
  while (LBPID.exit_condition({l_LB, r_LB}, true) == ez::RUNNING) {   //rot_LB.get_position()
    pros::delay(ez::util::DELAY_TIME);
  }
}
inline double target_set = 0;

inline void lift_task() {
  pros::delay(2000);  // Set EZ-Template calibrate before this function starts running
  while (true) {
    set_lift(LBPID.compute(((r_LB.get_position()+l_LB.get_position())/2)));
    pros::delay(ez::util::DELAY_TIME);
  }
}

inline void LB_rest_limit (){
if (LB_Limit.get_value())
pros::delay(100);
r_LB.tare_position();
l_LB.tare_position();
target_set = 0;

}
inline pros::Motor intake(-4);  // Make this number negative if you want to reverse the motor


inline pros::Optical OpColor (5);

inline ez::Piston doinker('E');
inline ez::Piston doinkerL('D');
inline ez::Piston PIntake('B');
inline ez::Piston MOGOClamp('-A');
inline ez::Piston PColor('F');
