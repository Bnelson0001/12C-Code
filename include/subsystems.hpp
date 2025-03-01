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


inline void set_lift(int input) {
  r_LB.move(input);
  l_LB.move(input);
}

inline ez::PID LBPID{0.45, 0, .1, 0, "LB"};

inline void lift_wait() {
  while (LBPID.exit_condition({l_LB, r_LB}, true) == ez::RUNNING) {   //rot_LB.get_position()
    pros::delay(ez::util::DELAY_TIME);
  }
}


inline pros::Motor intake(-4);  // Make this number negative if you want to reverse the motor


inline pros::Optical OpColor (5);

inline ez::Piston doinker('E');
inline ez::Piston doinkerL('D');
inline ez::Piston PIntake('B');
inline ez::Piston MOGOClamp('-A');
inline ez::Piston PColor('F');
