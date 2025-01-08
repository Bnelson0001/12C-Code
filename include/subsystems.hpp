#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor r_LB(18);

inline void set_lift(int input) {
  r_LB.move(input);
}

inline ez::PID LBPID{0.45, 0, .1, 0, "LB"};

inline void lift_wait() {
  while (LBPID.exit_condition(r_LB, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}
inline pros::Rotation rot_LB (-16);


inline pros::Motor intake(7);  // Make this number negative if you want to reverse the motor

void set_intake(int input);
void intake_opcontrol();

inline void set_intake(int input) {
  intake.move(input);
}

inline ez::Piston doinker('F');
inline ez::Piston PIntake('H');
inline ez::Piston MOGOClamp('A');