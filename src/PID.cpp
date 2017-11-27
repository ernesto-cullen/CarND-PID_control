#include <iostream>
#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() = default;

PID::~PID() = default;

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kpi = Kp;
  PID::Kp = Kp;
  PID::Kd = Kd;
  PID::Ki = Ki;
  PID::p_error = 0.0d;
  PID::d_error = 0.0d;
  PID::i_error = 0.0d;
  PID::prev_cte = 0.0d;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  d_error = (cte-prev_cte);
  i_error += cte;
  prev_cte = cte;
}

double PID::getSteerValue(double cte) {
  UpdateError(cte);
  double steer = -(Kp * p_error + Kd * d_error + Ki * i_error);
  prev_cte = cte;
  sum_cte += cte;
  if (steer < -1.0) steer = -1.0;
  if (steer > 1.0) steer = 1.0;
  return steer;
}

