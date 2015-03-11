// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <Bus_Motor_Encoder.h>

Bus_Motor_Encoder::Bus_Motor_Encoder() {
  _Kp = 20;	// PID Proportional Constant
  _Kd = 12;	// PID Differential Constant
  _Ki = 0;	// PID Integal Constant
  _Ko = 50;	// PID common denOminator 
  reset();
}

void Bus_Motor_Encoder::reset() {
   _target_ticks_per_frame = 0.0;
   // Leave *_encoder* field alone:
   _previous_encoder = _encoder;
   _output = 0;
   _previous_input = 0;
   _integral_term = 0;
}

void Bus_Motor_Encoder::do_pid() {
  Integer Perror;
  Integer output;
  Short input;

  //Perror = pid->TargetTicksPerFrame - (pid->Encoder - pid->PrevEnc);
  input = _encoder - _previous_encoder;
  Perror = _target_ticks_per_frame - input;

  // Avoid derivative kick and allow tuning changes, see:
  //
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

  //output =
  // (Kp * Perror + Kd * (Perror - pid->PrevErr) + Ki * pid->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (_Kp * Perror - _Kd * (input - _previous_input) + _integral_term) / _Ko;
  _previous_encoder = _encoder;

  output += _output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    // allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   _integral_term += _Ki * Perror;

  _output = output;
  _previous_input = input;
}

