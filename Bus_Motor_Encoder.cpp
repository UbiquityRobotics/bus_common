// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <Bus_Motor_Encoder.h>

//  Loki: Ku 120, Pu 0.27  Kp=0.60Ku  Kd=KpPu/8 Ki=3Kp/Pu
Bus_Motor_Encoder::Bus_Motor_Encoder() {
  _encoder = 0;
  _proportional = 70;
  _derivative = 3;
  _integral = 500;
  _integral_cap = 500;
  _denominator = 100;
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
  //Perror = pid->TargetTicksPerFrame - (pid->Encoder - pid->PrevEnc);
  Short input = _encoder - _previous_encoder;
  Integer Perror = _target_ticks_per_frame - input;

  // Avoid derivative kick and allow tuning changes, see:
  //
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

  Integer output = (_proportional * Perror -
   _derivative * (input - _previous_input) + _integral_term) / _denominator;
  _previous_encoder = _encoder;

  output += _output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= _maximum_pwm)
    output = _maximum_pwm;
  else if (output <= -_maximum_pwm)
    output = -_maximum_pwm;
  else
    // allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   _integral_term += _integral * Perror;
   if (_integral_term > _integral_cap) {
     _integral_term = _integral_cap;
   }
   if (_integral_term < (-_integral_cap)) {
     _integral_term = (-_integral_cap);
   }

  _output = output;
  _previous_input = input;
}

