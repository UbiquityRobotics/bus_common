// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <Bus_Motor_Encoder.h>

//  Loki: Kp=100 Kd=10 Ki=100
Bus_Motor_Encoder::Bus_Motor_Encoder() {
  _encoder = 0;
  _pid_Kp = 100;
  _pid_Kd = 10;
  _pid_Ki = 100;
  _pid_Kdom = 300;
  _integral_term = 0;
  _integral_cap = 100;
  reset();
}

void Bus_Motor_Encoder::reset() {
   _target_ticks_per_frame = 0.0;
   // Leave *_encoder* field alone:
   _previous_encoder = _encoder;
   _integral_term = 0;
   _pwm = 0;
   _previous_pwm = 0;
   _previous_rate = 0;
   _integral_term = 0;
}



//
// Pid control loop math
//
// pwm will be what drives PWM and we will use current and last w combined with standard PID loop.
// rate is tics that have happened in the last sample frame (this can be negative for direction)
// Perror is desired target rate in tics per frame minus tics seen in the just finished frame
// The pwm value is bidirectional out of this routine where negative means backwards.
//
// pwm = prevPwm +   
//
void Bus_Motor_Encoder::do_pid() {
  Integer pwm    = 0;
  Integer rate   = _encoder - _previous_encoder;
  Integer Perror = _target_ticks_per_frame - rate;

  // Reset integral term when target is zero and we are at 0
  if ((_target_ticks_per_frame == 0) && (rate == 0)) {
    _integral_term    = 0;			// We will reset integral error at rest
    _previous_pwm     = 0;
    _previous_rate    = 0;
    _previous_encoder = _encoder;
    _pwm  = 0;
    return;
  }

  Integer pid_delta = ( (_pid_Kp * Perror)                   +
                        (_pid_Kd * (rate - _previous_rate))  + 
                        _integral_term)    
                       / _pid_Kdom;

  pwm  = _previous_pwm + pid_delta;
  if (pwm >= _maximum_pwm) {
    pwm = _maximum_pwm;
    _integral_term = 0;			// We will reset integral error if we go non-linear
  } else if (pwm <= -_maximum_pwm) {
    pwm = -_maximum_pwm;
    _integral_term = 0;			// We will reset integral error if we go non-linear
  } else {
    // Only accumulate integral error if output is in linear range
    _integral_term += _pid_Ki * Perror;
    if (_integral_term > _integral_cap) {
      _integral_term = _integral_cap;
    }
    if (_integral_term < (-_integral_cap)) {
      _integral_term = (-_integral_cap);
    }
  }

  // Set the pwm value in our motor control state
  _pwm = pwm;	

  // Stash the current values away for next pass
  _previous_pwm     = _pwm;
  _previous_encoder = _encoder;
  _previous_rate    = rate;
}

#ifdef OLD_PID_LOOP
  // PID math prior to 9/2015.
  //
  // This code uses a scheme to avoid derivative kick and allow tuning changes, see:
  //
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
void Bus_Motor_Encoder::do_pid() {
  //Perror = pid->TargetTicksPerFrame - (pid->Encoder - pid->PrevEnc);
  Short rate = _encoder - _previous_encoder;
  Integer Perror = _target_ticks_per_frame - rate;


  Integer output = (_pid_Kp * Perror -
   _pid_Kd * (rate - _previous_rate) + _integral_term) / _pid_Kdom;
  _previous_encoder = _encoder;

  output += _pwm;
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

  _pwm = output;
  _previous_rate = rate;
}

#endif //  OLD_PID_LOOP
