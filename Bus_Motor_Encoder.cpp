// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <Bus_Motor_Encoder.h>



//  Loki: 20150907 mjstn  Kp 30 Kd 7 Ki 25 Ko 600 Ci 400
Bus_Motor_Encoder::Bus_Motor_Encoder() {
  _encoder = 0;
  _pid_Kp = 30;
  _pid_Kd = 7;
  _pid_Ki = 25;
  _pid_Kdom = 600;
  _integral_cap = 100;
  _integral_term = 0;
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
void Bus_Motor_Encoder::do_pid() {
  Integer pwm    = 0;
  _rate          = _encoder - _previous_encoder;
  _perr          = _target_ticks_per_frame - _rate;

  // Reset integral term when target is zero and we are at 0
  if ((_target_ticks_per_frame == 0) && (_rate == 0)) {
    _integral_term    = 0;			// We will reset integral error at rest
    _previous_pwm     = 0;
    _rate             = 0;
    _previous_rate    = 0;
    _previous_encoder = _encoder;
    _pwm  = 0;
    return;
  }

  _pid_delta = ( (_pid_Kp * _perr)                   +
                 (_pid_Kd * (_rate - _previous_rate))  + 
                 _integral_term)    
                  / _pid_Kdom;


  pwm  = _previous_pwm + _pid_delta;
  if (pwm >= _maximum_pwm) {
    pwm = _maximum_pwm;
    _integral_term = 0;			// We will reset integral error if we go non-linear
  } else if (pwm <= -_maximum_pwm) {
    pwm = -_maximum_pwm;
    _integral_term = 0;			// We will reset integral error if we go non-linear
  } else {
    // Only accumulate integral error if output is in linear range
    _integral_term += _pid_Ki * _perr;           
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
  _previous_rate    = _rate;
}

