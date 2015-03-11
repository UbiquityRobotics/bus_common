// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef BUS_MOTOR_ENCODER_H_INCLUDED
#define BUS_MOTOR_ENCODER_H_INCLUDED 1

class Bus_Motor_Encoder {
 public:
  Bus_Motor_Encoder();
  void reset();
  void do_pid();
  Short proportional_get() {
    return _proportional;
  };
  void proportional_set(Short proportional) {
    _proportional = proportional;
  };

  Short derivative_get() {
    return _derivative;
  };
  void derivative_set(Short derivative) {
    _derivative = derivative;
  };

  Short integral_get() {
    return _integral;
  };
  void integral_set(Short integral) {
    _integral = integral;
  };

  Short denominator_get() {
    return _denominator;
  };
  void denominator_set(Short denominator) {
    _denominator = denominator;
  };

  // private:
  static const Integer _maximum_pwm = 127;
  Short _proportional;	// PID Proportional constant
  Short _derivative;	// PID Differential constant
  Short _integral;	// PID Integal constant
  Short _denominator;	// PID common denominator 

  Double _target_ticks_per_frame;	// target speed in ticks per frame
  Integer _encoder;			// encoder count
  Integer _previous_encoder;		// last encoder count

  // Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  Short _previous_input;		// last input

  // Using integrated term (ITerm) instead of integrated error (Ierror),
  // to allow tuning changes,
  // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

  Short _integral_term;			// integral term

  Integer _output;			// last motor setting
};

#endif //BUS_MOTOR_ENCODER_H_INCLUDED
