// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef BUS_MOTOR_ENCODER_H_INCLUDED
#define BUS_MOTOR_ENCODER_H_INCLUDED 1

class Bus_Motor_Encoder {
 public:
  Bus_Motor_Encoder();
  void reset();
  void do_pid();
  static const Byte MAX_PWM = 127;
  Short proportional_get() {
    return _Kp;
  };
  void proportional_set(Short proportional) {
    _Kp = proportional;
  };

  Short derivative_get() {
    return _Kd;
  };
  void derivative_set(Short derivative) {
    _Kd = derivative;
  };

  Short integral_get() {
    return _Ki;
  };
  void integral_set(Short integral) {
    _Ki = integral;
  };

  Short denominator_get() {
    return _Ko;
  };
  void denominator_set(Short denominator) {
    _Ko = denominator;
  };

  // private:
  Short _Kp;	// PID Proportional Constant
  Short _Kd;	// PID Differential Constant
  Short _Ki;	// PID Integal Constant
  Short _Ko;	// PID common denOminator 

  Double _target_ticks_per_frame;	// target speed in ticks per frame
  Integer _encoder;			// encoder count
  Integer _previous_encoder;		// last encoder count

  // Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  Short _previous_input;		// last input

  // Using integrated term (ITerm) instead of integrated error (Ierror),
  // to allow tuning changes,
  // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

  Short _integral_term;			//integrated term

  Integer _output;			// last motor setting
};

#endif //BUS_MOTOR_ENCODER_H_INCLUDED
