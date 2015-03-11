// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef BUS_MOTOR_ENCODER_H_INCLUDED
#define BUS_MOTOR_ENCODER_H_INCLUDED 1

class Motor_Encoder {
 public:
  Motor_Encoder();
  void reset();
  void do_pid();

  Short Kp;	// PID Proportional Constant
  Short Kd;	// PID Differential Constant
  Short Ki;	// PID Integal Constant
  Short Ko;	// PID common denOminator 
  static const Byte MAX_PWM = 127;

  Logical is_moving;
  Double TargetTicksPerFrame;	// target speed in ticks per frame
  Integer Encoder;		// encoder count
  Integer PrevEnc;		// last encoder count

  // Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  Short PrevInput;		// last input

  // Using integrated term (ITerm) instead of integrated error (Ierror),
  // to allow tuning changes,
  // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

  Short ITerm;			//integrated term

  Integer output;		// last motor setting
};

#endif //BUS_MOTOR_ENCODER_H_INCLUDED
