// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef BUS_MOTOR_ENCODER_H_INCLUDED
#define BUS_MOTOR_ENCODER_H_INCLUDED 1

typedef struct {
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
} SetPointInfo;

void motor_speeds_set(Byte left_speed, Byte right_speed);
void pid_reset(SetPointInfo *pid);


#endif //BUS_MOTOR_ENCODER_H_INCLUDED
