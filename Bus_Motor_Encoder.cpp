// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <Bus_Motor_Encoder.h>

Motor_Encoder::Motor_Encoder() {
  Encoder = 0;
  Kp = 20;
  Kd = 12;
  Ki = 0;
  Ko = 50;
  reset();
}

void Motor_Encoder::reset(){
   TargetTicksPerFrame = 0.0;
   // Leave *encoder* field alone:
   PrevEnc = Encoder;
   output = 0;
   PrevInput = 0;
   ITerm = 0;
}

void Motor_Encoder::do_pid() {
  Short input = Encoder - PrevEnc;
  Integer Perror = TargetTicksPerFrame - input;

  // Avoid derivative kick and allow tuning changes, see:
  //
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

  //output =
  // (Kp * Perror + Kd * (Perror - pid->PrevErr) + Ki * pid->Ierror) / Ko;
  // p->PrevErr = Perror;
  Integer delta_output = (Kp * Perror - Kd * (input - PrevInput) + ITerm) / Ko;
  PrevEnc = Encoder;

  output += delta_output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    // allow turning changes, see
    // http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    ITerm += Ki * Perror;

  output = output;
  PrevInput = input;
}

