// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef BUS_MOTOR_ENCODER_H_INCLUDED
#define BUS_MOTOR_ENCODER_H_INCLUDED 1

class Bus_Motor_Encoder {
 public:
  Bus_Motor_Encoder();

  Short denominator_get() {
    return _pid_Kdom;
  };
  void denominator_set(Short pid_Kdom) {
    _pid_Kdom = pid_Kdom;
  };

  Short derivative_get() {
    return _pid_Kd;
  };
  void derivative_set(Short pid_Kd) {
    _pid_Kd = pid_Kd;
  };

  void do_pid();

  virtual void encoder_set(Integer encoder) = 0;

  virtual Integer encoder_get() = 0;

  Short integral_get() {
    return _pid_Ki;
  };
  void integral_set(Short pid_Ki) {
    _pid_Ki = pid_Ki;
  };

  Short integral_cap_get() {
    return _integral_cap;
  };
  void integral_cap_set(Short integral_cap) {
    _integral_cap = integral_cap;
  };

  Logical is_reset() {
    return (Logical)(_previous_rate == 0);
  }

  Integer output_get() {
    return _pwm;
  };
  void output_set(Integer pwm) {
    _pwm = pwm;
  };

  void pid_encoder_set(Integer encoder) {
    _encoder = encoder;
  };
  
  Short proportional_get() {
    return _pid_Kp;
  };
  void proportional_set(Short pid_Kp) {
    _pid_Kp = pid_Kp;
  };

  virtual void pwm_set(Byte pwm) = 0;

  void reset();

  void target_ticks_per_frame_set(Integer speed) {
    _target_ticks_per_frame = (Double)speed;
  };

 private:
  static const Integer _maximum_pwm = 127;
  Short _pid_Kp;	// PID Proportional constant
  Short _pid_Kd;	// PID Differential constant
  Short _pid_Ki;	// PID Integal constant
  Short _pid_Kdom;	// PID common denominator 
  Short _integral_term;	// PID integrated error term
  Short _integral_cap;	// PID Integal term cap

  Integer _pwm;			        // last motor setting
  Integer _previous_pwm;		// last pid loop output value

  Double _target_ticks_per_frame;	// target speed in ticks per frame
  Integer _encoder;			// encoder count
  Integer _previous_encoder;		// last encoder count
  Integer _rate;			// encoder count rate for current frame
  Integer _previous_rate;		// encoder count rate for prior frame

};

#endif //BUS_MOTOR_ENCODER_H_INCLUDED
