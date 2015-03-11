// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <Bus_Motor_Encoder.h>

void pid_reset(SetPointInfo *pid){
   pid->_target_ticks_per_frame = 0.0;
   // Leave *encoder* field alone:
   //pid->Encoder = 0;
   pid->_previous_encoder = pid->_encoder;
   pid->_output = 0;
   pid->_previous_input = 0;
   pid->_integral_term = 0;
}

