// Copyright (c) 2015 by Mark Johnston.  All rights reserved.
// Copyright (c) 2015 by Wayne Gramlich.  All rights reserved.

#ifndef SONAR_H_INCLUDED
#define SONAR_H_INCLUDED 1

#include <Bus_Slave.h>

class Sonar_Queue {
 public:
  Sonar_Queue(UByte mask_index,
   volatile uint8_t *io_port_base, UART *debug_uart);
  void change_mask_set(UByte change_mask)
   { *change_mask_register_ |= change_mask; };
  void consume_one() { consumer_index_ = (consumer_index_ + 1) & QUEUE_MASK_; };
  UART *debug_uart_get() {return debug_uart_; } ;
  UByte echos_peek() {return echos_[consumer_index_]; };
  //void enable();
  void interrupt_service_routine();
  Logical is_empty() {return producer_index_ == consumer_index_; };
  UByte mask_index_get() { return mask_index_; };
  void shut_down();
  UShort ticks_peek() {return ticks_[consumer_index_]; };
 private:
  static const UByte QUEUE_POWER_ = 4;
  static const UByte QUEUE_SIZE_ = 1 << QUEUE_POWER_;
  static const UByte QUEUE_MASK_ = QUEUE_SIZE_ - 1;
  static const UByte PIN_INDEX_ = 0;
  static const UByte DDR_INDEX_ = 1;
  static const UByte PORT_INDEX_ = 2;

  UByte consumer_index_;
  volatile uint8_t *change_mask_register_;
  UByte echos_[QUEUE_SIZE_];
  UART *debug_uart_;
  volatile uint8_t *io_port_base_;
  UByte interrupt_mask_;
  UByte mask_index_;
  UByte producer_index_;
  UShort ticks_[QUEUE_SIZE_];
};

// Each instance of a *Sonar* class object represents a single
// HC-SR04 sonar object.
class Sonar {
 public:
  // Public constructors and member functions:
  Sonar(volatile uint8_t *trigger_base, UByte trigger_mask,
   Sonar_Queue *sonar_queue, UByte change_bit,
   volatile uint8_t *echo_base, UByte echo_mask);
  UByte change_mask_get() { return change_mask_; };
  UByte echo_mask_get() { return echo_mask_; };
  Sonar_Queue *sonar_queue_get() { return sonar_queue_; };
  Logical is_done() {return (Logical)((state_ == STATE_OFF_) ? 1 : 0); };
  void initialize();
  void time_out();
  void trigger();
  void trigger_setup();
  void measurement_trigger();
  void update(UShort queue_tick, UByte queue_echo, Sonar_Queue *sonar_queue);

  // Public member variables (for now):
  float distance_in_meters;    // Distance in meters
  UInteger sample_time;	       // Sample time

 private:
  // Constants:
  static const UByte PIN_OFFSET_ = 0;       // Offset to port input register
  static const UByte DDR_OFFSET_ = 1;       // Offset to data direcction reg.
  static const UByte PORT_OFFSET_ = 2;      // Offset to port output register
  static const UShort TRIG_PRE_LOW_US_ = 4; // Pre-trigger low hold time
  static const UShort TRIG_HIGH_US_ = 20;   // Trigger high hold time
  static const UByte STATE_OFF_ = 0;
  static const UByte STATE_ECHO_RISE_WAIT_ = 1;
  static const UByte STATE_ECHO_FALL_WAIT_ = 2;
  static const UShort TRIGGER_TICKS_ = 4;   // 4ticks * 4uSec/tick = 16uSec.

  // Private member variables:
  UART *debug_uart_;		// Debugging UART
  UByte state_;			// Sonar state
  UShort echo_start_ticks_;	// Time when echo pulse rose
  UShort echo_end_ticks_;	// Time when echo pulse lowered
  UByte change_mask_;		// Mask for PCINT register
  volatile uint8_t *echo_base_; // Base address of echo registers
  UByte echo_mask_;             // Mask to use to trigger pin.
  Sonar_Queue *sonar_queue_;    // Queue for sonar changes
  volatile uint8_t *trigger_base_; // Bass address trigger registers
  UByte trigger_mask_;          // Mask to use to trigger pin.
};

// There is one instance of a *Sonars_Controller* class object.
// In order to use this class you need to do the following:
// 
// * define the Interrupt Service Routines for the appropraite
//   pin change interrupts.  These routine call the static member
//   function *Sonar::interrupt_handler()*.
//
// * A single *Sonars_Controller* object is defined.  The *Sonars_Controller*
//   constructor takes a null-terminated list of *Sonar* objects.
//
// * The main loop of the embedded program (aka. *loop*() ) must
//   regularaly call *Sonars_Controller::poll()*.
//
// * The latest sonar values are optained with ...
class Sonars_Controller {
 public:
  Sonars_Controller(UART *debug_uart,
   Sonar *sonars[], Sonar_Queue *sonar_queues[], UByte sonars_schedule[]);
  static void interrupt_handler(UByte flags);
  unsigned long measurement_trigger(UByte sonar_index);
  UByte change_mask_get(UByte sonar_index);
  void debug_flags_set(UShort debug_flags);
  void debug_flag_values_set(
   UShort error_flag, UShort general_flag, UShort results_flag);
  UByte echo_mask_get(UByte sonar_index);
  void initialize();
  UByte mask_index_get(UByte sonar_index);
  UShort mm_distance_get(UByte sonar_index);
  void sonar_queues_reset();
  UByte sonars_schedule_size_get() {return sonars_schedule_size_; };
  void poll();

  static const UByte GROUP_END = 250;
  static const UByte SCHEDULE_END = 255;

 private:
  // Constants:
  static const UByte PIN_OFFSET_ = 0;  // Offset to port input register
  static const UByte DDR_OFFSET_ = 1;  // Offset to data direcection register
  static const UByte PORT_OFFSET_ = 2; // Offset to port output register
  // States of sonar sensor acquision:
  static const UByte STATE_MEAS_START_ = 0;
  static const UByte STATE_WAIT_FOR_MEAS_ = 1;
  static const UByte STATE_POST_SAMPLE_WAIT_ = 2;

  // Longest echo time we expect (28100 is 5  meters):
  // Define the parameters for measurement of Sonar Units
  //
  // The numbers below assume lots of sensors being cycled because if a sonar
  // is blocked we timeout for about 10meters but the high pulse may be up to
  // 200ms so that is ok only if we enable single interrupt pins and let the
  // blocked sensor with 200ms pulse be ignored as others are measured.
  //  For very small sensor counts BEWARE!
  //
  // 30000 is about 5 meters
  static const UInteger ECHO_MAXIMUM_ = (UInteger)28100;
  // Time to wait per measurement:
  static const UInteger MEASURE_TIME_ = (UInteger)80000;
  // Measurement itself was too long:
  static const UInteger MEASURE_TOO_LONG_ = (UInteger)70000;
  // One measurement each time this many uSec goes by:
  static const UInteger SAMPLES_US_ = (UInteger)100000;
  static const float US_TO_METERS_ = 5620.0;
  // A cap used in reading well after meas has finished:
  static const float MAXIMUM_DISTANCE_CM_ = 900.0;

  static const UShort TIMEOUT_TICKS_ = 60000;

  // State values for _
  static const UByte STATE_SHUT_DOWN_ = 0;
  static const UByte STATE_GROUP_NEXT_ = 1;
  static const UByte STATE_TRIGGER_SETUP_ = 2;
  static const UByte STATE_TRIGGER_ = 3;
  static const UByte STATE_ECHO_WAIT_ = 4;

  // Member variables:
  unsigned long current_delay_data1_;
  unsigned long current_delay_data2_;
  int cycle_number_;
  UShort debug_flags_;
  UART *debug_uart_;
  unsigned long full_cycle_counts_;
  unsigned long measured_trigger_time_;
  int number_measurement_specs_;
  UByte sample_state_;
  Sonar **sonars_;
  UByte *sonars_schedule_;
  UByte sonars_schedule_size_;
  UByte sonars_size_;
  // Debug flag masks:
  UShort error_debug_flag_;
  UShort general_debug_flag_;
  UShort results_debug_flag_;
  UByte pin_change_interrupts_mask_;
  Sonar_Queue **sonar_queues_;
  UByte sonar_queues_size_;

  UByte first_schedule_index_;
  UByte last_schedule_index_;
  UByte state_;
  UShort start_ticks_;
  UShort now_ticks_;
  UShort previous_now_ticks_;

  // Owned by ISRs and only inspected by consumer:
  static unsigned int producer_index_;
  // Owned by consumer and only inspected by producer:
  static unsigned int consumer_index_;
};

#endif // SONAR_H_INCLUDED
