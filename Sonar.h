// Copyright (c) 2015 by Mark Johnston.  All rights reserved.
// Copyright (c) 2015 by Wayne Gramlich.  All rights reserved.

#ifndef SONAR_H_INCLUDED
#define SONAR_H_INCLUDED 1

#include <Bus_Slave.h>

class Sonar_Queue {
 public:
  Sonar_Queue(UByte mask_index, volatile uint8_t *io_port_base);
  void interrupt_service_routine();
 private:
  static const UByte QUEUE_POWER_ = 4;
  static const UByte QUEUE_SIZE_ = 1 << QUEUE_POWER_;
  static const UByte QUEUE_MASK_ = QUEUE_SIZE_ - 1;
  static const UByte PIN_INDEX_ = 0;
  static const UByte DDR_INDEX_ = 1;
  static const UByte PORT_INDEX_ = 2;

  volatile uint8_t *io_port_base_;
  UByte mask_index_;
  UByte producer_index_;
  UByte consumer_index_;
  UByte changes_[QUEUE_SIZE_];
  UShort ticks_[QUEUE_SIZE_];
};

// Each instance of a *Sonar* class object represents a single
// HC-SR04 sonar object.
class Sonar {
 public:
  // Public constructors and member functions:
  Sonar(UByte interrupt_register_number, UByte interrupt_bit,
   volatile uint8_t *trigger_base, UByte trigger_mask,
   Sonar_Queue *sonar_queue,
   volatile uint8_t *echo_base, UByte echo_mask);
  UByte echo_mask_get() { return echo_mask_; };
  Sonar_Queue *sonar_queue_get() { return sonar_queue_; };
  void initialize();
  void measurement_trigger();

  // Public member variables (for now):
  UByte intRegNum;             // Int reg number for pinint interrupt enable
  UByte intBit;                // bit for enable of interrupts for this pinint
  float distance_in_meters;    // Distance in meters
  UInteger sample_time;	       // Sample time

 private:
  // Constants:
  static const UByte PIN_OFFSET_ = 0;       // Offset to port input register
  static const UByte DDR_OFFSET_ = 1;       // Offset to data direcction reg.
  static const UByte PORT_OFFSET_ = 2;      // Offset to port output register
  static const UShort TRIG_PRE_LOW_US_ = 4; // Pre-trigger low hold time
  static const UShort TRIG_HIGH_US_ = 20;   // Trigger high hold time

  // Private member variables:
  volatile uint8_t *echo_base_; // Base address of echo registers
  UByte echo_mask_;             // Mask to use to trigger pin.
  Sonar_Queue *sonar_queue_;    // Queue for sonar changes
  volatile uint8_t *trigger_base_; // Bass address trigger registers
  UByte trigger_mask_;          // Mask to use to trigger pin.
};

// There is one instance of a *Sonar_Controller* class object.
// In order to use this class you need to do the following:
// 
// * define the Interrupt Service Routines for the appropraite
//   pin change interrupts.  These routine call the static member
//   function *Sonar::interrupt_handler()*.
//
// * A single *Sonar_Controller* object is defined.  The *Sonar_Controller*
//   constructor takes a null-terminated list of *Sonar* objects.
//
// * The main loop of the embedded program (aka. *loop*() ) must
//   regularaly call *Sonar_Controller::poll()*.
//
// * The latest sonar values are optained with ...
class Sonar_Controller {
 public:
  Sonar_Controller(UART *debug_uart, Sonar *sonars[]);
  static void interrupt_handler(UByte flags);
  unsigned long measurement_trigger(UByte sonar_index);
  void debug_flags_set(UShort debug_flags);
  void debug_flag_values_set(
   UShort error_flag, UShort general_flag, UShort results_flag);
  void initialize();
  UShort mm_distance_get(UByte sonar_index);
  void poll();

  int calcQueueLevel(int Pidx, int Cidx, int queueSize);
  int getQueueLevel();
  unsigned long pullQueueEntry();
  int flushQueue();
  int isMeasSpecNumValid(int specNumber);
  int getInterruptMaskRegNumber(int specNumber);
  int getInterruptBit(int specNumber);
  float echoUsToMeters(unsigned long pingDelay);
 private:
  // Constants:
  static const UByte PIN_OFFSET_ = 0;  // Offset to port input register
  static const UByte DDR_OFFSET_ = 1;  // Offset to data direcection register
  static const UByte PORT_OFFSET_ = 2; // Offset to port output register
  // States of sonar sensor acquision:
  static const UByte STATE_MEAS_START_ = 0;
  static const UByte STATE_WAIT_FOR_MEAS_ = 1;
  static const UByte STATE_POST_SAMPLE_WAIT_ = 2;
  static const UByte ERROR_COUNTERS_SIZE_ = 8;
  static const UByte QUEUE_SIZE_ = 8;             // MUST be a power of 2
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
  static const UInteger ECHO_ERROR1_ = (UInteger)(1*282);
  static const UInteger ECHO_ERROR2_ = (UInteger)(2*282);
  static const UInteger ECHO_ERROR3_ = (UInteger)(3*282);
  static const UInteger ECHO_ERROR4_ = (UInteger)(4*282);
  // Standard air nominal uSec delay for 2-way bounce time:
  static const float US_TO_METERS_ = 5620.0;
  // A cap used in reading well after meas has finished:
  static const float MAXIMUM_DISTANCE_CM_ = 900.0;

  // Member variables:
  unsigned long current_delay_data1_;
  unsigned long current_delay_data2_;
  int cycle_number_;
  UShort debug_flags_;
  UART *debug_uart_;
  int error_counters_[ERROR_COUNTERS_SIZE_];
  unsigned long full_cycle_counts_;
  unsigned long measured_trigger_time_;
  int number_measurement_specs_;
  UByte sample_state_;
  Sonar **sonars_;
  UByte sonars_size_;
  // Debug flag masks:
  UShort error_debug_flag_;
  UShort general_debug_flag_;
  UShort results_debug_flag_;
  UByte pin_change_interrupts_mask_;

  // Owned by ISRs and only inspected by consumer:
  static unsigned int producer_index_;
  // Owned by consumer and only inspected by producer:
  static unsigned int consumer_index_;
  static unsigned long echo_edge_queue_[QUEUE_SIZE_];
  static unsigned long echo_info_queue_[QUEUE_SIZE_];
};

#endif // SONAR_H_INCLUDED
