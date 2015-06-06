// Copyright (c) 2015 by Mark Johnston.  All rights reserved.
// Copyright (c) 2015 by Wayne Gramlich.  All rights reserved.

#ifndef SONAR_H_INCLUDED
#define SONAR_H_INCLUDED 1

#include <Bus_Slave.h>
#include <RAB_Sonar.h>

// We require these externally defined indexes due to need for super fast ISR
#define  USONAR_QUEUE_LEN     8             // MUST be a power of 2
extern unsigned long usonar_echoEdgeQueue[USONAR_QUEUE_LEN];
extern unsigned long usonar_echoInfoQueue[USONAR_QUEUE_LEN];

// Owned by ISRs and only inspected by consumer:
extern unsigned int  usonar_producerIndex;
// Owned by consumer and only inspected by producer:
extern unsigned int  usonar_consumerIndex;

// System dependent clock for getting microseconds as fast as we can do it
#define USONAR_GET_MICROSECONDS    micros()

// We found that the nice producer consumer queue has to be reset down wo
// just do one meas per loop and reset queue each time so we get 2 edges
#define  USONAR_ULTRA_FAST_ISR

// Tables to map sonar number to trigger digital line # and echo Axx line
// If entry is 0, not supported yet as it does not have digital pin #
// Need a more clever set of code to deal with all the abnormal pins
class Sonar {
 public:
  Sonar(UByte unit_number, UByte interrupt_register_number,
   UByte interrupt_bit,
   volatile uint8_t *trigger_base, UByte trigger_mask,
   volatile uint8_t *echo_base, UByte echo_mask);
  void ports_initialize();
  void measurement_trigger();
  // Member variables:
  UByte unitNumber;            // Sonar unit (e.g 3 => connector N3 on Loki)
  UByte intRegNum;             // Int reg number for pinint interrupt enable
  UByte intBit;                // bit for enable of interrupts for this pinint
  float distance_in_meters;    // Distance in meters
  UInteger sample_time;	       // Sample time
 private:
  static const UByte PIN_OFFSET_ = 0;       // Offset to port input register
  static const UByte DDR_OFFSET_ = 1;       // Offset to data direcction reg.
  static const UByte PORT_OFFSET_ = 2;      // Offset to port output register
  static const UShort TRIG_PRE_LOW_US_ = 4; // Pre-trigger low hold time
  static const UShort TRIG_HIGH_US_ = 20;   // Trigger high hold time
  // Member variables:
  volatile uint8_t *trigger_base_; // Bass address trigger registers
  UByte trigger_mask_;          // Mask to use to trigger pin.
  volatile uint8_t *echo_base_; // Base address of echo registers
  UByte echo_mask_;             // Mask to use to trigger pin.
};

// Class to pull together management routines for Loki ultrasonic sensor
// distance measurements
//
// This is really to make things a little bit more contained but does not
// stand on it's own
// This is specific to use of high speed ISR doing acquisition of data
// so this class must use external fast edge detect queue which is not
// 'ideal' but is acceptable
class Sonar_Controller {
 public:
  Sonar_Controller(UART *debug_uart, RAB_Sonar *rab_sonar, Sonar *sonars[]);
  void ports_initialize();
  unsigned long measurement_trigger(UByte sonar_index);
  int calcQueueLevel(int Pidx, int Cidx, int queueSize);
  int getQueueLevel();
  unsigned long pullQueueEntry();
  int flushQueue();
  int isMeasSpecNumValid(int specNumber);
  int unitNumToMeasSpecNum(int sonarUnit);
  int measSpecNumToUnitNum(int specNumber);
  int getInterruptMaskRegNumber(int specNumber);
  int getInterruptBit(int specNumber);
  float echoUsToMeters(unsigned long pingDelay);
  int getLastDistInMm(int sonarUnit);
  void poll();
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
  UART *debug_uart_;
  int error_counters_[ERROR_COUNTERS_SIZE_];
  unsigned long full_cycle_counts_;
  unsigned long measured_trigger_time_;
  int number_measurement_specs_;
  RAB_Sonar *rab_sonar_;
  UByte sample_state_;
  Sonar **sonars_;
  UByte sonars_size_;
};

#endif // SONAR_H_INCLUDED
