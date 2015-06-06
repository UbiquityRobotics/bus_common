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

// Define max number of sonar units used in this system
#define USONAR_MAX_UNITS    16    // Sonar number as silkscreened on Loki board
#define USONAR_MAX_SPECS    16    // Max meas. specs (max table of spec entries)

// Define the parameters for measurement of Sonar Units
//
// The numbers below assume lots of sensors being cycled because if a sonar
// is blocked we timeout for about 10meters but the high pulse may be up to
// 200ms so that is ok only if we enable single interrupt pins and let the
// blocked sensor with 200ms pulse be ignored as others are measured.
//  For very small sensor counts BEWARE!
//
// 30000 is about 5 meters
#define USONAR_ECHO_MAX       ((long)(28100))   // Longest echo time we expect (28100 is 5  meters)
#define USONAR_MEAS_TIME      ((long)(80000))   // Time to wait per measurement
#define USONAR_MEAS_TOOLONG   ((long)(70000))   // Measurement itself was too long
#define USONAR_SAMPLES_US    ((long)(100000))   // One measurement each time this many uSec goes by
#define USONAR_ECHO_ERR1     ((long)(1*282))
#define USONAR_ECHO_ERR2     ((long)(2*282))
#define USONAR_ECHO_ERR3     ((long)(3*282))
#define USONAR_ECHO_ERR4     ((long)(4*282))
#define USONAR_US_TO_METERS  ((float)(5620.0))  // Standard air nominal uSec delay for 2-way bounce time
#define USONAR_MAX_DIST_CM        900.0         // a cap used in reading well after meas has finished


// States of sonar sensor acquision
#define  USONAR_STATE_MEAS_START         0
#define  USONAR_STATE_WAIT_FOR_MEAS      1
#define  USONAR_STATE_POST_SAMPLE_WAIT   2

// Because a unit can support either of two methods the measurement method 
// field is a bitmap.  Custom methods once we support them will add to the
// 2 well known methods of PIN and PCINT
#define US_MEAS_METHOD_NONE    0
#define US_MEAS_METHOD_PIN     1     // Supports direct pulseIn() method
#define US_MEAS_METHOD_PCINT   2     // Supports pin change interrupt method
// Custom modes for trigger start
// Trigger is Port J bit 7
#define US_MEAS_METHOD_T01_PG2 (0x010|US_MEAS_METHOD_PIN|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T10_PJ7 (0x020|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T15_PG4 (0x040|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T16_PG3 (0x080|US_MEAS_METHOD_PCINT)

#define US_MEAS_METHOD_PIN_PCINT   (US_MEAS_METHOD_PIN|US_MEAS_METHOD_PCINT)

// We found that the nice producer consumer queue has to be reset down wo
// just do one meas per loop and reset queue each time so we get 2 edges
#define  USONAR_ULTRA_FAST_ISR

// Tables to map sonar number to trigger digital line # and echo Axx line
// If entry is 0, not supported yet as it does not have digital pin #
// Need a more clever set of code to deal with all the abnormal pins
typedef struct Sonar__struct {
  int unitNumber;     // The sonar unit for this entry where 3 would be the N3 sonar
  int measMethod;     // Method to be used for the measurement
  int xtrigPin;        // For direct pin this is digital pin number for custom it a routine to use
  int xechoPin;        // The echo pin in terms of arduino pin to use for pulseIn
  int intRegNum;      // Int reg number for pinint interrupt enable
  int intBit;         // the bit for enable of interrupts for this pinint
  volatile uint8_t *trigger_base; // Bass address trigger registers
  UByte trigger_mask; // Mask to use to trigger pin.
  volatile uint8_t *echo_base; // Base address of echo registers
  UByte echo_mask;    // Mask to use to trigger pin.
} Sonar;

#define ERR_COUNTERS_MAX                 8

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
    Sonar_Controller(UART *debug_uart, RAB_Sonar *rab_sonar, Sonar *sonars);
    void ports_initialize();
    int calcQueueLevel(int Pidx, int Cidx, int queueSize);
    int getQueueLevel();
    unsigned long pullQueueEntry();
    int flushQueue();
    int isMeasSpecNumValid(int specNumber);
    int unitNumToMeasSpecNum(int sonarUnit);
    int measSpecNumToUnitNum(int specNumber);
    int xgetMeasTriggerPin(int specNumber);
    int getInterruptMaskRegNumber(int specNumber);
    int getInterruptBit(int specNumber);
    int xgetEchoDetectPin(int specNumber);
    int isUnitEnabled(int sonarUnit);
    int getMeasSpec(int specNumber);
    unsigned long measTrigger(int specNumber);
    float echoUsToMeters(unsigned long pingDelay);
    int getLastDistInMm(int sonarUnit);
    void poll();
  private:
    unsigned long current_delay_data1_;
    unsigned long current_delay_data2_;
    int cycle_number_;
    UART *debug_uart_;
    int error_counters_[ERR_COUNTERS_MAX];
    unsigned long full_cycle_counts_;
    unsigned long measured_trigger_time_;
    int number_sonars_;
    int number_measurement_specs_;
    RAB_Sonar *rab_sonar_;
    UByte sample_state_;
    Sonar *sonars_;
    float sonar_distances_in_meters_[USONAR_MAX_UNITS+1];
    unsigned long sonar_sample_times_[USONAR_MAX_UNITS+1];
    // Constants:
    static const UByte PIN_OFFSET_ = 0;
    static const UByte DDR_OFFSET_ = 1;
    static const UByte PORT_OFFSET_ = 2;
};

#endif // SONAR_H_INCLUDED
