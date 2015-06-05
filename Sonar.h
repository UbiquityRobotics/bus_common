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
extern unsigned int  usonar_producerIndex;    // Owned by ISRs and only inspected by consumer
extern unsigned int  usonar_consumerIndex;    // Owned by consumer and only inspected by producer

// System dependent clock for getting microseconds as fast as we can do it
#define USONAR_GET_MICROSECONDS    micros()


// Define max number of sonar units used in this system
#define USONAR_MAX_UNITS    16    // Sonar number as silkscreened on Loki board
#define USONAR_MAX_SPECS    16    // Max measurement specs  (max table of spec entries)

// Define the parameters for measurement of Sonar Units
//
// The numbers below assume lots of sensors being cycled because if a sonar is blocked
// we timeout for about 10meters but the high pulse may be up to 200ms so that is ok
// only if we enable single interrupt pins and let the blocked sensor with 200ms pulse
// be ignored as others are measured.  For very small sensor counts BEWARE!
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
#define USONAR_MAX_DIST_CM        900.0      // a cap used in reading well after meas has finished


// States of sonar sensor acquision
#define  USONAR_STATE_MEAS_START         0
#define  USONAR_STATE_WAIT_FOR_MEAS      1
#define  USONAR_STATE_POST_SAMPLE_WAIT   2

// Because a unit can support either of two methods the measurement method field is a bitmap
// Custom methods once we support them will add to the 2 well known methods of PIN and PCINT
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
typedef struct usonar_meas_spec_t {
    int unitNumber;     // The sonar unit for this entry where 3 would be the N3 sonar
    int measMethod;     // Method to be used for the measurement
    int trigPin;        // For direct pin this is digital pin number for custom it a routine to use
    int echoPin;        // The echo pin in terms of arduino pin to use for pulseIn
    int intRegNum;      // Int reg number for pinint interrupt enable
    int intBit;         // the bit for enable of interrupts for this pinint
} Usonar_Meas_Spec;
extern const Usonar_Meas_Spec  usonar_measSpecs[];

// Because a unit can support either of two methods the measurement method field is a bitmap
// Custom methods once we support them will add to the 2 well known methods of PIN and PCINT
#define US_MEAS_METHOD_NONE    0
#define US_MEAS_METHOD_PIN     1     // Supports direct pulseIn() method
#define US_MEAS_METHOD_PCINT   2     // Supports pin change interrupt method
// Custom modes for trigger start
// Trigger is Port J bit 7
#define US_MEAS_METHOD_T01_PG2 (0x010|US_MEAS_METHOD_PIN|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T10_PJ7 (0x020|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T15_PG4 (0x040|US_MEAS_METHOD_PCINT)
#define US_MEAS_METHOD_T16_PG3 (0x080|US_MEAS_METHOD_PCINT)



// We define pins used on Loki Adruino side platform for Sonar functionalities

// for sonar pins also see the usonar tables we use to lookup
static const int sonar_echo1_pin  = A15;        // IC Pin 82
static const int sonar_echo2_pin  = A14;        // IC Pin 83
static const int sonar_echo3_pin  = A13;        // IC Pin 84
static const int sonar_echo4_pin  = A12;        // IC Pin 85
static const int sonar_echo5_pin  = A11;        // IC Pin 86
static const int sonar_echo6_pin  = A10;        // IC Pin 87
static const int sonar_echo7_pin  = A9;         // IC Pin 88
static const int sonar_echo8_pin  = A8;         // IC Pin 89
static const int sonar_echo9_pin  = 0;          // IC Pin 69
static const int sonar_echo10_pin = 0;          // IC Pin 68
static const int sonar_echo11_pin = 0;          // IC Pin 67
static const int sonar_echo12_pin = 0;          // IC Pin 66
static const int sonar_echo13_pin = 0;          // IC Pin 65
static const int sonar_echo14_pin = 0;          // IC Pin 65
static const int sonar_echo15_pin = 0;          // IC Pin 64
static const int sonar_echo16_pin = 0;          // IC Pin 64

static const int sonar_trig1_pin  = 39;         // IC Pin 70
static const int sonar_trig2_pin  = 22;         // IC Pin 78
static const int sonar_trig3_pin  = 23;         // IC Pin 77
static const int sonar_trig4_pin  = 24;         // IC Pin 76
static const int sonar_trig5_pin  = 25;         // IC Pin 75
static const int sonar_trig6_pin  = 26;         // IC Pin 74
static const int sonar_trig7_pin  = 27;         // IC Pin 73
static const int sonar_trig8_pin  = 28;         // IC Pin 72
static const int sonar_trig9_pin  = 29;         // IC Pin 71
static const int sonar_trig10_pin = 0;          // IC Pin 79
static const int sonar_trig11_pin = 46;         // IC Pin 38
static const int sonar_trig12_pin = 47;         // IC Pin 37
static const int sonar_trig13_pin = 48;         // IC Pin 36
static const int sonar_trig14_pin = 49;         // IC Pin 35
static const int sonar_trig15_pin = 0;          // IC Pin 29
static const int sonar_trig16_pin = 0;          // IC Pin 28

#define ERR_COUNTERS_MAX                 8


// Class to pull together management routines for Loki ultrasonic sensor
// distance measurements
//
// This is really to make things a little bit more contained but does not
// stand on it's own
// This is specific to use of high speed ISR doing acquisition of data
// so this class must use external fast edge detect queue which is not
// 'ideal' but is acceptable
class Sonar {
  public:
    Sonar(UART *debug_uart, RAB_Sonar *rab_sonar);
    int calcQueueLevel(int Pidx, int Cidx, int queueSize);
    int getQueueLevel();
    unsigned long pullQueueEntry();
    int flushQueue();
    int isMeasSpecNumValid(int specNumber);
    int unitNumToMeasSpecNum(int sonarUnit);
    int measSpecNumToUnitNum(int specNumber);
    int getMeasTriggerPin(int specNumber);
    int getInterruptMaskRegNumber(int specNumber);
    int getInterruptBit(int specNumber);
    int getEchoDetectPin(int specNumber);
    int isUnitEnabled(int sonarUnit);
    int getMeasSpec(int specNumber);
    unsigned long measTrigger(int specNumber);
    float echoUsToMeters(unsigned long pingDelay);
    int getLastDistInMm(int sonarUnit);
    float getInlineReadMeters(int sonarUnit);
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
    float sonar_distances_in_meters_[USONAR_MAX_UNITS+1];
    unsigned long sonar_sample_times_[USONAR_MAX_UNITS+1];
};

#endif // SONAR_H_INCLUDED
