// Copyright (c) 2015 by Mark Johnston.  All rights reserved.
// Copyright (c) 2015 by Wayne Gramlich.  All rights reserved.

// This is the shared sonar code for the bus_sonar10 and bus_loki boards.
// Class to pull together management routines for Loki ultrasonic sensor
// distance measurements
//
// This is really to make things a little bit more contained but does not
// stand on it's own.  This is specific to use of high speed ISR doing
// acquisition of data so this class must use external fast edge detect
// queue which is not 'ideal' but is acceptable

#include "Arduino.h"
#include "Sonar.h"

Sonar::Sonar() {
  _numSonars    = USONAR_MAX_UNITS;
  _numMeasSpecs = USONAR_MAX_SPECS;
}

// Sonar echo completion Circular Queue interfaces for a background consumer

/// A generic circular queue utility to get number of entries in any circular
// queue.
int Sonar::calcQueueLevel(int Pidx, int Cidx, int queueSize) {
  int queueLevel = 0;
  if (Pidx >= Cidx) {
    // Simple case is Cidx follows Pidx OR empty queue they are equal
    queueLevel = (Pidx - Cidx);
  } else {
    queueLevel = (queueSize - Cidx) + Pidx + 1;
  }
  return queueLevel;
}

// getQueueLevel() returns number of entries in the circular queue but
// changes nothing
int Sonar::getQueueLevel() {
  int localPI = usonar_producerIndex;     // get atomic copy of producer index

  return calcQueueLevel(localPI, usonar_consumerIndex, USONAR_QUEUE_LEN);
}

// Pull one entry from our edge detection circular queue if there are entries.
// A return of 0 will happen if no entries are ready to pull at this time OR
// the entry is 0
unsigned long Sonar::pullQueueEntry() {
  int localPI = usonar_producerIndex;     // get atomic copy of producer index
  unsigned long queueEntry = 0;

  if (calcQueueLevel(localPI, usonar_consumerIndex, USONAR_QUEUE_LEN) > 0) {

    queueEntry = usonar_echoEdgeQueue[usonar_consumerIndex];

    // Find the next index we will bump the consumer index to once done
    int nextCI = usonar_consumerIndex + 1;
    if (nextCI >= USONAR_QUEUE_LEN) {
      nextCI = 0;     // case of roll-around for next entry
    }

    usonar_consumerIndex = nextCI;   // We have consumed so bump consumer index
  }

  return queueEntry;
}

// This empties the queue and no members are seen, they just go bye-bye
//
// We do return how many members were flushed
int Sonar::flushQueue() {
  int queueEntries = 0;
  while (pullQueueEntry() != 0) { queueEntries++; };  // Eat um all up, yum yum.
  return queueEntries;
}


// Helper to check range of measurement spec entries
// 
// Return value of 1 indicates valid meas spec entry number
// Negative value indicates unsupported spec table Entry
int Sonar::isMeasSpecNumValid(int specNumber) {
  if ((specNumber < 0) || (specNumber > _numMeasSpecs)) {
    return -1;
  }
  return 1;
}

// Get the measurement spec number for a given sonar unit number
//
// Negative value indicates the sonar unit number was not found
int Sonar::unitNumToMeasSpecNum(int sonarUnit) {
  int specNumber = -1;

  for (int i = 0; i < _numMeasSpecs; i++) {
    if (usonar_measSpecs[i].unitNumber == sonarUnit) {
      specNumber = i;
    }
  }

  return specNumber;
}

// Get the sonar unit number for a given spec table entry
//
// Negative value indicates unsupported spec table Entry
int Sonar::measSpecNumToUnitNum(int specNumber) {
  int sonarUnit = 0;

  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }

  sonarUnit = usonar_measSpecs[specNumber].unitNumber;

  return sonarUnit;
}

// Get the trigger pin for the given spec table entry
//
// Negative value indicates unsupported spec table Entry
// Zero return 0 indicates this unit does not support trigger line
// a custom trigger is required for the measurement method.
int Sonar::getMeasTriggerPin(int specNumber) {
  int trigPin = 0;

  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }

  // If either of two modes is supported for main modes there is a trigger line
  if (usonar_measSpecs[specNumber].measMethod & US_MEAS_METHOD_PIN_PCINT) {
    trigPin = usonar_measSpecs[specNumber].trigPin;
  }

  return trigPin;
}

// Get the Pin change Interrupt Bank number for a spec table entry
//
// Negative value indicates unsupported spec table Entry
int Sonar::getInterruptMaskRegNumber(int specNumber) {
  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }
  return usonar_measSpecs[specNumber].intRegNum;
}

// Get the interrupt enable bit for the given spec table entry
//
// Negative value indicates unsupported spec table Entry
int Sonar::getInterruptBit(int specNumber) {
  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }
  return usonar_measSpecs[specNumber].intBit;
}

// Get the echo detect pin from sonar unit numberthe given spec table entry
// Echo detect is only used for the inline measurement modes
// which only the lower half of the sonars support.
//
// Negative value indicates unsupported spec table Entry
// Zero return indicates this unit does not support the feature
int Sonar::getEchoDetectPin(int specNumber) {
  int echoPin = 0;

  if (isMeasSpecNumValid(specNumber) < 0) {
   return -1;
  }

  // We have to have this sonar unit at least support echo pin mode
  if (usonar_measSpecs[specNumber].measMethod & US_MEAS_METHOD_PIN) {
    echoPin = usonar_measSpecs[specNumber].echoPin;
  }

  return echoPin;
}

// Indicate if the sonar unit is enabled
//
// NOTE: This is one of a few calls that take in Sonar Unit Number!
//
// Negative value indicates unsupported sonar unit number
//
// Zero return indicates this unit does not support the feature
// Non-zero returns the measurement methods and is thus non-zero
int Sonar::isUnitEnabled(int sonarUnit) {
  int enabled = 0;
  if ((sonarUnit < 1) || (sonarUnit > _numSonars)) {
    return -1;
  }

  // This may seem 'silly' since NONE is zero but in case it changes
  // we will explicitly check with the define
  enabled = usonar_measSpecs[unitNumToMeasSpecNum(sonarUnit)].measMethod;
  if (enabled == US_MEAS_METHOD_NONE) {
    enabled = 0;
  }

  return enabled;
}

// Fetch the measurement spec for a given spec table entry
//
// Negative value indicates unsupported spec table Entry
// A return of < 0 indicates bad sonar unit number
int Sonar::getMeasSpec(int specNumber) {
  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }

  return usonar_measSpecs[specNumber].measMethod;
}


// Trigger the given ultrasonic sonar in the given spec table entry
// The sonar unit number is given to this routine as written on PC board
// This routine shields the caller from knowing the hardware pin
//
// Return value is system clock in microseconds for when the trigger was sent
// Note that the sonar itself will not reply for up to a few hundred
// microseconds
//
#define  USONAR_TRIG_PRE_LOW_US   4     // Time to hold trig line low b4 start
#define  USONAR_TRIG_HIGH_US     20     // Time to hold trigger line high

unsigned long Sonar::measTrigger(int specNumber) {
  unsigned long triggerTime;

  if (isMeasSpecNumValid(specNumber) < 0) {
    return 0;
  }

  triggerTime = USONAR_GET_MICROSECONDS | 1;   // set lsb, zero is an error code

  // Trap out custom trigger pin modes (Ugly but necessary)
  if (usonar_measSpecs[specNumber].measMethod == US_MEAS_METHOD_T01_PG2) {
    PORTG &= 0xfb;
    delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
    PORTG |= 0x04;
    delayMicroseconds(USONAR_TRIG_HIGH_US);
    PORTG &= 0xfb;
  } else if (usonar_measSpecs[specNumber].measMethod == US_MEAS_METHOD_T10_PJ7) {
    PORTJ &= 0x7f;
    delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
    PORTJ |= 0x80;
    delayMicroseconds(USONAR_TRIG_HIGH_US);
    PORTJ &= 0x7f;
  } else if (usonar_measSpecs[specNumber].measMethod == US_MEAS_METHOD_T15_PG4) {
    PORTG &= 0xef;
    delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
    PORTG |= 0x10;
    delayMicroseconds(USONAR_TRIG_HIGH_US);
    PORTG &= 0xef;
  } else if (usonar_measSpecs[specNumber].measMethod == US_MEAS_METHOD_T16_PG3) {
    PORTG &= 0xf7;
    delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
    PORTG |= 0x08;
    delayMicroseconds(USONAR_TRIG_HIGH_US);
    PORTG &= 0xf7;
  } else {
    // Non-custom modes just lookup digital line and do trigger with that
    int trigPin = getMeasTriggerPin(specNumber);
    if (trigPin == 0) {
      return 0;
    }

    digitalWrite(trigPin, LOW);
    delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(USONAR_TRIG_HIGH_US);
    digitalWrite(trigPin, LOW);
  }

  return triggerTime;
}


float Sonar::echoUsToMeters(unsigned long pingDelay) {
  float  meters;
  meters = (float)(pingDelay) / USONAR_US_TO_METERS;
  return meters;
}

// Trigger and readback delay from an HC-SR04 Ulrasonic Sonar
//
// NOTE: This is one of a few calls that take in Sonar Unit Number!
//
// Returns distance in meters
//    0.0 = Bad measurement result (unclear fault of sensor)
//   -1.0 =  invalid sensor number
//   -2.0 =  sensor number is in range but not supported yet
//
// Note that this routine will not cause our background edge triggering
// interrupts as the pulseIn() routine seems to prevent edge interrupts
//
float Sonar::inlineReadMeters(int sonarUnit) {
  float distInMeters = 0.0;
  unsigned long startTics;

  int specNumber = unitNumToMeasSpecNum(sonarUnit);
  if (specNumber < 0) {
    return -9;              // Invalid sonar unit number
  }

  int echoPin = getEchoDetectPin(specNumber);
  if (echoPin < 0) {
    return -2.0;            // sensor number not supported yet for inline read
  }

  startTics = measTrigger(specNumber);    // Trigger the sonar unit to measure
  if (startTics == 0) {
    return -1.0;            // Bad sensor number or not supported yet
  }

  // Wait on the edge detect to measure pulse width in microseconds
  unsigned long duration = pulseIn(echoPin, HIGH, USONAR_ECHO_MAX);
  distInMeters = echoUsToMeters(duration);

  // If we get distance less than spec, return 0 as this can also be a fault
  if (distInMeters < 0.03) {
    distInMeters = 0;
  }

  return distInMeters;
}



