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
#include "RAB_Sonar.h"
#include "bus_server.h"

static int  usonarSampleState = USONAR_STATE_MEAS_START;
// The measurement cycle number (does not have to be sonar unit)
static int  cycleNum = 0;
static unsigned long currentDelayData1 = (unsigned long)0;
static unsigned long currentDelayData2 = (unsigned long)0;
static unsigned long fullCycleCounts = 0;
#define ERR_COUNTERS_MAX                 8
static int  errCounters[ERR_COUNTERS_MAX];
//  for (int ix = 0; ix < ERR_COUNTERS_MAX ; ix++) {
//    errCounters[ix] = 0;
//  }
unsigned long sonarSampleTimes[USONAR_MAX_UNITS+1];
float sonarDistancesInMeters[USONAR_MAX_UNITS+1];
//  for (int ix = 0; ix < USONAR_MAX_UNITS; ix++) {
//    sonarDistancesInMeters[ix] = (float)(0.0);
//    sonarSampleTimes[ix] = (unsigned long)0;
//  }
static unsigned long sonarMeasTriggerTime = 0;
Sonar::Sonar(UART *debug_uart, RAB_Sonar *rab_sonar) {
  debug_uart_ = debug_uart;
  numSonars_ = USONAR_MAX_UNITS;
  numMeasSpecs_ = USONAR_MAX_SPECS;
  rab_sonar_ = rab_sonar;
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
  if ((specNumber < 0) || (specNumber > numMeasSpecs_)) {
    return -1;
  }
  return 1;
}

// Get the measurement spec number for a given sonar unit number
//
// Negative value indicates the sonar unit number was not found
int Sonar::unitNumToMeasSpecNum(int sonarUnit) {
  int specNumber = -1;

  for (int i = 0; i < numMeasSpecs_; i++) {
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
  if ((sonarUnit < 1) || (sonarUnit > numSonars_)) {
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
  } else if (usonar_measSpecs[specNumber].measMethod == US_MEAS_METHOD_T10_PJ7){
    PORTJ &= 0x7f;
    delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
    PORTJ |= 0x80;
    delayMicroseconds(USONAR_TRIG_HIGH_US);
    PORTJ &= 0x7f;
  } else if (usonar_measSpecs[specNumber].measMethod == US_MEAS_METHOD_T15_PG4){
    PORTG &= 0xef;
    delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
    PORTG |= 0x10;
    delayMicroseconds(USONAR_TRIG_HIGH_US);
    PORTG &= 0xef;
  } else if (usonar_measSpecs[specNumber].measMethod == US_MEAS_METHOD_T16_PG3){
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
float Sonar::getInlineReadMeters(int sonarUnit) {
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

// This is a way to make accessable using extern for backdoors to query read
// sensor distances.  We avoid using our class that may have encapsulated
// this outside of this module
int Sonar::getLastDistInMm(int sonarUnit) {
    if ((sonarUnit < 1) || (sonarUnit > USONAR_MAX_UNITS)) {
        return -10;
    }
    return (int)(sonarDistancesInMeters[sonarUnit] * (float)1000.0);
}

void Sonar::poll() {
      // -----------------------------------------------------------------
      // Deal with sampling the sonar ultrasonic range sensors
      //
      // We cycle through entries in the measurement spec table using cycleNum
      // Most calls accept measurement spec table entry number NOT sonar unit
      // number.
      switch (usonarSampleState) {
        case USONAR_STATE_MEAS_START: {
          int queueLevel = 0;
          queueLevel = getQueueLevel();  // We expect this to always be 0
          if ((queueLevel != 0) &&
	   (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG)) {
            debug_uart_->print((Text)" Sonar WARNING at meas cycle ");
            debug_uart_->integer_print(measSpecNumToUnitNum(cycleNum));
            debug_uart_->print((Text)" start: Queue had ");
            debug_uart_->integer_print(queueLevel);
            debug_uart_->print((Text)" edges! \r\n");
            flushQueue();
            currentDelayData1 = 0; // Reset sample gathering values for this run
            currentDelayData2 = 0;
          } else {
            // Indicate we are going to start next sonar measurement cycle
            if (rab_sonar_->system_debug_flags_get() &
	     DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar meas cycle ");
              debug_uart_->integer_print(measSpecNumToUnitNum(cycleNum));
              debug_uart_->print((Text)" starting.\r\n");
            }
          }

          cycleNum += 1;   // Bump to next entry in our measurement spec table 
          if (cycleNum > USONAR_MAX_SPECS) {
            cycleNum = 0;

            fullCycleCounts+= 1;
            if (((fullCycleCounts & (unsigned long)(0x7)) == 0) && 
             (rab_sonar_->system_debug_flags_get() & 
             DBG_FLAG_USENSOR_ERR_DEBUG)) {
              debug_uart_->print((Text)" Sonar ERR Counters: ");
              for (int ix = 0; ix < ERR_COUNTERS_MAX ; ix++) {
                debug_uart_->integer_print(errCounters[ix]);
                debug_uart_->print((Text)" ");
              }
              debug_uart_->print((Text)"\r\n");
            }

            if (rab_sonar_->system_debug_flags_get() &
              DBG_FLAG_USENSOR_RESULTS) {
               char  outBuf[32];
               float distInCm;
               debug_uart_->print((Text)" Sonars: ");
               for (int sonarUnit = 1;
		 sonarUnit <= USONAR_MAX_UNITS ; sonarUnit++) {
                  distInCm =
		   (float)(getLastDistInMm(sonarUnit))/(float)(10.0);
                  if (distInCm > USONAR_MAX_DIST_CM) {
                    distInCm = USONAR_MAX_DIST_CM;      // graceful hard cap
                  }
                  
                  dtostrf(distInCm, 3, 1, outBuf);
                  debug_uart_->string_print((Text)outBuf);
                  debug_uart_->string_print((Text)" ");
               }
               debug_uart_->string_print((Text)"\r\n");
            }
          }

          // Skip any unit that will not work with PinChange interrupt
          if ((getMeasSpec(cycleNum) & US_MEAS_METHOD_PCINT) == 0)  {
            // This unit will not work so just skip it and do next on on
	    // next pass
            break;
          }

          // Start the trigger for this sensor and enable interrupts
          #ifdef USONAR_ULTRA_FAST_ISR
	  // For ultra-fast we only gather 2 and reset queue each time
          usonar_producerIndex = 0;
          usonar_consumerIndex = 0;    
          #endif

          // Enable this units pin change interrupt then enable global
	  // ints for pin changes
          PCIFR = 0x06;		// This clears any pending pin change ints
          switch (getInterruptMaskRegNumber(cycleNum)) {
            case 1:
              PCMSK2 = 0;
              PCMSK1 = getInterruptBit(cycleNum);
              break;
            case 2:
              PCMSK1 = 0;
              PCMSK2 = getInterruptBit(cycleNum);
              break;
            default:
              // This is REALLY a huge coding issue/problem in usonar_measSpec
              if (rab_sonar_->system_debug_flags_get() &
	       DBG_FLAG_USENSOR_DEBUG) {
                debug_uart_->string_print(
		 (Text)" Sonar ERROR in code for intRegNum!\r\n");
              }
              break;
          } 
          SREG |= _BV(7);
         
          // Trigger the sonar unit to start measuring and return start time
          sonarMeasTriggerTime = measTrigger(cycleNum);

          if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            ltoa(sonarMeasTriggerTime, longStr,10);
            debug_uart_->string_print((Text)" Sonar start Sample: ");
            debug_uart_->integer_print(measSpecNumToUnitNum(cycleNum));
            debug_uart_->string_print((Text)" at ");
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us\r\n");
          }

          usonarSampleState = USONAR_STATE_WAIT_FOR_MEAS;
          }
          break;

        case USONAR_STATE_WAIT_FOR_MEAS: {
          // wait max time to ensure both edges get seen for realistic
	  // detection max
          //
          // If the sonar is blocked then these units can take about 200ms
	  // for edge pulse.  We will time out well before that and because
	  // we use a single edge detect the long sensor will be effectively
	  // ignored for several following measurements.
          unsigned long measCycleTime; // Time so far waiting for this meas.
          unsigned long rightNow;

          rightNow = USONAR_GET_MICROSECONDS;

          // Look for counter to go 'around' and ignore this one. 
          // Unsigned math so we can get HUGE number
          if (sonarMeasTriggerTime > rightNow) {
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print(
	       (Text)" Sonar system tic rollover in meas wait. \r\n");
            }
            usonarSampleState = USONAR_STATE_MEAS_START;
            break;
          }

          measCycleTime = rightNow - sonarMeasTriggerTime; 

          // If meas timer not done break on through till next pass
          if (measCycleTime < USONAR_MEAS_TIME)     {   
            break;     // Still waiting for measurement
          }


          // OK we think we have measurement data so check for and get edge data
          if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            debug_uart_->string_print((Text)" Sonar meas from: ");
            ltoa(sonarMeasTriggerTime, longStr,10);
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us to: ");
            ltoa(rightNow, longStr,10);
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us\r\n");
          }

          int edgeCount = getQueueLevel();
          if (edgeCount != 2) {
            // We expect exactly two edges.  If not we abort this meas cycle
            if (edgeCount == 0) {
              errCounters[0] += 1;       //  Most likely defective or broken/flakey wiring
            } else if (edgeCount ==1) {
              errCounters[1] += 1;       //  Most likely blocked sensor that waits 200ms
            } else {
              errCounters[2] += 1;       //  If this happens something is very wrong
            }
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar ");
              debug_uart_->integer_print((int)measSpecNumToUnitNum(cycleNum));
              debug_uart_->print((Text)" in cycle ");
              debug_uart_->integer_print(cycleNum);
              debug_uart_->print((Text)" ERROR! meas saw ");
              debug_uart_->integer_print(edgeCount);
              debug_uart_->print((Text)" edges! \r\n");
              // special value as error type indicator but as things mature we should NOT stuff this
              sonarDistancesInMeters[measSpecNumToUnitNum(cycleNum)] = 
                echoUsToMeters((USONAR_ECHO_MAX + USONAR_ECHO_ERR1));
            }

            usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle

            break;   // break to wait till next pass and do next sensor
          }

          // We clear the individual pin interrupt enable bits now
          //PCMSK1 = 0;
          //PCMSK2 = 0;

          // So lets (FINALLY) get the two edge samples
          unsigned long echoPulseWidth;    // Time between edges
          currentDelayData1 = pullQueueEntry();    // pull entry OR we get 0 if none yet
          currentDelayData2 = pullQueueEntry();    // pull entry OR we get 0 if none yet

          echoPulseWidth =  currentDelayData2 - currentDelayData1; // The 'real' meas data in uSec

          if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            debug_uart_->string_print((Text)" Sonar edges from: ");
            ltoa(currentDelayData1, longStr,10);
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us to: ");
            ltoa(currentDelayData2, longStr,10);
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us \r\n");
          }

          if (currentDelayData1 > currentDelayData2) {
            // This is another form of rollover every 70 minutes or so but just 
            errCounters[3] += 1;       // debug tally of errors
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar sys tic rollover OR edges reversed\r\n");
              // special value as error type indicator but as things mature we should NOT stuff this
              sonarDistancesInMeters[measSpecNumToUnitNum(cycleNum)] = 
                echoUsToMeters((unsigned long)USONAR_ECHO_MAX + (unsigned long)USONAR_ECHO_ERR2);
            }

            usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle

          } else if (echoPulseWidth > USONAR_MEAS_TOOLONG) {  
            errCounters[4] += 1;       // debug tally of errors
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar meas result over the MAX\r\n");
            }

          //  // special value as error type indicator but as things mature we should NOT stuff this
          //  sonarDistancesInMeters[measSpecNumToUnitNum(cycleNum)] = 
          //    usonar.echoUsToMeters((USONAR_ECHO_MAX + USONAR_ECHO_ERR3));
            usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle

          } else {
              // Save our sample delay AND save our time we acquired the sample
              if (echoPulseWidth > USONAR_ECHO_MAX) {
               // We are going to cap this as a form of non-expected result so can it
                errCounters[5] += 1;       // debug tally of errors
                if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
                  char longStr[32];
                  ltoa(echoPulseWidth, longStr,10);
                  debug_uart_->print((Text)" Sonar echo delay of ");
                  debug_uart_->print((Text)longStr);
                  debug_uart_->print((Text)" is over MAX!\r\n");
                }
                // We really should ignore this once system is robust
                // echoPulseWidth = (unsigned long)USONAR_ECHO_MAX + (unsigned long)USONAR_ECHO_ERR4;
                usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;   // move on to next sample cycle
                break;
              }

              // THIS IS THE REAL AND DESIRED PLACE WE EXPECT TO BE EACH TIME!
              sonarSampleTimes[measSpecNumToUnitNum(cycleNum)] = currentDelayData2;
              float distanceInMeters = echoUsToMeters(echoPulseWidth);
              sonarDistancesInMeters[measSpecNumToUnitNum(cycleNum)] = distanceInMeters;

              if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
                char outBuf2[32];
                float distInCm;
                int   echoCm;
                echoCm = echoPulseWidth / 58;
                distInCm = distanceInMeters * (float)(100.0);
                dtostrf(distInCm, 6, 1, outBuf2);
                debug_uart_->string_print((Text)" S: ");
                debug_uart_->integer_print((int)measSpecNumToUnitNum(cycleNum));
                debug_uart_->string_print((Text)" E: ");
                debug_uart_->integer_print((int)echoCm);
                debug_uart_->string_print((Text)"cm D: ");
                debug_uart_->string_print((Text)outBuf2);
                debug_uart_->string_print((Text)"cm \r\n");
              }
              }
              usonarSampleState = USONAR_STATE_POST_SAMPLE_WAIT;
            }
          break;

        case USONAR_STATE_POST_SAMPLE_WAIT: {
          // We have included a deadtime so we don't totaly hammer the ultrasound 
          // this will then not drive dogs 'too' crazy
          unsigned long waitTimer;
          unsigned long curTicks;
          curTicks = USONAR_GET_MICROSECONDS;

          currentDelayData1 = 0;      // Reset the sample gathering values for this run
          currentDelayData2 = 0;

          waitTimer = curTicks - sonarMeasTriggerTime; 

          if (sonarMeasTriggerTime > curTicks) {   // Unsigned math so we can get HUGE number
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar system timer rollover in meas spacing.\r\n");
            }
            usonarSampleState = USONAR_STATE_MEAS_START;
          } else if (waitTimer > USONAR_SAMPLES_US) {   
            usonarSampleState = USONAR_STATE_MEAS_START;
          }

          // If we fall through without state change we are still waiting
          }
          break;

        default:
              usonarSampleState = USONAR_STATE_MEAS_START;
        break;
      }
}