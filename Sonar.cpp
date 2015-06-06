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

// *Sonar* constructor and methods:

Sonar::Sonar(UByte unit_number, UByte interrupt_register_number,
 UByte interrupt_bit,
 volatile uint8_t *xtrigger_base, UByte xtrigger_mask,
 volatile uint8_t *xecho_base, UByte xecho_mask) {
  unitNumber = unit_number;
  intRegNum = interrupt_register_number;
  intBit = interrupt_bit;
  trigger_base = xtrigger_base;
  trigger_mask = xtrigger_mask;
  echo_base = xecho_base;
  echo_mask = xecho_mask;
}

// The measurement cycle number (does not have to be sonar unit)

Sonar_Controller::Sonar_Controller(UART *debug_uart, RAB_Sonar *rab_sonar,
 Sonar *sonars[]) {
  current_delay_data1_ = (unsigned long)0;
  current_delay_data2_ = (unsigned long)0;
  cycle_number_ = 0;
  debug_uart_ = debug_uart;
  full_cycle_counts_ = 0;
  number_measurement_specs_ = USONAR_MAX_SPECS;
  rab_sonar_ = rab_sonar;
  sonars_ = sonars;
  sample_state_ = USONAR_STATE_MEAS_START;
  measured_trigger_time_ = 0;

  UByte sonars_size = 0;
  while (*sonars++ != (Sonar *)0) {
    sonars_size += 1;
  }
  sonars_size_ = sonars_size;

  for (int ix = 0; ix < USONAR_MAX_UNITS; ix++) {
    sonar_distances_in_meters_[ix] = (float)(0.0);
    sonar_sample_times_[ix] = (unsigned long)0;
  }
  for (int ix = 0; ix < ERR_COUNTERS_MAX ; ix++) {
    error_counters_[ix] = 0;
  }
}

// Initialize the I/O pins used by the sonar.
void Sonar_Controller::ports_initialize() {
  for (UByte index = 0; index < sonars_size_; index++) {
    Sonar *sonar = sonars_[index];

    // Set the triggers to be output pins:
    volatile uint8_t *trigger_base = sonar->trigger_base;
    UByte trigger_mask = sonar->trigger_mask;
    trigger_base[PORT_OFFSET_] &= ~trigger_mask; // Clear output bit
    trigger_base[DDR_OFFSET_] |= trigger_mask;	 // Set pin to be an output

    // Set the echos to be input pins:
    volatile uint8_t *echo_base = sonar->echo_base;
    UByte echo_mask = sonar->echo_mask;
    echo_base[DDR_OFFSET_] &= ~echo_mask;	 // Set pin to be an input
  }
}

// Sonar echo completion Circular Queue interfaces for a background consumer

/// A generic circular queue utility to get number of entries in any circular
// queue.
int Sonar_Controller::calcQueueLevel(int Pidx, int Cidx, int queueSize) {
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
int Sonar_Controller::getQueueLevel() {
  int localPI = usonar_producerIndex;     // get atomic copy of producer index

  return calcQueueLevel(localPI, usonar_consumerIndex, USONAR_QUEUE_LEN);
}

// Pull one entry from our edge detection circular queue if there are entries.
// A return of 0 will happen if no entries are ready to pull at this time OR
// the entry is 0
unsigned long Sonar_Controller::pullQueueEntry() {
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
int Sonar_Controller::flushQueue() {
  int queueEntries = 0;
  while (pullQueueEntry() != 0) { queueEntries++; };  // Eat um all up, yum yum.
  return queueEntries;
}


// Helper to check range of measurement spec entries
// 
// Return value of 1 indicates valid meas spec entry number
// Negative value indicates unsupported spec table Entry
int Sonar_Controller::isMeasSpecNumValid(int specNumber) {
  if ((specNumber < 0) || (specNumber > number_measurement_specs_)) {
    return -1;
  }
  return 1;
}

// Get the measurement spec number for a given sonar unit number
//
// Negative value indicates the sonar unit number was not found
int Sonar_Controller::unitNumToMeasSpecNum(int sonarUnit) {
  int specNumber = -1;

  for (int i = 0; i < number_measurement_specs_; i++) {
    if (sonars_[i]->unitNumber == sonarUnit) {
      specNumber = i;
    }
  }

  return specNumber;
}

// Get the sonar unit number for a given spec table entry
//
// Negative value indicates unsupported spec table Entry
int Sonar_Controller::measSpecNumToUnitNum(int specNumber) {
  int sonarUnit = 0;

  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }

  sonarUnit = sonars_[specNumber]->unitNumber;

  return sonarUnit;
}

// Get the Pin change Interrupt Bank number for a spec table entry
//
// Negative value indicates unsupported spec table Entry
int Sonar_Controller::getInterruptMaskRegNumber(int specNumber) {
  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }
  return sonars_[specNumber]->intRegNum;
}

// Get the interrupt enable bit for the given spec table entry
//
// Negative value indicates unsupported spec table Entry
int Sonar_Controller::getInterruptBit(int specNumber) {
  if (isMeasSpecNumValid(specNumber) < 0) {
    return -1;
  }
  return sonars_[specNumber]->intBit;
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

unsigned long Sonar_Controller::measTrigger(int specNumber) {

  if (isMeasSpecNumValid(specNumber) < 0) {
    return 0;
  }

  Sonar *sonar = sonars_[specNumber];
  volatile uint8_t *trigger_base = sonar->trigger_base;
  UByte trigger_mask = sonar->trigger_mask;
  trigger_base[PORT_OFFSET_] &= ~trigger_mask;
  delayMicroseconds(USONAR_TRIG_PRE_LOW_US);
  trigger_base[PORT_OFFSET_] |= trigger_mask;
  delayMicroseconds(USONAR_TRIG_HIGH_US);
  trigger_base[PORT_OFFSET_] &= ~trigger_mask;

  return USONAR_GET_MICROSECONDS | 1;
}


float Sonar_Controller::echoUsToMeters(unsigned long pingDelay) {
  float  meters;
  meters = (float)(pingDelay) / USONAR_US_TO_METERS;
  return meters;
}

// This is a way to make accessable using extern for backdoors to query read
// sensor distances.  We avoid using our class that may have encapsulated
// this outside of this module
int Sonar_Controller::getLastDistInMm(int sonarUnit) {
    if ((sonarUnit < 1) || (sonarUnit > USONAR_MAX_UNITS)) {
        return -10;
    }
    return (int)(sonar_distances_in_meters_[sonarUnit] * (float)1000.0);
}

void Sonar_Controller::poll() {
      // -----------------------------------------------------------------
      // Deal with sampling the sonar ultrasonic range sensors
      //
      // We cycle through entries in the measurement spec table using cycleNum
      // Most calls accept measurement spec table entry number NOT sonar unit
      // number.
      switch (sample_state_) {
        case USONAR_STATE_MEAS_START: {
          int queueLevel = 0;
          queueLevel = getQueueLevel();  // We expect this to always be 0
          if ((queueLevel != 0) &&
	   (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG)) {
            debug_uart_->print((Text)" Sonar WARNING at meas cycle ");
            debug_uart_->integer_print(measSpecNumToUnitNum(cycle_number_));
            debug_uart_->print((Text)" start: Queue had ");
            debug_uart_->integer_print(queueLevel);
            debug_uart_->print((Text)" edges! \r\n");
            flushQueue();
            current_delay_data1_ = 0; // Reset sample gathering values for this run
            current_delay_data2_ = 0;
          } else {
            // Indicate we are going to start next sonar measurement cycle
            if (rab_sonar_->system_debug_flags_get() &
	     DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar meas cycle ");
              debug_uart_->integer_print(measSpecNumToUnitNum(cycle_number_));
              debug_uart_->print((Text)" starting.\r\n");
            }
          }

	  // Bump to next entry in our measurement spec table 
          cycle_number_ += 1;
          if (cycle_number_ > USONAR_MAX_SPECS) {
            cycle_number_ = 0;

            full_cycle_counts_ += 1;
            if (((full_cycle_counts_ & (unsigned long)(0x7)) == 0) && 
             (rab_sonar_->system_debug_flags_get() & 
             DBG_FLAG_USENSOR_ERR_DEBUG)) {
              debug_uart_->print((Text)" Sonar ERR Counters: ");
              for (int ix = 0; ix < ERR_COUNTERS_MAX ; ix++) {
                debug_uart_->integer_print(error_counters_[ix]);
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
          //if ((getMeasSpec(cycle_number_) & US_MEAS_METHOD_PCINT) == 0)  {
          //  // This unit will not work so just skip it and do next on on
	  //  // next pass
          //  break;
          //}

          // Start the trigger for this sensor and enable interrupts
          #ifdef USONAR_ULTRA_FAST_ISR
	  // For ultra-fast we only gather 2 and reset queue each time
          usonar_producerIndex = 0;
          usonar_consumerIndex = 0;    
          #endif

          // Enable this units pin change interrupt then enable global
	  // ints for pin changes
          PCIFR = 0x06;		// This clears any pending pin change ints
          switch (getInterruptMaskRegNumber(cycle_number_)) {
            case 1:
              PCMSK2 = 0;
              PCMSK1 = getInterruptBit(cycle_number_);
              break;
            case 2:
              PCMSK1 = 0;
              PCMSK2 = getInterruptBit(cycle_number_);
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
          measured_trigger_time_ = measTrigger(cycle_number_);

          if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            ltoa(measured_trigger_time_, longStr,10);
            debug_uart_->string_print((Text)" Sonar start Sample: ");
            debug_uart_->integer_print(measSpecNumToUnitNum(cycle_number_));
            debug_uart_->string_print((Text)" at ");
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us\r\n");
          }

          sample_state_ = USONAR_STATE_WAIT_FOR_MEAS;
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
          if (measured_trigger_time_ > rightNow) {
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print(
	       (Text)" Sonar system tic rollover in meas wait. \r\n");
            }
            sample_state_ = USONAR_STATE_MEAS_START;
            break;
          }

          measCycleTime = rightNow - measured_trigger_time_; 

          // If meas timer not done break on through till next pass
          if (measCycleTime < USONAR_MEAS_TIME)     {   
            break;     // Still waiting for measurement
          }


          // OK we think we have measurement data so check for and get edge data
          if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            debug_uart_->string_print((Text)" Sonar meas from: ");
            ltoa(measured_trigger_time_, longStr,10);
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
	      //  Most likely defective or broken/flakey wiring
              error_counters_[0] += 1;
            } else if (edgeCount ==1) {
              //  Most likely blocked sensor that waits 200ms
              error_counters_[1] += 1;
            } else {
	      //  If this happens something is very wrong
              error_counters_[2] += 1;
            }
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar ");
              debug_uart_->integer_print(
		(int)measSpecNumToUnitNum(cycle_number_));
              debug_uart_->print((Text)" in cycle ");
              debug_uart_->integer_print(cycle_number_);
              debug_uart_->print((Text)" ERROR! meas saw ");
              debug_uart_->integer_print(edgeCount);
              debug_uart_->print((Text)" edges! \r\n");
              // special value as error type indicator but as things
	      // mature we should NOT stuff this
              sonar_distances_in_meters_[measSpecNumToUnitNum(cycle_number_)] = 
                echoUsToMeters((USONAR_ECHO_MAX + USONAR_ECHO_ERR1));
            }

	    // move on to next sample cycle
            sample_state_ = USONAR_STATE_POST_SAMPLE_WAIT;

            break;   // break to wait till next pass and do next sensor
          }

          // We clear the individual pin interrupt enable bits now
          //PCMSK1 = 0;
          //PCMSK2 = 0;

          // So lets (FINALLY) get the two edge samples
          unsigned long echoPulseWidth;    // Time between edges
	  // pull entry OR we get 0 if none yet
          current_delay_data1_ = pullQueueEntry();
	  // pull entry OR we get 0 if none yet
          current_delay_data2_ = pullQueueEntry();

	  // The 'real' meas data in uSec
          echoPulseWidth =  current_delay_data2_ - current_delay_data1_;

          if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
            char longStr[32];
            debug_uart_->string_print((Text)" Sonar edges from: ");
            ltoa(current_delay_data1_, longStr,10);
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us to: ");
            ltoa(current_delay_data2_, longStr,10);
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us \r\n");
          }

          if (current_delay_data1_ > current_delay_data2_) {
            // This is another form of rollover every 70 minutes or
	    // so but just 
            error_counters_[3] += 1;       // debug tally of errors
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print(
	        (Text)" Sonar sys tic rollover OR edges reversed\r\n");
              // special value as error type indicator but as things mature
	      // we should NOT stuff this
              sonar_distances_in_meters_[measSpecNumToUnitNum(cycle_number_)] = 
                echoUsToMeters((unsigned long)USONAR_ECHO_MAX +
		(unsigned long)USONAR_ECHO_ERR2);
            }

	    // move on to next sample cycle
            sample_state_ = USONAR_STATE_POST_SAMPLE_WAIT;

          } else if (echoPulseWidth > USONAR_MEAS_TOOLONG) {  
            error_counters_[4] += 1;       // debug tally of errors
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print((Text)" Sonar meas result over the MAX\r\n");
            }

            // special value as error type indicator but as things mature we
	    // should NOT stuff this 
            // sonar_distances_in_meters_[measSpecNumToUnitNum(cycle_number_)] =
            //    usonar.echoUsToMeters((USONAR_ECHO_MAX + USONAR_ECHO_ERR3));
	    // move on to next sample cycle
            sample_state_ = USONAR_STATE_POST_SAMPLE_WAIT;
          } else {
              // Save our sample delay AND save our time we acquired the sample
              if (echoPulseWidth > USONAR_ECHO_MAX) {
               // Cap this as a form of non-expected result so can it
                error_counters_[5] += 1;       // debug tally of errors
                if (rab_sonar_->system_debug_flags_get() &
		 DBG_FLAG_USENSOR_DEBUG) {
                  char longStr[32];
                  ltoa(echoPulseWidth, longStr,10);
                  debug_uart_->print((Text)" Sonar echo delay of ");
                  debug_uart_->print((Text)longStr);
                  debug_uart_->print((Text)" is over MAX!\r\n");
                }

                // We really should ignore this once system is robust
                // echoPulseWidth = (unsigned long)USONAR_ECHO_MAX +
		//  (unsigned long)USONAR_ECHO_ERR4;
	        // move on to next sample cycle
                sample_state_ = USONAR_STATE_POST_SAMPLE_WAIT;
                break;
              }

              // THIS IS THE REAL AND DESIRED PLACE WE EXPECT TO BE EACH TIME!
              sonar_sample_times_[measSpecNumToUnitNum(cycle_number_)] =
	       current_delay_data2_;
              float distanceInMeters = echoUsToMeters(echoPulseWidth);
              sonar_distances_in_meters_[measSpecNumToUnitNum(cycle_number_)] =
	       distanceInMeters;

              if (rab_sonar_->system_debug_flags_get() &
	       DBG_FLAG_USENSOR_DEBUG) {
                char outBuf2[32];
                float distInCm;
                int   echoCm;
                echoCm = echoPulseWidth / 58;
                distInCm = distanceInMeters * (float)(100.0);
                dtostrf(distInCm, 6, 1, outBuf2);
                debug_uart_->string_print((Text)" S: ");
                debug_uart_->integer_print(
		 (int)measSpecNumToUnitNum(cycle_number_));
                debug_uart_->string_print((Text)" E: ");
                debug_uart_->integer_print((int)echoCm);
                debug_uart_->string_print((Text)"cm D: ");
                debug_uart_->string_print((Text)outBuf2);
                debug_uart_->string_print((Text)"cm \r\n");
              }
              }
              sample_state_ = USONAR_STATE_POST_SAMPLE_WAIT;
            }
          break;

        case USONAR_STATE_POST_SAMPLE_WAIT: {
          // We have included a deadtime so we don't totaly hammer the
	  // ultrasound this will then not drive dogs 'too' crazy
          unsigned long waitTimer;
          unsigned long curTicks;
          curTicks = USONAR_GET_MICROSECONDS;

	  // Reset sample gathering values for this run
          current_delay_data1_ = 0;
          current_delay_data2_ = 0;

          waitTimer = curTicks - measured_trigger_time_; 

          if (measured_trigger_time_ > curTicks) {
	    // Unsigned math so we can get HUGE number
            if (rab_sonar_->system_debug_flags_get() & DBG_FLAG_USENSOR_DEBUG) {
              debug_uart_->print(
	       (Text)" Sonar system timer rollover in meas spacing.\r\n");
            }
            sample_state_ = USONAR_STATE_MEAS_START;
          } else if (waitTimer > USONAR_SAMPLES_US) {   
            sample_state_ = USONAR_STATE_MEAS_START;
          }

          // If we fall through without state change we are still waiting
          }
          break;

        default:
              sample_state_ = USONAR_STATE_MEAS_START;
        break;
      }
}
