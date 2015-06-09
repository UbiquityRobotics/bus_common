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

// ***************************************************************************
// UltraSonic Sonar Code
//
// One part of the code must be extremely fast code used in ISRs to log edge
// detections
//
// A second part of the code is used in background to do round robin sonar
// scans and keep track of latest good measurements of distance so they can
// be fetched by interested subsystems.
//
// Note: The background code could be moved into a separate header and file
// for the class.
//
// Some in-memory history of last sample times and readings in meters
// These tables are organized by sonar number and not by measurement cycle
// number.  Also the index is the sonar number so entry [0] is zero

#include "Arduino.h"
#include "Sonar.h"

// *Sonar_Queue* constructor and methods:

Sonar_Queue::Sonar_Queue(UByte mask_index, volatile uint8_t *io_port_base) {
  mask_index_ = mask_index;
  io_port_base_ = io_port_base;
  producer_index_ = 0;
  consumer_index_ = 0;
}

void Sonar_Queue::interrupt_service_routine() {
  // Grab the latest input port value:
  UByte change = io_port_base_[PIN_INDEX_];

  // The act of reading *TCNT1L* causes *TCNT1H* to be cached into
  // a temporary register.  Thus, by reading *TCNT1L* before *TCNT1H*,
  // we git a consistent 16-bit value without having to do silliness
  // disabling interrupts and so forth...
  UByte low_tick = TCNT1L;
  UByte high_tick = TCNT1H;

  // Stash the values away:
  changes_[producer_index_] = change;
  ticks_[producer_index_] = (((UShort)high_tick) << 8) | ((UShort)low_tick);

  // Increment the *producer_index_*:
  producer_index_ = (producer_index_ + 1) & QUEUE_MASK_;
}

// *Sonar* constructor and methods:

Sonar::Sonar(volatile uint8_t *trigger_base, UByte trigger_bit,
 Sonar_Queue *sonar_queue, UByte change_bit,
 volatile uint8_t *echo_base, UByte echo_bit) {
  change_mask_ = (1 << change_bit);
  distance_in_meters = (float)0.0;
  echo_base_ = echo_base;
  echo_mask_ = (1 << echo_bit);
  sonar_queue_ = sonar_queue;
  trigger_base_ = trigger_base;
  trigger_mask_ = (1 << trigger_bit);
}

// Trigger a single sonar:
void Sonar::measurement_trigger() {
  trigger_base_[PORT_OFFSET_] &= ~trigger_mask_;
  delayMicroseconds(TRIG_PRE_LOW_US_);
  trigger_base_[PORT_OFFSET_] |= trigger_mask_;
  delayMicroseconds(TRIG_HIGH_US_);
  trigger_base_[PORT_OFFSET_] &= ~trigger_mask_;
}

// Initialize the appropriate I/O pins for a *Sonar*:
void Sonar::initialize() {
    // Set the triggers to be output pins:
    trigger_base_[PORT_OFFSET_] &= ~trigger_mask_; // Clear output bit
    trigger_base_[DDR_OFFSET_] |= trigger_mask_;   // Set pin to be an output

    // Set the echos to be input pins:
    echo_base_[DDR_OFFSET_] &= ~echo_mask_;	   // Set pin to be an input
}

// *Sonar_Controller* constructor:

Sonar_Controller::Sonar_Controller(UART *debug_uart,
 Sonar *sonars[], Sonar_Queue *sonar_queues[]) {
  // Initialize various member variables:
  consumer_index_ = 0;
  current_delay_data1_ = (unsigned long)0;
  current_delay_data2_ = (unsigned long)0;
  cycle_number_ = 0;
  debug_flags_ = 0;
  debug_uart_ = debug_uart;
  full_cycle_counts_ = 0;
  measured_trigger_time_ = 0;
  producer_index_ = 0;
  sample_state_ = STATE_MEAS_START_;
  sonars_ = sonars;
  sonar_queues_ = sonar_queues;
  general_debug_flag_ = 0;
  error_debug_flag_ = 0;
  results_debug_flag_ = 0;

  // Count up the number of sonars:
  UByte sonars_size = 0;
  while (*sonars++ != (Sonar *)0) {
    sonars_size += 1;
  }
  sonars_size_ = sonars_size;
  number_measurement_specs_ = sonars_size;

  // Zero out error counters;
  for (UByte index = 0; index < ERROR_COUNTERS_SIZE_ ; index++) {
    error_counters_[index] = 0;
  }
}

// *Sonar_Controller* static variables and method(s):

// All the interrupt code is done as static member variables and
// static member functions.

// Define the circular queue members and indexes.
// Remember! this is highly optimized so no fancy classes used here folks!
//
// Queue Rules:
//  - The index values increment for each entry and roll back to 0 when equal
//    to USONAR_QUEUE_LEN
//    It is possible to have consumer index near end and producer near start
//    index of 0 which means
//    the entries are rolling around but are still all valid.  Producers and
//    consumer MUST deal with this.
//  - Producer may only put in entries if 2 or more spaces are open
//  - Producer must drop entries if queue is full
//  - Producer places data at next index past current producer index in both
//    arrays and THEN bumps producer index to point to that index when values
//    are intact.
//  - Consumer may only read entries up to and including current producer index.
//  - Consumer may only bump consumer index to one more than what has just
//    been processed.  Consumer must NEVER try to read the values once
//    consumer has bumped consumer index past
//
// Queue Member Descriptions
//  echo_edge_queue_[]        ISR fed queue to hold edge change detections
//                            produced in fast ISR.
//                            Entires are 30 bit timestamp with lower 2
//                            bits showing the bank that changed
//                            Note that the timestamp will roll over in
//                            around 70 min 
//  echo_changed_pins_[]      Not used yet!  Is to hold a 1 for each pin
//                            that has changed at this timestamp.
//                            Bits 23:0 are to be placed properly by each ISR
//
// The Echo timestamp is saved with the least sig 2 bits being set to the
// channel for this timestamp.  We do not know which bits changed, we only
// know at least one changed in this bank.  We also have a word of info to
//  hold other info we may want the ISR to return.  So this means the sonar
// cycling needs to be careful to only do one sample at a time from any one
// bank.

unsigned int Sonar_Controller::consumer_index_ = 0;
unsigned int Sonar_Controller::producer_index_ = 0;
unsigned long Sonar_Controller::echo_edge_queue_[QUEUE_SIZE_];
unsigned long Sonar_Controller::echo_info_queue_[QUEUE_SIZE_];

// We found that the nice producer consumer queue has to be reset down wo
// just do one meas per loop and reset queue each time so we get 2 edges

// Set *USONAR_ULTRA_FAST_ISR* to disable error checking:
#define  USONAR_ULTRA_FAST_ISR

// This ISR must be LIGHTNING FAST and is bare bones petal-to-the-metal
// processing!
//
// Pidx = Cidx when queue is empty.  Consumer sees this as empy. Pi can
// stuff away
// Pidx = Index that will next be filled by producer IF there was space
// Cidx = Index that will be read next time around IF Pidx != Cidx
//
// Pin Change Processing:
// This ISR will take in which pins had changes on one bank of pins
// and then push those changes with an associated timestamp in echoQueue[]
// The echoQueue[] is a circular queue so if the background consumer task
// has left the queue too full this ISR will loose data.
void Sonar_Controller::interrupt_handler(UByte flags) {
  unsigned long now = micros(); // Get clock FAST as we can

#ifndef USONAR_ULTRA_FAST_ISR
  int unused = 0;
  int inuse  = 0;
  // Ensure there is room in the queue even if we have rollover
  if (usonar_consumerIndex <= usonar_producerIndex) {
    inuse = usonar_producerIndex - usonar_consumerIndex;
  } else {
    inuse =
     (USONAR_QUEUE_LEN - usonar_consumerIndex) + usonar_producerIndex + 1;
  }
  unused = USONAR_QUEUE_LEN - inuse;

  if (unused >= USONAR_MIN_EMPTIES) {
#endif

    // We can produce this entry into the queue and bump producer index
    unsigned int nextProducerIndex = producer_index_ + 1;
    if (nextProducerIndex >= QUEUE_SIZE_) 
      nextProducerIndex = 0;

    // Since timer resolution is about 8us and this clock is usec we will
    // use the LOWER 2 bits to indicate which bank this change is from
    echo_edge_queue_[producer_index_] = (now & 0xfffffffc) | flags;
    // Bump producer index to this valid one
    producer_index_ = nextProducerIndex;

#ifndef USONAR_ULTRA_FAST_ISR
  } else {
    usonar_queueOverflow  += 1;
  }
#endif
}

// *Sonar_Controller* methods:

// Set the *debug_flags_* variable:

UByte Sonar_Controller::change_mask_get(UByte sonar_index) {
  Sonar *sonar = sonars_[sonar_index];
  UByte change_mask = sonar->change_mask_get();
  return change_mask;
}

void Sonar_Controller::debug_flags_set(UShort debug_flags) {
  debug_flags_ = debug_flags;
}

void Sonar_Controller::debug_flag_values_set(UShort error_debug_flag,
 UShort general_debug_flag, UShort results_debug_flag) {
  error_debug_flag_ = error_debug_flag;
  general_debug_flag_ = general_debug_flag;
  results_debug_flag_ = results_debug_flag;
}

UByte Sonar_Controller::echo_mask_get(UByte sonar_index) {
  Sonar *sonar = sonars_[sonar_index];
  UByte echo_mask = sonar->echo_mask_get();
  return echo_mask;
}

UByte Sonar_Controller::mask_index_get(UByte sonar_index) {
  Sonar *sonar = sonars_[sonar_index];
  Sonar_Queue *sonar_queue = sonar->sonar_queue_get();
  UByte mask_index = sonar_queue->mask_index_get();
  return mask_index;
}

// Initialize the I/O port for the sonar:

// Trigger the given ultrasonic sonar in the given spec table entry
// The sonar unit number is given to this routine as written on PC board
// This routine shields the caller from knowing the hardware pin
//
// Return value is system clock in microseconds for when the trigger was sent
// Note that the sonar itself will not reply for up to a few hundred
// microseconds

unsigned long Sonar_Controller::measurement_trigger(UByte sonar_index) {
  // Make sure we access a valid *sonar*:
  if (sonar_index >= sonars_size_) {
    sonar_index = 0;
  }
  Sonar *sonar = sonars_[sonar_index];

  // Trigger the measurement:
  sonar->measurement_trigger();

  return micros() | 1;
}

// Initialize the I/O pins used by the sonar controller:
void Sonar_Controller::initialize() {
  // Initialize each *sonar* and compute *pin_change_interrrupts_mask_*:
  pin_change_interrupts_mask_ = 0;
  for (UByte index = 0; index < sonars_size_; index++) {
    Sonar *sonar = sonars_[index];
    sonar->initialize();
    Sonar_Queue *sonar_queue = sonar->sonar_queue_get();
    pin_change_interrupts_mask_ |= (1 << sonar_queue->mask_index_get());
  }

  // Figure out how many *Sonar_Queues* we have:
  sonar_queues_size_ = 0;
  while (sonar_queues_[sonar_queues_size_] != (Sonar_Queue *)0) {
    sonar_queues_size_++;
    //debug_uart_->string_print((Text)"+");
  }

  // Enable the pin change interrupt registers:
  PCICR |= pin_change_interrupts_mask_;
}

void Sonar_Controller::poll() {
      // -----------------------------------------------------------------
      // Deal with sampling the sonar ultrasonic range sensors
      //
      // We cycle through entries in the measurement spec table using cycleNum
      // Most calls accept measurement spec table entry number NOT sonar unit
      // number.
      switch (sample_state_) {
        case STATE_MEAS_START_: {
          int queueLevel = 0;
	  // We expect this to always be 0
          queueLevel = getQueueLevel();
          if ((queueLevel != 0) && (debug_flags_ & general_debug_flag_)) {
            debug_uart_->print((Text)" Sonar WARNING at meas cycle ");
            debug_uart_->integer_print(cycle_number_);
            debug_uart_->print((Text)" start: Queue had ");
            debug_uart_->integer_print(queueLevel);
            debug_uart_->print((Text)" edges! \r\n");
            flushQueue();
	    // Reset sample gathering values for this run
            current_delay_data1_ = 0;
            current_delay_data2_ = 0;
          } else {
            // Indicate we are going to start next sonar measurement cycle
            if (debug_flags_ & general_debug_flag_) {
              debug_uart_->print((Text)" Sonar meas cycle ");
              debug_uart_->integer_print(cycle_number_);
              debug_uart_->print((Text)" starting.\r\n");
            }
          }

	  // Bump to next entry in our measurement spec table 
          cycle_number_ += 1;
          if (cycle_number_ > sonars_size_) {
            cycle_number_ = 0;

            full_cycle_counts_ += 1;
            if (((full_cycle_counts_ & (unsigned long)(0x7)) == 0) && 
             (debug_flags_ & error_debug_flag_)) {
              debug_uart_->print((Text)" Sonar ERR Counters: ");
              for (int ix = 0; ix < ERROR_COUNTERS_SIZE_ ; ix++) {
                debug_uart_->integer_print(error_counters_[ix]);
                debug_uart_->print((Text)" ");
              }
              debug_uart_->print((Text)"\r\n");
            }

            if (debug_flags_ & results_debug_flag_) {
               char  outBuf[32];
               debug_uart_->print((Text)" Sonars: ");
	       for (UByte index = 0; index < sonars_size_; index++) {
		 UShort mm_distance = mm_distance_get(index);
                 dtostrf(mm_distance, 3, 1, outBuf);
                 debug_uart_->string_print((Text)outBuf);
                 debug_uart_->string_print((Text)" ");
               }
               debug_uart_->string_print((Text)"\r\n");
            }
          }

          // Start the trigger for this sensor and enable interrupts
          #ifdef USONAR_ULTRA_FAST_ISR
	  // For ultra-fast we only gather 2 and reset queue each time
          producer_index_ = 0;
          consumer_index_ = 0;    
          #endif

          // Enable this units pin change interrupt then enable global
	  // ints for pin changes
          //PCIFR = 0x06;	// This clears any pending pin change ints

	  // This clears any pending pin change interurrps:
          PCIFR = pin_change_interrupts_mask_;

	  // Zero out the PCMSKn registers:
	  if (pin_change_interrupts_mask_ & 1) {
	    PCMSK0 = 0;
	  }
	  if (pin_change_interrupts_mask_ & 2) {
	    PCMSK1 = 0;
	  }
          if (pin_change_interrupts_mask_ & 4) {
	    PCMSK2 = 0;
	  }

	  // Now set the appropriate pin change in the appropriate PCMSKn
	  // register:
	  UByte mask_index = mask_index_get(cycle_number_);
	  UByte change_mask = change_mask_get(cycle_number_);
	  volatile uint8_t *masks_base = &PCMSK0;
	  masks_base[mask_index] = change_mask;

          //switch (getInterruptMaskRegNumber(cycle_number_)) {
          //  case 1:
          //    PCMSK2 = 0;
          //    PCMSK1 = getInterruptBit(cycle_number_);
          //    break;
          //  case 2:
          //    PCMSK1 = 0;
          //    PCMSK2 = getInterruptBit(cycle_number_);
          //    break;
          //  default:
          //    // This is REALLY a huge coding issue/problem in usonar_measSpec
          //    if (debug_flags_ & general_debug_flag_) {
          //      debug_uart_->string_print(
	  //	 (Text)" Sonar ERROR in code for intRegNum!\r\n");
          //    }
          //    break;
          //} 

	  // Enable global interrupts:
          SREG |= _BV(7);
         
          // Trigger the sonar unit to start measuring and return start time
          measured_trigger_time_ = measurement_trigger(cycle_number_);

          if (debug_flags_ & general_debug_flag_) {
            char longStr[32];
            ltoa(measured_trigger_time_, longStr,10);
            debug_uart_->string_print((Text)" Sonar start Sample: ");
            debug_uart_->integer_print(cycle_number_);
            debug_uart_->string_print((Text)" at ");
            debug_uart_->string_print((Text)longStr);
            debug_uart_->string_print((Text)"us\r\n");
          }

          sample_state_ = STATE_WAIT_FOR_MEAS_;
          }
          break;

        case STATE_WAIT_FOR_MEAS_: {
          // wait max time to ensure both edges get seen for realistic
	  // detection max
          //
          // If the sonar is blocked then these units can take about 200ms
	  // for edge pulse.  We will time out well before that and because
	  // we use a single edge detect the long sensor will be effectively
	  // ignored for several following measurements.
          unsigned long measCycleTime; // Time so far waiting for this meas.
          unsigned long rightNow;

          rightNow = micros();

          // Look for counter to go 'around' and ignore this one. 
          // Unsigned math so we can get HUGE number
          if (measured_trigger_time_ > rightNow) {
            if (debug_flags_ & general_debug_flag_) {
              debug_uart_->print(
	       (Text)" Sonar system tic rollover in meas wait. \r\n");
            }
            sample_state_ = STATE_MEAS_START_;
            break;
          }

          measCycleTime = rightNow - measured_trigger_time_; 

          // If meas timer not done break on through till next pass
          if (measCycleTime < MEASURE_TIME_)     {   
            break;     // Still waiting for measurement
          }


          // OK we think we have measurement data so check for and get edge data
          if (debug_flags_ & general_debug_flag_) {
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
            if (debug_flags_ & general_debug_flag_) {
              debug_uart_->print((Text)" Sonar ");
              debug_uart_->integer_print(cycle_number_);
              debug_uart_->print((Text)" in cycle ");
              debug_uart_->integer_print(cycle_number_);
              debug_uart_->print((Text)" ERROR! meas saw ");
              debug_uart_->integer_print(edgeCount);
              debug_uart_->print((Text)" edges! \r\n");
              // special value as error type indicator but as things
	      // mature we should NOT stuff this
	      UByte sonar_index = cycle_number_;
	      sonars_[sonar_index]->distance_in_meters =
                echoUsToMeters((ECHO_MAXIMUM_ + ECHO_ERROR1_));
            }

	    // move on to next sample cycle
            sample_state_ = STATE_POST_SAMPLE_WAIT_;

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

          if (debug_flags_ & general_debug_flag_) {
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
            if (debug_flags_ & general_debug_flag_) {
              debug_uart_->print(
	        (Text)" Sonar sys tic rollover OR edges reversed\r\n");
              // special value as error type indicator but as things mature
	      // we should NOT stuff this
	      UByte sonar_index = cycle_number_;
              sonars_[sonar_index]->distance_in_meters = 
                echoUsToMeters((unsigned long)ECHO_MAXIMUM_ +
		(unsigned long)ECHO_ERROR2_);
            }

	    // move on to next sample cycle
            sample_state_ = STATE_POST_SAMPLE_WAIT_;

          } else if (echoPulseWidth > MEASURE_TOO_LONG_) {  
            error_counters_[4] += 1;       // debug tally of errors
            if (debug_flags_ & general_debug_flag_) {
              debug_uart_->print((Text)" Sonar meas result over the MAX\r\n");
            }

            // special value as error type indicator but as things mature we
	    // should NOT stuff this 
            // sonar_distances_in_meters_[measSpecNumToUnitNum(cycle_number_)] =
            //    usonar.echoUsToMeters((USONAR_ECHO_MAX + USONAR_ECHO_ERR3));
	    // move on to next sample cycle
            sample_state_ = STATE_POST_SAMPLE_WAIT_;
          } else {
              // Save our sample delay AND save our time we acquired the sample
              if (echoPulseWidth > ECHO_MAXIMUM_) {
               // Cap this as a form of non-expected result so can it
                error_counters_[5] += 1;       // debug tally of errors
                if (debug_flags_ & general_debug_flag_) {
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
                sample_state_ = STATE_POST_SAMPLE_WAIT_;
                break;
              }

              // THIS IS THE REAL AND DESIRED PLACE WE EXPECT TO BE EACH TIME!
	      UByte sonar_index = cycle_number_;
              sonars_[sonar_index]->sample_time = current_delay_data2_;
              float distanceInMeters = echoUsToMeters(echoPulseWidth);
              sonars_[sonar_index]->distance_in_meters = distanceInMeters;

              if (debug_flags_ & general_debug_flag_) {
                char outBuf2[32];
                float distInCm;
                int   echoCm;
                echoCm = echoPulseWidth / 58;
                distInCm = distanceInMeters * (float)(100.0);
                dtostrf(distInCm, 6, 1, outBuf2);
                debug_uart_->string_print((Text)" S: ");
                debug_uart_->integer_print(cycle_number_);
                debug_uart_->string_print((Text)" E: ");
                debug_uart_->integer_print((int)echoCm);
                debug_uart_->string_print((Text)"cm D: ");
                debug_uart_->string_print((Text)outBuf2);
                debug_uart_->string_print((Text)"cm \r\n");
              }
              }
              sample_state_ = STATE_POST_SAMPLE_WAIT_;
            }
          break;

        case STATE_POST_SAMPLE_WAIT_: {
          // We have included a deadtime so we don't totaly hammer the
	  // ultrasound this will then not drive dogs 'too' crazy
          unsigned long waitTimer;
          unsigned long curTicks;
          curTicks = micros();

	  // Reset sample gathering values for this run
          current_delay_data1_ = 0;
          current_delay_data2_ = 0;

          waitTimer = curTicks - measured_trigger_time_; 

          if (measured_trigger_time_ > curTicks) {
	    // Unsigned math so we can get HUGE number
            if (debug_flags_ & general_debug_flag_) {
              debug_uart_->print(
	       (Text)" Sonar system timer rollover in meas spacing.\r\n");
            }
            sample_state_ = STATE_MEAS_START_;
          } else if (waitTimer > SAMPLES_US_) {   
            sample_state_ = STATE_MEAS_START_;
          }

          // If we fall through without state change we are still waiting
          }
          break;

        default:
              sample_state_ = STATE_MEAS_START_;
        break;
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
  int localPI = producer_index_;     // get atomic copy of producer index

  return calcQueueLevel(localPI, consumer_index_, QUEUE_SIZE_);
}

// Pull one entry from our edge detection circular queue if there are entries.
// A return of 0 will happen if no entries are ready to pull at this time OR
// the entry is 0
unsigned long Sonar_Controller::pullQueueEntry() {
  int localPI = producer_index_;     // get atomic copy of producer index
  unsigned long queueEntry = 0;

  if (calcQueueLevel(localPI, consumer_index_, QUEUE_SIZE_) > 0) {

    queueEntry = echo_edge_queue_[consumer_index_];

    // Find the next index we will bump the consumer index to once done
    int nextCI = consumer_index_ + 1;
    if (nextCI >= QUEUE_SIZE_) {
      nextCI = 0;     // case of roll-around for next entry
    }

    consumer_index_ = nextCI;   // We have consumed so bump consumer index
  }

  return queueEntry;
}

// This empties the queue and no members are seen, they just go bye-bye
//
// We do return how many members were flushed
int Sonar_Controller::flushQueue() {
  int queueEntries = 0;
  while (pullQueueEntry() != 0) {
    queueEntries++;
  }  // Eat um all up, yum yum.
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

float Sonar_Controller::echoUsToMeters(unsigned long pingDelay) {
  float  meters;
  meters = (float)(pingDelay) / US_TO_METERS_;
  return meters;
}

UShort Sonar_Controller::mm_distance_get(UByte sonar_index) {
  UShort distance = (UShort)-10;  
  if (sonar_index < sonars_size_) {
    Sonar *sonar = sonars_[sonar_index];
    distance = (UShort)(sonar->distance_in_meters * (float)1000.0);
  }
  return distance;
}

