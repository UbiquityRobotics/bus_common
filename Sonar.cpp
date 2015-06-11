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

Sonar_Queue::Sonar_Queue(
 UByte mask_index, volatile uint8_t *io_port_base, UART *debug_uart) {
  volatile uint8_t *change_mask_base = &PCMSK0;
  change_mask_register_ = change_mask_base + mask_index;
  consumer_index_ = 0;
  debug_uart_ = debug_uart;
  interrupt_mask_ = (1 << mask_index);
  io_port_base_ = io_port_base;
  mask_index_ = mask_index;	// Is *mask_index_* used by anyone?
  producer_index_ = 0;
}

void Sonar_Queue::interrupt_service_routine() {
  // Grab the latest input port value:
  UByte echo = io_port_base_[PIN_INDEX_];

  // The act of reading *TCNT1L* causes *TCNT1H* to be cached into
  // a temporary register.  Thus, by reading *TCNT1L* before *TCNT1H*,
  // we git a consistent 16-bit value without having to do silliness
  // disabling interrupts and so forth...
  UByte low_tick = TCNT1L;
  UByte high_tick = TCNT1H;

  // Stash the values away:
  echos_[producer_index_] = echo;
  ticks_[producer_index_] = (((UShort)high_tick) << 8) | ((UShort)low_tick);

  // Increment the *producer_index_*:
  producer_index_ = (producer_index_ + 1) & QUEUE_MASK_;
}

// This routine resets the pin change mask and empties the queue.
void Sonar_Queue::shut_down() {
  // Zero out the pin change mask bits:
  *change_mask_register_ = 0;

  // Empty the queue:
  consumer_index_ = producer_index_;
}

// *Sonar* constructor and methods:

Sonar::Sonar(volatile uint8_t *trigger_base, UByte trigger_bit,
 Sonar_Queue *sonar_queue, UByte change_bit,
 volatile uint8_t *echo_base, UByte echo_bit) {
  change_mask_ = (1 << change_bit);
  debug_uart_ = sonar_queue->debug_uart_get();
  distance_in_meters = (float)0.0;
  echo_start_ticks_ = 0;
  echo_end_ticks_ = 0;
  echo_base_ = echo_base;
  echo_mask_ = (1 << echo_bit);
  sonar_queue_ = sonar_queue;
  trigger_base_ = trigger_base;
  trigger_mask_ = (1 << trigger_bit);
}

// Initialize the appropriate I/O pins for a *Sonar*:
void Sonar::initialize() {
    // Set the triggers to be output pins:
    trigger_base_[PORT_OFFSET_] &= ~trigger_mask_; // Clear output bit
    trigger_base_[DDR_OFFSET_] |= trigger_mask_;   // Set pin to be an output

    // Set the echos to be input pins:
    echo_base_[DDR_OFFSET_] &= ~echo_mask_;	   // Set pin to be an input
}

// The speed of sound is 340.29 M/Sec at sea level.
// A sonar echo is requires a round trip to the object and back.
// Thus, this distance is divided by 2.  Our clock ticks at 4uSec/tick.
// Thus:
//
//   340.29 M    1    1000 mM      1 Sec.       4 uSec.             mM
//   ======== * === * ======= * ============= * =======  =  .68048 ====
//     1 Sec.    2      1 M     1000000 uSec.   1 Tick             Tick

UShort Sonar::mm_distance_get() {
  return ((echo_end_ticks_ - echo_start_ticks_) * 68) / 100;
}

// Set distance to 0xffff and if the sonar is still active.
void Sonar::time_out() {
  if (state_ != STATE_OFF_) {
    echo_start_ticks_ = 0;
    echo_end_ticks_ = 0xffff;
    state_ = STATE_OFF_;
  }
}

void Sonar::trigger_setup() {

  // Verify that echo pulse line is zero:
  UByte echo_bits = echo_base_[PORT_OFFSET_];
  if ((echo_bits & echo_mask_) != 0) {
    // The sonar is still returning an echo.  Perhaps this is from the
    // previous iteration.  Whatever, we can't trigger the sonar until
    // the echo line goes low.  So, we leave this sonar inactive.
    state_ = STATE_OFF_;
    //debug_uart_->string_print((Text)"-");
  } else {
    // Echo pulse is low.  We can activate this sonar:

    // Clear the trigger bit:
    trigger_base_[PORT_OFFSET_] &= ~trigger_mask_;

    // Set the change bit for the *sonar_queue_*:
    sonar_queue_->change_mask_set(change_mask_);

    // Mark this sonar as active:
    state_ = STATE_ECHO_RISE_WAIT_;
    //debug_uart_->string_print((Text)"+");
  }
}

void Sonar::trigger() {
  // Set trigger bit high:
  trigger_base_[PORT_OFFSET_] |= trigger_mask_;

  // Wait for *TRIGGER_TICKS_* to elapse:
  UShort now = TCNT1;
  while (TCNT1 - now < TRIGGER_TICKS_) {
    // do nothing:
  }

  // Clear trigger_bit:
  trigger_base_[PORT_OFFSET_] &= ~trigger_mask_;
}

void Sonar::update(UShort ticks, UByte echo, Sonar_Queue *sonar_queue) {
  //debug_uart_->integer_print(echo);
  //debug_uart_->string_print((Text)" ");
  //debug_uart_->integer_print(echo_mask_);

  switch (state_) {
    case STATE_OFF_: {
      //debug_uart_->string_print((Text)"%");
      break;
    }
    case STATE_ECHO_RISE_WAIT_: {
      if ((echo & echo_mask_) != 0) {
	// We have a rising edge:
	echo_start_ticks_ = ticks;
	state_ = STATE_ECHO_FALL_WAIT_;
	//debug_uart_->string_print((Text)"^");
      }
      break;
    }
    case STATE_ECHO_FALL_WAIT_: {
      if ((echo & ~echo_mask_) == 0) {
	// We have a falling edge:
	echo_end_ticks_ = ticks;
	// If we want, we can compute the distance here:
	distance_in_meters =
	 (float)(echo_end_ticks_ - echo_start_ticks_) / 1000.0;
	state_ = STATE_OFF_;
	//debug_uart_->string_print((Text)"v");
      }
      break;
    }
  }
}

// *Sonars_Controller* constructor:

Sonars_Controller::Sonars_Controller(UART *debug_uart,
 Sonar *sonars[], Sonar_Queue *sonar_queues[], UByte sonars_schedule[]) {
  //debug_uart->begin(16000000L, 115200L, (Character *)"8N1");
  //debug_uart->string_print((Text)"=>Sonars_Controller()!\r\n");

  // Initialize various member variables:
  debug_flags_ = 0;
  debug_uart_ = debug_uart;
  sonar_queues_ = sonar_queues;
  sonars_ = sonars;
  sonars_schedule_ = sonars_schedule;
  state_ = STATE_SHUT_DOWN_;
  general_debug_flag_ = 0;
  error_debug_flag_ = 0;
  results_debug_flag_ = 0;
  //debug_uart->string_print((Text)"1 \r\n");

  // Figure out the value for *sonars_size_*:
  sonars_size_ = 0;
  for (UByte sonar_index = 0; sonar_index <= 255; sonar_index++) {
    if (sonars_[sonar_index] == (Sonar *)0) {
      sonars_size_ = sonar_index;
      break;
    }
  }

  // Figure out the value for *sonar_queues_size_*:
  sonar_queues_size_ = 0;
  for (UByte queue_index = 0; queue_index <= 255; queue_index++) {
    if (sonar_queues[queue_index] == (Sonar_Queue *)0) {
      sonar_queues_size_ = queue_index;
      break;
    }
  }
  //debug_uart->string_print((Text)"4 \r\n");

  // Figure out the value for *sonars_schedule_size_*:
  sonars_schedule_size_ = 0;
  for (UByte schedule_index = 0; schedule_index <= 255; schedule_index++) {
    if (sonars_schedule[schedule_index] == SCHEDULE_END) {
      sonars_schedule_size_ = schedule_index;
      break;
    }
  }
  //debug_uart->string_print((Text)"5 \r\n");

  // Start in the shut down state:
  state_ = STATE_SHUT_DOWN_;

  // Setting *last_schedule_index_* to a large value will force the next
  // schedule group to start at the beginning of *sonar_schedules_*:
  last_schedule_index_ = sonars_schedule_size_;
  first_schedule_index_ = 0;

  //debug_uart->string_print((Text)"<=Sonars_Controller()!\r\n");
}

// *Sonars_Controller* static variables and method(s):

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

// We found that the nice producer consumer queue has to be reset down wo
// just do one meas per loop and reset queue each time so we get 2 edges

// *Sonars_Controller* methods:

// Set the *debug_flags_* variable:

UByte Sonars_Controller::change_mask_get(UByte sonar_index) {
  Sonar *sonar = sonars_[sonar_index];
  UByte change_mask = sonar->change_mask_get();
  return change_mask;
}

void Sonars_Controller::debug_flags_set(UShort debug_flags) {
  debug_flags_ = debug_flags;
}

void Sonars_Controller::debug_flag_values_set(UShort error_debug_flag,
 UShort general_debug_flag, UShort results_debug_flag) {
  error_debug_flag_ = error_debug_flag;
  general_debug_flag_ = general_debug_flag;
  results_debug_flag_ = results_debug_flag;
}

UByte Sonars_Controller::echo_mask_get(UByte sonar_index) {
  Sonar *sonar = sonars_[sonar_index];
  UByte echo_mask = sonar->echo_mask_get();
  return echo_mask;
}

UByte Sonars_Controller::mask_index_get(UByte sonar_index) {
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

unsigned long Sonars_Controller::measurement_trigger(UByte sonar_index) {
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
void Sonars_Controller::initialize() {
  // Initialize each *sonar* and compute *pin_change_interrrupts_mask_*:
  pin_change_interrupts_mask_ = 0;
  for (UByte index = 0; index < sonars_size_; index++) {
    Sonar *sonar = sonars_[index];
    sonar->initialize();
    Sonar_Queue *sonar_queue = sonar->sonar_queue_get();
    pin_change_interrupts_mask_ |= (1 << sonar_queue->mask_index_get());
  }

  // Enable the pin change interrupt registers:
  PCICR |= pin_change_interrupts_mask_;

  // We need to configure Timer 1 to be a 16-bit counter that
  // ticks every 4uSec.  This means it is configured as a simple
  // counter, with a 1/64 prescaler.  With a 16MHz crystal, this
  // works out to 4uSec/Tick.

  // TCCR1A has the following format:
  //
  //   aabbccww
  //
  // where:
  //
  //   aa is the output compare mode for A (we want 00)
  //   bb is the output compare mode for B (we want 00)
  //   cc is the output compare mode for c (we want 00)
  //   ww is the lower two bits of WWww (waveform generation mode) (we want 00)
  //
  // We want "aabbccWW" to be 0000000 (== 0):
  TCCR1A = 0;  

  // TCCR1B has the following format:
  //
  //   ne-WWccc
  //
  // where:
  //
  //   n   is the noise input canceller (not needed, set to 0)
  //   e   is the edge input select (not neededs, set to 0)
  //   WW  is the high two bits of WWww (waveform generation mode (we want 00)
  //   ccc is the clock prescaler (CLKio/64 == 011)
  //
  // We want "ne-WWccc" to be 00000011 (== 3):
  TCCR1B = 3;

  // TCC1C has the following format:
  //
  //    ff------
  // 
  // where:
  //
  //   ff  is the foruce output control bits (not needed, to 0)
  TCCR1C = 0;

  // TIMSK1 has the following format:
  //
  //   --i--ba
  //
  // where:
  //
  //   i   is the input capture interrupt enable.
  //   b   is the B compare match interrupt enable.
  //   a   is the A compare match interrupt enable.
  //   t   is the timer overflow interrupt enable.
  //
  // We want no interrupts, so set "--i--baa" to 00000000 (== 0);
  TIMSK1 = 0;

  // We can ignore the interrupt flag register TIFR1.

  // On the ATmega640, ATmega1280, and the ATmega2560, we assume
  // that the power reducition registers PRR0 and PRR1 are initialized
  // to 0 and hence, that Timer 1 is enabled.
}

void Sonars_Controller::poll() {
  switch (state_) {
    case STATE_SHUT_DOWN_: {
      if (last_schedule_index_ + 2 >= sonars_schedule_size_) {
        //debug_uart_->string_print((Text)"\r\n");
      }
      //debug_uart_->string_print((Text)"\r\nA");
      // Shut down each *sonar_queue*:
      for (UByte queue_index = 0;
       queue_index <= sonar_queues_size_; queue_index++) {
	sonar_queues_[queue_index]->shut_down();
      }

      // Disable pin change interrupts:
      PCICR &= ~pin_change_interrupts_mask_;

      // Next, select another group of sonars to trigger:
      state_ = STATE_GROUP_NEXT_;
      break;
    }
    case STATE_GROUP_NEXT_: {
      //debug_uart_->string_print((Text)"B");
      // Figure out what the next sonar group in the *sonars_schedule* is:

      // Start with the assumption the we skip over a GROUP_END_:
      first_schedule_index_ = last_schedule_index_ + 2;

      // If *first_schedule_index_* is too big, reset it to 0:
      if (first_schedule_index_ >= sonars_schedule_size_) {
	first_schedule_index_ = 0;
      }

      // Now hunt for the the correct value for *last_schedule_index_*:
      last_schedule_index_ = first_schedule_index_;
      for (UByte schedule_index = first_schedule_index_;
       schedule_index < sonars_schedule_size_; schedule_index++) {
	if (sonars_schedule_[schedule_index + 1] == GROUP_END) {
	  last_schedule_index_ = schedule_index;
	  break;
	}
      }

      // Next, do the prework for the set of sonar triggers:
      state_ = STATE_TRIGGER_SETUP_;

      //debug_uart_->integer_print(first_schedule_index_);
      break;
    }
    case STATE_TRIGGER_SETUP_: {
      //debug_uart_->string_print((Text)"C");
      // Enable each sonar for pin-change interrupts:
      for (UByte schedule_index = first_schedule_index_;
       schedule_index <= last_schedule_index_; schedule_index++) {
	sonars_[sonars_schedule_[schedule_index]]->trigger_setup();
      }
      // Next, trigger each of the sonars:
      state_ = STATE_TRIGGER_;
      break;
    }
    case STATE_TRIGGER_: {
      //debug_uart_->string_print((Text)"D");
      // Advance to the next block of sonars:

      // Grab the starting time:
      start_ticks_ = TCNT1;
      now_ticks_ = start_ticks_;

      // Enable the pin change interrupts:
      PCICR |= pin_change_interrupts_mask_;

      // Trigger all the sonars:
      for (UByte schedule_index = first_schedule_index_;
       schedule_index <= last_schedule_index_; schedule_index++) {
	sonars_[sonars_schedule_[schedule_index]]->trigger();
      }

      // Now wait for the echos to come in:
      state_ = STATE_ECHO_WAIT_;
      break;
    }
    case STATE_ECHO_WAIT_: {
      //debug_uart_->string_print((Text)"<");
      // Remember *previous_now_ticks_* and get the latest *now_ticks_*:
      previous_now_ticks_ = now_ticks_;
      //UByte now_ticks_low = TCNT1L;
      //UByte now_ticks_high = TCNT1H;
      //now_ticks_ = (((UShort)now_ticks_high) << 8) | (UShort)now_ticks_low;
      now_ticks_ = TCNT1;

      //debug_uart_->integer_print(now_ticks_);
      //debug_uart_->integer_print(first_schedule_index_);

      // Compute the deltas for both using *start_ticks_*:
      UShort delta_ticks = now_ticks_ - start_ticks_;
      UShort previous_delta_ticks = previous_now_ticks_ - start_ticks_;

      // There are two ways that we can time out.
      // * We can exceed *TIMEOUT_TICKS_*, or
      // * We can can wrap around 2^16 and notice that our *delta_ticks*
      //   has gotten smaller than *previous_delta_ticks*:
      // The if statement below notices both conditions:
      if (delta_ticks >= TIMEOUT_TICKS_ || delta_ticks < previous_delta_ticks) {
	// We have totally timed out and need to shut everything down
	// for this group of sonars.  Visit each sonar and time-out each
	// sonar that does have a value:
        for (UByte schedule_index = first_schedule_index_;
         schedule_index <= last_schedule_index_; schedule_index++) {
	  sonars_[sonars_schedule_[schedule_index]]->time_out();
	}
	// Since we are done, we shut down shut everything down and
	// go to the next sonar group in the sonar schedule:
        //debug_uart_->string_print((Text)"<F>");

	state_ = STATE_SHUT_DOWN_;
	return;
      }

      // Visit each *sonar_queue*.  The first non-empty sonar queue
      // gets immeidately processed:
      for (UByte queue_index = 0;
       queue_index < sonar_queues_size_; queue_index++) {
	Sonar_Queue *sonar_queue = sonar_queues_[queue_index];
	if (!sonar_queue->is_empty()) {
	  // Process the queue:
          //debug_uart_->string_print((Text)"<");
	  UShort tick = sonar_queue->ticks_peek();
	  UByte echo = sonar_queue->echos_peek();
	  sonar_queue->consume_one();

	  // Now visit each *sonar* and see let it decide if it wants
	  // do anything with the values:
	  for (UByte schedule_index = first_schedule_index_;
	   schedule_index <= last_schedule_index_; schedule_index++) {
	    Sonar *sonar = sonars_[sonars_schedule_[schedule_index]];
	    sonar->update(tick, echo, sonar_queue);
	  }

	  // The next time through we'll proccess any remaining
	  // non-empty sonar queues. We remaining in the same *state_*:
          //debug_uart_->string_print((Text)"G>");
	  return;
	}
      }

      // None of the sonar queues needed processing, lets figure out
      // if all of the sonars are done:
      UByte done_count = 0;
      for (UByte schedule_index = first_schedule_index_;
       schedule_index <= last_schedule_index_; schedule_index++) {
	if (sonars_[sonars_schedule_[schedule_index]]->is_done()) {
	  done_count += 1;
	}
      }

      // If every sonar in the current group is done, we can move onto
      // the next group:
      if (done_count >= last_schedule_index_ - first_schedule_index_ + 1) {
        // We are done:
        state_ = STATE_SHUT_DOWN_;
        //debug_uart_->string_print((Text)"<H>");
        return;
      }

      // Otherwise, we are still waiting for a sonar to finish up
      // and remain in the same state:
      return;       
      break;
    }
    default: {
      //debug_uart_->string_print((Text)"<Z>");
      // It should be impossible to get here, but just in case, let's force
      // *state_* to be valid for the next time around:
      state_ = STATE_SHUT_DOWN_;
      break;
    }
  }
}

UShort Sonars_Controller::mm_distance_get(UByte sonar_index) {
  return sonars_[sonar_index]->mm_distance_get();
}

