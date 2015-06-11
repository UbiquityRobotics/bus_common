// Copyright (c) 2015 by Mark Johnston.  All rights reserved.
// Copyright (c) 2015 by Wayne Gramlich.  All rights reserved.

/// @file
///
/// # `Sonars_Controller` Module
///
/// The `Sonars_Controller` module is used to control a bunch of sonar
/// modules with separate trigger and echo pins.  The HC-SR04 is the
/// nominal module to be used.
///
/// ## Documentation and Code Formatting
///
/// The documentation is mostly written in markdown format:
///
///     http://daringfireball.net/projects/markdown/
///
/// A short summary of markdown is:
///
/// * Back quotes (e.g. \``back_quotes`\`)  are used to change to a
///   `fixed pitch code` font.
///
/// * Single asterisks (e.g. `*italics*`) are used for an *italics* font.
/// 
/// * Double asterisks (e.g. `**bold**`)  are used for a **bold** font.
///
/// * A line starting with an asterisk is an itemized list bullet item.
///
/// * Lines starting with a greater than sign (i.e. `>`) indicate block
///   quoted indented text.  This the same way it is done in E-mail.
///
/// * Lines that start with one or more "hash" characters (i.e. `#`)
///   indicate a
///   heading with the number indicating heading level.
///
/// * Lines indented by 8 spaces or more are code samples.
///
/// * HTML markup is used for tables, special characters, etc.
///
/// When markdown is embedded in C/C++ code, it is preceded by a
/// triple slash comment on each line (i.e. `///`).  This is standard
/// `doxygen` format (see below.)
///
/// There are many C/C++ coding standards out there.  The ones used for
/// this code are:
///
/// * Indentation is 2 spaces per code level.
///
/// * Lines are no longer that 80 characters.
///
/// * Continuation lines are indented by 1 space beyond the current
///   code block indent level.
///
/// * K&R curly brace indentation is used.
///
/// * All single line code blocks are enclosed in curly braces.
///
/// * All types and classes start with a capital letter (e.g. `Sonar`,
///   `Sonar_Queue`, etc.)  `CamalCase` types are not used.  Instead,
///   Underscores separate words in type names (e.g. `Sonar_Queue`,
///   `Sonars_Controller`, etc.)
///
/// * The 8/16/32/64 bit signed types are `Byte`, `Short`, `Integer`,
///   and `Long`.  The unsigned 8/16/32/64 bit signed types are `UByte`,
///   `UShort`, `UInteger`, and `ULong`.  `Character` and `Text` are used
///   instead of `char` and `char *`.  These types are currently defined
///   in the file `bus_common/bus_slave.h`.
///
/// * `Logical` is the type used for true and false.
///
/// * All variables start with a lower case letter and use underscores
///   to separate words (e.g. `echo_mask`, `sonars_schedule`, etc.
///   Words tend to be spelled out in their entirety with no syllable
///   or vowel dropping.  `camelCaseVariable` names are not used.
///
/// * Private member functions and variables are marked with a trailing
///   underscore (e.g. `producer_index_`, `consumer_index_`, etc.)
///
/// * Constants are in all upper case.  They are mostly declared as
///   `static const` rather that #`define`.
///
/// * #`ifdef` are discouraged.  If used they are indented like any
///   other language construct.
///
/// * In general, a "paragraph" style is used for each code block.
///   In this style, a comment precedes each code block that explains
///   what the code block is trying to accomplish.
///
/// That pretty much covers the C++ coding style.
///
/// In addition, `doxygen` is used to produce HTML class documenation.
///
///     http://www.doxygen.org/
///
/// By the way, the more recent versions of `doxygen` use markdown syntax
/// for font changes and the like.  The `doxygen` formatting style is:
///
/// * C++ code uses triple forward slash (e.g. `///`) to indicate
///   `doxygen` comments.
///
/// * The at sign (e.g. `@`) is used to indicate `doxygen` commands.
///   (Note that the `doxygen` manual tends to use the backslash format
///   (e.g. `\param` instead of `@param`.)
///
/// ## `Sonars_Controller` Overview
///
/// The `Sonars_Controller` system is broken into three C++ classes:
///
/// * `Sonar_Queue`.  There is one `Sonar_Queue` for each pin change
///   interrupt vector that is used.  This class basically stuffs
///   (time, port) into a queue.  The queue is emptied by the higher
///   level `Sonars_Controller::poll()` method.
///
/// * `Sonar`.  There is one `Sonar` for each physical sonar object.
///   This class specifies the trigger pin, the echo pin, and which
///   `Sonar_Queue` will collect the echo pin changes.
///
/// * `Sonar_Controller`.  There is only one `Sonar_Controller`.
///   This class takes a list of `Sonar_Queue`'s, a list of `Sonar`'s,
///   A something called the "sonars schedule" and manages trigger
///   the sonars and timing the echo returns.
///
/// ## Code Usage
///
/// To use this code you need to do the following.
///
/// * Add #`include <Sonar.h>`:
///
///        #include <Sonar.h>
///
/// * Define a debug uart.  It can be either a null UART or a real UART:
///
///        NULL_UART null_uart;
///
/// * Use the `Sonar_Queue` constructor to define each needed sonar queue.
///
///        Sonar_Queue b_sonar_queue(0, &PINB, debug_uart);
///        Sonar_Queue d_sonar_queue(2, &PIND, debug_uart);
///
/// * Create a null terminated list of `Sonar_Queue`'s:
///
///        Sonar_Queue *sonar_queues[] = {
///          &b_sonar_queue,
///          &d_sonar_queue,
///          (Sonar_Queue *)0,
///        };
///
/// * Use the `Sonar` constructor for each sonar module:
///
///        Sonar sonar0(&PINC, 1, &b_sonar_queue, 1, 1);
///        // ...
///        Sonar sonar9(&PIND, 2, &d_sonar_queue, 3, 3);
///
/// * Create a null terminated `Sonar`'s list:
///
///        Sonar *sonars[] = {
///          &sonar0,
///          // ...
///          &sonar9,
///         (Sonar *)0,
///        };
///
/// * Create a sonars schedule.  This a byte string that specifies
///   groups of sonars to trigger at the same time:
///
///        UByte sonars_schedule[] = {
///          0, 5, Sonars_Controller::GROUP_END,
///          /...
///          4, 9, Sonars_Controller::GROUP_END,
///          Sonars_Controller::SCHEDULE_END,
///        };
///
///   Each sonar is specified by it index in the sonars list.
///   The end of a group is indicated by `Sonars_Controller::GROUP_END`.
///   The list end is indicated  by `Sonars_Controller::SCHEDULE_END`.
///
/// * Use the `Sonar_Constructor` constructor to create the one and
///    only `Sonar_Constructor` object:
///
///       Sonars_Controller sonars_controller((UART *)debug_uart,
///        sonars, sonar_queues, sonars_schedule);
///
/// * Create an interrupt service routine for each `Sonar_Queue` object.
///   This simply calls `Sonar_Queue::interrupt_service_routine()`:
///
///        ISR(PCINT0_vect) {
///          b_sonar_queue.interrupt_service_routine();
///        }
///
/// * Make sure that `Sonars_Controller::initialize()` is called once.
///
///        void setup() {
///            sonars_controller.initialize();
///        }
///
/// * Make sure that `Sonars_Controller::poll()` is called on a regular basis:
///
///        void loop() {
///            sonars_controller.poll();
///        }
///
/// That is all.
///
/// {Talk about Sonar modules.}
///
/// ## `Sonar_Queue` Details.
///
/// The `Sonar_Queue` object implements a fixed size buffer that is
/// used as a FIFO queue.  Each time a pin change interrupt occurs
/// the current echo port pins and a time stamp are entered into the
/// queue.
///
/// Time stamp is provided by the 16-bit Timer 1 counter.  This timer
/// is configured by the `Sonar_Controller` to increment every 4&mu;Sec.
/// (This assumes a 16MHz system clock.)
///
/// Each I/O port is controlled by three registers called PIN, DDR,
/// and PORT, where PIN is the port input register, DDR is the data
/// direction register, and PORT is the port output register.  These
/// three registers always occur as three consecutive registers.  We
/// always specify an I/O port

#include <Arduino.h>
#include <Sonar.h>

// *Sonar_Queue* constructor and methods:

/// @brief Construct a `Sonar_Queue` object.
/// @param mask_index is the pin change interrupt mask number (e.g. PCMSK0,...).
/// @param echo_registers is the base register echo input pins.
/// @param debug_uart is a UART for debugging messages.
///
/// This constructor creates a sonar object that can store information
/// about a pin change interrupt into a queue.  **mask_index** specifies the
/// which PCMSK0, ..., PCMSK2, mask register is used.  Thus, **mask_index**
/// specifies which set of PCINT pins are used (0=>PCINT1,...,PCINT8,
/// 1=>PCING9,...PCINT16, and 2=>PCINT17,...PCINT24.)  **echo_registers**
/// specifies the base address register for the echo pins I/O port.
/// **debug_uart** is a uart used for debugging output.

Sonar_Queue::Sonar_Queue(
 UByte mask_index, volatile uint8_t *echo_registers, UART *debug_uart) {
  // Save *debug_uart*, *echo_registers*, *mask_index* into private variables:
  debug_uart_ = debug_uart;
  echo_registers_ = echo_registers;
  mask_index_ = mask_index;

  // Store PCMSK0, PCMSK1, or PCMSK2 into *change_mask_register_* depending
  // upon value of *mask_index*:
  volatile uint8_t *change_mask_registers = &PCMSK0;
  change_mask_register_ = change_mask_registers + mask_index;

  // Compute the interrupt mask bit to be used for PCICR from *mask_index*
  // and store into *interrupt_mask_*:
  interrupt_mask_ = (1 << mask_index);

  // Zero out the consume and producer indices:
  consumer_index_ = 0;
  producer_index_ = 0;
}

/// @brief Set the echo I/O port input pin direction.
/// @param echo_mask is the mask of pins to set to inputs.
///
/// This method will set the I/O pins specified by **echo_mask** to
/// be input bits.  This method only affects pins specified by a 1
/// in **echo_mask**; all other pins remain unchanged.

void Sonar_Queue::input_direction_set(UByte echo_mask) {
  echo_registers_[DIRECTION_] &= ~echo_mask;
}

/// @brief process a pin change interrupt.
///
/// This method will stuff (echo_bits, ticks) pair into the
/// sonar queue in FIFO order.  The **consume_one**(),
/// **echo_bits_peek**(), and **ticks_peek**() methods are used
/// to get the data out of the queue.

void Sonar_Queue::interrupt_service_routine() {
  // Grab the latest input port value:
  UByte echo = echo_registers_[INPUT_];

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

/// @brief Shuts down any further pin change interrupts and flush the queue.
///
/// This method will flush the FIFO queue and disable all interrupt
/// change pins from further interrupts.

void Sonar_Queue::shut_down() {
  // Zero out the pin change mask bits:
  *change_mask_register_ = 0;

  // Empty the queue:
  consumer_index_ = producer_index_;
}

// *Sonar* constructor and methods:

/// @brief constructs a new *Sonar* object.
/// @param trigger_registers specifies the I/O port used to trigger sonar.
/// @param trigger_bit specifies with pin to use for sonar trigger.
/// @param sonar_queue specifies which `Sonar_Queue` will get echo pin changes.
/// @param change_bit specifies which pin change to connected to the echo pin.
/// @param echo_bit specifies which pin to use for sonar echos.
///
/// *Sonar()* constructs a sonar object that is one-to-one with a
/// physical sonar module.  **trigger_registers** specifies the I/O
/// port that is connected to the trigger pin.  **trigger_bit** specifies
/// which bit of the trigger I/O port is the pin to trigger with.
/// **sonar_queue** specifies the `Sonar_Queue` object to use to collect
/// the echo pin changes with.  **change_bit** specifies which pin of
/// the appropriate pin change mask register is connected to the echo pin.
/// **echo_pin** specifies which pin of the echo I/P port is connected
/// to the echo pin.
///
/// The **change_bit** and **echo_bit** can be a bit confusing.  On most
/// Atmel AVR processors, the pin change mask register has a one-to-one
/// correspondence with pins in the pin change mask register.  This is
/// true for the ATmega328 processor.  For other processors, like the
/// ATmega2560, the correspondence is not one-to-one.  You will have to
/// look carefully at processor data sheet and the schematic to figure
/// out which pins to use.

Sonar::Sonar(volatile uint8_t *trigger_registers, UByte trigger_bit,
 Sonar_Queue *sonar_queue, UByte change_bit, UByte echo_bit) {
  pin_change_mask_ = (1 << change_bit);
  debug_uart_ = sonar_queue->debug_uart_get();
  echo_start_ticks_ = 0;
  echo_end_ticks_ = 0;
  echo_mask_ = (1 << echo_bit);
  sonar_queue_ = sonar_queue;
  trigger_registers_ = trigger_registers;
  trigger_mask_ = (1 << trigger_bit);
}

/// @brief Initialize the sonar.
///
/// This method initializes the trigger and echo pins for the sonar.

void Sonar::initialize() {
    // Set the trigger pin to be an output pin:
    trigger_registers_[OUTPUT_] &= ~trigger_mask_;   // Clear first
    trigger_registers_[DIRECTION_] |= trigger_mask_; // 1=>output;0=>input

    // Set the echos to be input pins:
    //echo_base_[DDR_OFFSET_] &= ~echo_mask_;
    sonar_queue_->input_direction_set(echo_mask_);
}

/// @brief Return the distance in millimeters
/// @returns the distance in millimeters.
///
/// This method will return the last successful sonar measurement
/// in millimeters.

UShort Sonar::mm_distance_get() {
  // The speed of sound is 340.29 M/Sec at sea level.
  // A sonar echo is requires a round trip to the object and back.
  // Thus, this distance is divided by 2.  Our clock ticks at 4uSec/tick.
  // Thus:
  //
  //   340.29 M    1    1000 mM      1 Sec.       4 uSec.             mM
  //   ======== * === * ======= * ============= * =======  =  .68048 ====
  //     1 Sec.    2      1 M     1000000 uSec.   1 Tick             Tick

  return ((echo_end_ticks_ - echo_start_ticks_) * 68) / 100;
}

/// @brief Marks that the current sonar measurement has timed out.
///
/// This method will mark the current sonar measurement as timed out.
void Sonar::time_out() {
  if (state_ != STATE_OFF_) {
    echo_start_ticks_ = 0;
    echo_end_ticks_ = 0xffff;
    state_ = STATE_OFF_;
  }
}

/// @brief Trigger the sonar.
///
/// This method will cause a sonar trigger pulse to be generated.
/// The **trigger_setup**() method should be called prior to this method.
void Sonar::trigger() {
  // Set trigger bit high:
  trigger_registers_[OUTPUT_] |= trigger_mask_;

  // Wait for *TRIGGER_TICKS_* to elapse:
  UShort now = TCNT1;
  while (TCNT1 - now < TRIGGER_TICKS_) {
    // do nothing:
  }

  // Clear trigger_bit:
  trigger_registers_[OUTPUT_] &= ~trigger_mask_;
}

/// @brief Prepare the sonar to be triggered.
///
/// This method will prepare the sonar to be triggered.
/// This method should be called prior to the **trigger** method.
void Sonar::trigger_setup() {

  // Verify that echo pulse line is zero:
  UByte echo_bits = sonar_queue_->echo_bits_get();
  if ((echo_bits & echo_mask_) != 0) {
    // The sonar is still returning an echo.  Perhaps this is from the
    // previous iteration.  Whatever, we can't trigger the sonar until
    // the echo line goes low.  So, we leave this sonar inactive.
    state_ = STATE_OFF_;
    //debug_uart_->string_print((Text)"-");
  } else {
    // Echo pulse is low.  We can activate this sonar:

    // Clear the trigger bit:
    trigger_registers_[OUTPUT_] &= ~trigger_mask_;

    // Set the change bit for the *sonar_queue_*:
    sonar_queue_->pin_change_mask_set(pin_change_mask_);

    // Mark this sonar as active:
    state_ = STATE_ECHO_RISE_WAIT_;
    //debug_uart_->string_print((Text)"+");
  }
}

/// @brief Update the sonar state.
/// @param ticks is the current ticks from the sonar queue.
/// @param echo_bits is the current echo bits from the sonar queue.
/// @param sonar_queue is the associated `Sonar_Queue` object.
///
/// This method will update the state of the sonar using **ticks** and
/// **echo_bits** as additional input.  **sonar_queue** is only used
/// to get access to the debug uart.
void Sonar::update(UShort ticks, UByte echo_bits, Sonar_Queue *sonar_queue) {
  //debug_uart_->integer_print(echo);
  //debug_uart_->string_print((Text)" ");
  //debug_uart_->integer_print(echo_mask_);

  // The only states we care about are for rising and falling echo edges:
  switch (state_) {
    case STATE_OFF_: {
      //debug_uart_->string_print((Text)"%");
      break;
    }
    case STATE_ECHO_RISE_WAIT_: {
      // We are waiting for the echo pin to go from 0 to 1:
      if ((echo_bits & echo_mask_) != 0) {
	// Since we have a rising edge, we simply remember when it
	// occurred and wait for the subsequent falling edge:
	echo_start_ticks_ = ticks;
	state_ = STATE_ECHO_FALL_WAIT_;
	//debug_uart_->string_print((Text)"^");
      }
      break;
    }
    case STATE_ECHO_FALL_WAIT_: {
      // We are waiting for the echo pin to go from 1 to 0:
      if ((echo_bits & ~echo_mask_) == 0) {
	// Since we have a falling edge, we simply remember when it
	// occurred and mark that we are done:
	echo_end_ticks_ = ticks;
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
  debug_uart_ = debug_uart;
  sonar_queues_ = sonar_queues;
  sonars_ = sonars;
  sonars_schedule_ = sonars_schedule;
  state_ = STATE_SHUT_DOWN_;
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

UByte Sonars_Controller::pin_change_mask_get(UByte sonar_index) {
  Sonar *sonar = sonars_[sonar_index];
  UByte change_mask = sonar->pin_change_mask_get();
  return change_mask;
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

/// @brief Initialize the controller.

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

