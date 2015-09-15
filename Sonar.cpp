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
/// ## Miscellaneous Issues
///
/// ### `Sonar_Queue` Details
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
/// ### `Sonar` Details
///
/// The key issue with `Sonar` object is that echo pin numbers and
/// pin change pin numbers can be confusing.  When you look at the
/// schematic you will see an echo line going to a pin labeled
/// `Pln/PCINTm`, where:
///
/// * `l` is the port letter (e.g. `A`, `B`, `C`, ...),
///
/// * `n` is the bit number of the port (e.g. `0`, `1`, ..., `7`),
///
/// * `m` is the pin change interrupt number.  This number is from
///   `0` through `23`.  
///
/// `x` and `n` are easy to understand.  The `m` number needs to be
/// converted into a mask index register and bit number.  The process
/// is to mask off the appropriate bits.  Thus,
///
/// * `b` = `m` & 7   // The low order 3 bits
///
/// * `x` = `m` // The remaining high order bits.
///
/// This is summarized in the table below:
///
///
/// ### `Sonar_Controller` Details
///
/// {More here}
///
/// ### Timer1
///
/// Timer1 in configured as a 16-bit counter that increments once
/// every 4&mu;Sec.  While the AVR is really a 8-bit processor,
/// the AVR designers added a mirror register to the Timer1.
/// When the `TCNT1L` register is read, the AVR processor stores
/// the high order 8 bits of the counter into the mirror register.
/// When the `TCNT1H` is accessed the mirror register is returned.
/// What this means is that you can get a consistent 16-bit counter
/// value without having to do a bunch of schenanigans.
///
/// The GCC compiler when it sees an access to TCNT1, it will access
/// both registers in the correct order.  So, utlimately, the right
/// thing happens, but it is worth understanding what is really going on.
///
/// Each I/O port is controlled by three registers called `PIN`, `DDR`,
/// and `PORT`, where
///
/// * `PIN` is the port input register,
///
/// * `DDR` is the data direction register, and
/// 
/// * `PORT` is the port output register.
///
/// These three registers always occur as three consecutive registers.
/// We always specify an I/O port as an array using the constants:
///
/// * `INPUT_` is the offset that gets to the `PIN` register,
///
/// * `DIRECTION_` is the offset that gets to the `DDR` register, and
///
/// * `OUTPUT_` is the offset that gets to the `PORT` register.

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
  echos_[producer_index_] = echo_registers_[INPUT_];

  // Grab the counter value;
  ticks_[producer_index_] = TCNT1;

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
/// @param pcint_index specifies which pin change to connected to the echo pin.
/// @param echo_bit specifies which pin to use for sonar echos.
///
/// *Sonar()* constructs a sonar object that is one-to-one with a
/// physical sonar module.  **trigger_registers** specifies the I/O
/// port that is connected to the trigger pin.  **trigger_bit** specifies
/// which bit of the trigger I/O port is the pin to trigger with.
/// **sonar_queue** specifies the `Sonar_Queue` object to use to collect
/// the echo pin changes with.  **pcint_index** specifies which `PCINTn`
/// is being used for the echo pulse.  **echo_pin** specifies which pin
/// of the echo I/P port is connected to the echo pin.

Sonar::Sonar(volatile uint8_t *trigger_registers, UByte trigger_bit,
 Sonar_Queue *sonar_queue, UByte pcint_index, UByte echo_bit) {
  UByte pin_change_bit = pcint_index & 7;
  pin_change_mask_ = (1 << pin_change_bit);
  debug_uart_ = sonar_queue->debug_uart_get();
  echo_delta_ticks_ = 0;
  echo_end_ticks_ = 0;
  echo_mask_ = (1 << echo_bit);
  echo_start_ticks_ = 0;
  queue_available_ = (Logical)1;
  queue_time_ = 0;
  queue_value_ = 0;
  sonar_queue_ = sonar_queue;
  trigger_registers_ = trigger_registers;
  trigger_mask_ = (1 << trigger_bit);
}

void Sonar::configure(UByte sonar_class, Byte left_id, Byte right_id) {
  sonar_class_ = sonar_class;
  left_id_ = left_id;
  right_id_ = right_id;
}

/// @brief Initialize the sonar.
/// @param sonar_index is the index for this sonar
///
/// This method initializes the trigger and echo pins for the sonar.
/// **sonar_index** is the index for this sonar in the sonars list.
void Sonar::initialize(UByte sonar_index, UShort *shared_changes_mask) {
    // Set the trigger pin to be an output pin:
    trigger_registers_[OUTPUT_] &= ~trigger_mask_;   // Clear first
    trigger_registers_[DIRECTION_] |= trigger_mask_; // 1=>output;0=>input

    // Set the echos to be input pins:
    sonar_queue_->input_direction_set(echo_mask_);

    // Initialize some private variables:
    echo_delta_ticks_ = 0;
    echo_end_ticks_ = 0;
    echo_start_ticks_ = 0;
    shared_changes_mask_ = shared_changes_mask;
    sonar_mask_ = (1 << sonar_index);    
}

/// @brief Return the distance in millimeters
/// @returns the distance in millimeters.
///
/// This method will return the last successful sonar measurement
/// in millimeters.

UShort Sonar::mm_distance_get() {

  UInteger mm_int;

  // Clear the changed bit for this snoar:
  *shared_changes_mask_ &= sonar_mask_;

  //debug_uart_->string_print((Text)"{");
  //debug_uart_->integer_print(echo_delta_ticks_);
  //debug_uart_->string_print((Text)"}");

  // Trap for special timeout value for the measurement
  if ((echo_start_ticks_ == SONAR_MEAS_TIMEOUT_START_TICKS_) &&
      (echo_end_ticks_   == SONAR_MEAS_TIMEOUT_END_TICKS_)) {
    return SONAR_MEAS_TIMEOUT_MM_;
  } 

  // Compute the distance and return it.  The speed of sound is 
  // 340.29 M/Sec at sea level.  A sonar echo is requires a round
  // trip to the object and back.  Thus, this distance is divided by 2.
  //  Our clock ticks at 4uSec/tick.  Thus:
  //
  //   340.29 M    1    1000 mM      1 Sec.       4 uSec.             mM
  //   ======== * === * ======= * ============= * =======  =  .68048 ====
  //     1 Sec.    2      1 M     1000000 uSec.   1 Tick             Tick
  //mm_int = ((UInteger)(echo_end_ticks_ - echo_start_ticks_) * 68) / 100;
  mm_int = (((UInteger)echo_delta_ticks_) * 68) / 100;

  return (UShort)mm_int;
}

Logical Sonar::is_close() {
  Logical result = (Logical)0;
  UShort mm_distance = mm_distance_get();
  
  // 500 ~= 19 inches:
  if (mm_distance < 500) {
    result = (Logical)1;
  }
  return result;
}

UByte Sonar::priority_compute(Sonar **sonars, Byte direction) {
  Logical left_is_close = (Logical)0;
  if (left_id_ >= 0) {
    left_is_close = sonars[left_id_]->is_close();
  }
  Logical right_is_close = (Logical)0;
  if (right_id_ >= 0) {
    right_is_close = sonars[right_id_]->is_close();
  }

  UByte priority_delta = PRIORITY_BACKGROUND_;
  if (direction >= 0 && sonar_class_ == CLASS_FRONT) {
    // Robot is trending foward, turning in place, or stopped:
    if (is_close()) {
      priority_delta = PRIORITY_CLOSE_;
    } else if (left_is_close || right_is_close) {
      priority_delta = PRIORITY_NEAR_CLOSE_;
    } else {
      priority_delta = PRIORITY_TRAVEL_DIR_;
    }
  } else if (direction < 0 && sonar_class_ == CLASS_BACK) {
    // Robot is trending backward:
    if (is_close()) {
      priority_delta = PRIORITY_CLOSE_;
    } else if (left_is_close || right_is_close) {
      priority_delta = PRIORITY_NEAR_CLOSE_;
    } else {
      priority_delta = PRIORITY_TRAVEL_DIR_;
    }
  }

  // Incrment *priority*:
  priority_ += priority_delta;
  return priority_;
}

/// @brief Marks that the current sonar measurement has timed out.
///
/// This method will mark the current sonar measurement as timed out.
void Sonar::time_out() {
  if (state_ != STATE_OFF_) {
    echo_start_ticks_ = SONAR_MEAS_TIMEOUT_START_TICKS_;
    echo_end_ticks_   = SONAR_MEAS_TIMEOUT_END_TICKS_;
    state_ = STATE_OFF_;
  }
}

void Sonar::queue_poll(UART *host_uart, UInteger time_base, UByte id) {
  if (queue_available_) {
    queue_available_ = (Logical)0;
    host_uart->string_print((Text)" ");
    host_uart->integer_print((Integer)id);
    host_uart->string_print((Text)":");
    host_uart->integer_print((Integer)(queue_time_ - time_base));
    host_uart->string_print((Text)":");
    host_uart->integer_print((Integer)queue_value_);
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
  //debug_uart_->integer_print(echo_bits);
  //debug_uart_->string_print((Text)":");
  //debug_uart_->integer_print(echo_mask_);

  // The only states we care about are for rising and falling echo edges:
  switch (state_) {
    case STATE_OFF_: {
      //debug_uart_->string_print((Text)"%");
      break;
    }
    case STATE_ECHO_RISE_WAIT_: {
      // We are waiting for the echo pin to go from 0 to 1:
      //debug_uart_->string_print((Text)"r");
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
      //debug_uart_->string_print((Text)"f");
      if ((echo_bits & echo_mask_) == 0) {
	// Since we have a falling edge, we simply remember when it
	// occurred and mark that we are done:
	echo_end_ticks_ = ticks;

        UShort echo_delta_ticks = echo_end_ticks_ - echo_start_ticks_;
	queue_time_ = micros();
	queue_value_ = mm_distance_get();
	queue_available_ = (Logical)1;

	// Compute *echo_delta_ticks* and if it seems valid, see if it
	// has changed:
        //UShort echo_delta_ticks = echo_delta_ticks_;;
        //if (echo_end_ticks_ > echo_start_ticks_) {
        //  echo_delta_ticks = echo_end_ticks_ - echo_start_ticks_;

        //} else {
          // Tic counter rollover case. We have option of setting special
	  // value or some cap
          // echo_delta_ticks = SONAR_MAX_TIC_CAP;
        //}

	if (echo_delta_ticks_ != echo_delta_ticks) {
	  echo_delta_ticks_ = echo_delta_ticks;
	  *shared_changes_mask_ |= sonar_mask_;
	}

	state_ = STATE_OFF_;
        //delay(20);
	//debug_uart_->string_print((Text)"v");
	//debug_uart_->integer_print(echo_delta_ticks_);
	//debug_uart_->string_print("\r\n");
      }
      break;
    }
    default: {
      //debug_uart_->string_print((Text)"!");
      break;
    }

  }
}

// *Sonars_Controller* constructor:

/// @brief Construct `Sonars_Controller` object.
/// @param debug_uart is used for debugging print out.
/// @param sonars is a null-terminated list of sonar objects.
/// @param sonar_queues is a null-terminated list of `Sonar_Queue` objects.
/// @param sonars_schedule is a byte sequence specifying sonar trigger order.
///
/// This constructor constructs a `Sonar_Controllers` object. **debug_uart**
/// is used for debugging only.  **sonars** is a null terminated list of
/// `Sonar` object addresses.  **sonar_queues** is a null terminated list
/// of `Sonar_Queue` object addresses.  **sonars_schedule** a `UByte`
/// list that specifies groups of sonars.  Each sonar is represented by
/// its index in **sonars**.  Thus, 0 represents the first sonar and
/// 5 represents the sixth sonar.  Each group of sonars is terminated,
/// by `Sonars_Controller::GROUP_END`.  A group sonars will be triggered
/// all at once.  (It is up the use to make sure that simultaneously
/// triggered sonar pulses do not interfere with one another.)  The
/// **sonars_schedule** list is terminated by `Sonars_Controller::SCHEDULE_END`.
/// Be sure to call the **initialize**() method prior to calling the
/// **poll**() method after constructing this object.

Sonars_Controller::Sonars_Controller(UART *debug_uart,
 Sonar *sonars[], Sonar_Queue *sonar_queues[], UByte sonars_schedule[]) {
  //debug_uart->begin(16000000L, 115200L, (Character *)"8N1");
  //debug_uart->string_print((Text)"=>Sonars_Controller()!\r\n");

  // Initialize various member variables:
  changes_mask_ = 0;
  debug_uart_ = debug_uart;
  sonar_queues_ = sonar_queues;
  sonars_ = sonars;
  sonars_schedule_ = sonars_schedule;
  state_ = STATE_GROUP_DONE_;
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

  // Figure out the value for *sonars_schedule_size_* and
  // *sonars_schedule_num_groups_*.  We count the number of GROUP_END
  // values which is number of rows or the schedule size
  // and we also count total byte count for the schedule
  sonars_schedule_size_ = 0;
  sonars_schedule_num_groups_ = 0;
  for (UByte schedule_index = 0; schedule_index <= 255; schedule_index++) {
    if (sonars_schedule[schedule_index] == GROUP_END) {
      sonars_schedule_num_groups_ += 1;
    }

    if (sonars_schedule[schedule_index] == SCHEDULE_END) {
      sonars_schedule_size_ = schedule_index;
      break;
    }
  }
  //debug_uart->string_print((Text)"5 \r\n");

  // Start in the state that causes us to move to next group
  state_ = STATE_GROUP_DONE_;

  // Setting *last_schedule_index_* to a large value will force the next
  // schedule group to start at the beginning of *sonar_schedules_*:
  last_schedule_index_ = sonars_schedule_size_;
  first_schedule_index_ = 0;

  //debug_uart->string_print((Text)"<=Sonars_Controller()!\r\n");
}

void Sonars_Controller::group_advance() {
  // bypass the GROUP_END marker 
  first_schedule_index_ = last_schedule_index_ + 2;

  // If *first_schedule_index_* is too big, reset it to 0:
  if (first_schedule_index_ >= sonars_schedule_size_) {
    first_schedule_index_ = 0;
  }

  // Find the *last_schedule_index_* for the group;
  last_schedule_index_ = first_schedule_index_;
  for (UByte schedule_index = first_schedule_index_;
   schedule_index < sonars_schedule_size_; schedule_index++) {
    if (sonars_schedule_[schedule_index + 1] == GROUP_END) {
      last_schedule_index_ = schedule_index;
      break;
    }
  }
}

/// @brief Return the group threshould.
/// @param direction specifies the robot direction.
///
/// This method will return the maximum priority of each sonar
/// in the current group.  *direction* is positive if the robot
/// is trending forward, negative if the robot is trending backwards,
/// zero if the robot is turning place or stopped.
UByte Sonars_Controller::group_threshold() {
  /// Visit each *sonar* in the group:
  UByte threshold = 0;
  Sonar **sonars = sonars_;
  for (UByte schedule_index = first_schedule_index_;
   schedule_index <= last_schedule_index_; schedule_index++) {
    Sonar *sonar = sonars[sonars_schedule_[schedule_index]];

    /// Compute the *priority* and update the *threshold*:
    UByte priority = sonar->priority_compute(sonars, direction_);
    if (priority > threshold) {
      threshold = priority;
    }
  }
  return threshold;
}

/// @brief Reset the priorities of each sonar in the current group.
///
/// This method will reset the priorities of each sonar in the current group.
void Sonars_Controller::group_priorities_reset() {
  for (UByte schedule_index = first_schedule_index_;
   schedule_index <= last_schedule_index_; schedule_index++) {
    Sonar *sonar = sonars_[sonars_schedule_[schedule_index]];
    sonar->priority_reset();
  }
}

/// @brief Initialize the `Sonars_Controller` object.
///
/// This method will compute the pin change interrupts mask and 
/// configure Timer/Counter 1 to be a 16-bit counter that ticks
/// over every 4uSec.
void Sonars_Controller::initialize() {
  // Initialize each *sonar* and compute *pin_change_interrrupts_mask_*:
  pin_change_interrupts_mask_ = 0;
  for (UByte sonar_index = 0; sonar_index < sonars_size_; sonar_index++) {
    Sonar *sonar = sonars_[sonar_index];
    sonar->initialize(sonar_index, &changes_mask_);
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
  // to 0 and hence that Timer 1 is powered up.
}

/// @brief Return the interrupt mask associated with **sonar_index**.
/// @param sonar_index specifies which sonar to use.
/// @returns the associated pin interrupt mask.
///
/// This method will return the pin interrupt mask associated with
/// `Sonar_Queue` associated with **sonar_index**.a

UByte Sonars_Controller::mask_index_get(UByte sonar_index) {
  Sonar *sonar = sonars_[sonar_index];
  Sonar_Queue *sonar_queue = sonar->sonar_queue_get();
  UByte mask_index = sonar_queue->mask_index_get();
  return mask_index;
}

/// @brief Return the last distance measured in millimeters.
/// @param sonar_index specifies the sonar to select.
/// @returns the distance in millimeters.
///
/// This method will select the **sonar_index**'th sonar and return
/// the last measured distance in millimeters.

UShort Sonars_Controller::mm_distance_get(UByte sonar_index) {
  return sonars_[sonar_index]->mm_distance_get();
}

/// @brief Poll the sonars controller to do any pending tasks.
///
/// This method is called repeatably to keep the sonars controller
/// sending out sonar pulses and listening for responses.  Be sure
/// to call the **initialize** method once before calling this method.

void Sonars_Controller::poll() {
  switch (state_) {
    case STATE_GROUP_DONE_: {
      UShort done_ticks;

      //if (last_schedule_index_ + 2 >= sonars_schedule_size_) {
      //  debug_uart_->string_print((Text)"\r\n");
      //}
      //debug_uart_->string_print((Text)"\r\nA");

      // Shut down each *sonar_queue*:
      for (UByte queue_index = 0;
       queue_index <= sonar_queues_size_; queue_index++) {
	sonar_queues_[queue_index]->shut_down();
      }

      // Disable pin change interrupts:
      PCICR &= ~pin_change_interrupts_mask_;

      // Wait a minimal gap between groups to avoid hearing last groups echos
      // that can be still out there and accidentally heard to give false
      // close by.  We will ignore rollover case so this could be improved.
      if (trigger_ticks_ != 0) {
        done_ticks = TCNT1;
        if (done_ticks <= trigger_ticks_) {
          trigger_ticks_ = done_ticks;	// rollover timer so start delay again
          return;     // Still waiting for spacing between groups
        } else if ((done_ticks - trigger_ticks_) < GROUP_SPACING_TICKS_) {
          return;     // Still waiting for spacing between groups
        }
      }

      // Next, select another group of sonars to trigger:
      state_ = STATE_GROUP_NEXT_;
      break;
    }
    case STATE_GROUP_NEXT_: {
      //debug_uart_->string_print((Text)"B");

      // Figure out what the next sonar group in the *sonars_schedule* is:
      // first_schedule_index : Index of first sonar in this row
      // last_schedule_index  : Index of last sonar in this row.
      // first can be same index as last if only one sonar in this row

      // Now hunt for the the correct value for *last_schedule_index_*:
      for (UByte count = 0; count < 8; count++) {
	group_advance();
	if (group_threshold() >= 8) {
	  state_ = STATE_TRIGGER_SETUP_;
	  break;
	}
      }

      // Next, do the prework for the set of sonar triggers:
      //state_ = STATE_TRIGGER_SETUP_;

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
      trigger_ticks_ = start_ticks_;
      if (trigger_ticks_ == 0) {
        trigger_ticks_ = 1;	// we avoid 0 which is startup case 
      }

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
      if ((delta_ticks >= TIMEOUT_TICKS_) ||
          (delta_ticks < previous_delta_ticks)) {
	// We have timed out or timer has wrapped so we need to shut everything
	// down for this group of sonars.  Visit each sonar and time-out each
	// sonar that does have a value:
        for (UByte schedule_index = first_schedule_index_;
         schedule_index <= last_schedule_index_; schedule_index++) {
	  sonars_[sonars_schedule_[schedule_index]]->time_out();
	}
	// Since we are done, we shut down shut everything down and
	// go to the next sonar group in the sonar schedule:
        //debug_uart_->string_print((Text)"F>");

	state_ = STATE_GROUP_DONE_;
	return;
      }

      // Visit each *sonar_queue*.  The first non-empty sonar queue
      // gets immeidately processed:
      for (UByte queue_index = 0;
       queue_index < sonar_queues_size_; queue_index++) {
	Sonar_Queue *sonar_queue = sonar_queues_[queue_index];
	if (!sonar_queue->is_empty()) {
	  // Process the queue:
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
        state_ = STATE_GROUP_DONE_;
        //debug_uart_->string_print((Text)"H>");
        return;
      }

      // Otherwise, we are still waiting for a sonar to finish up
      // and remain in the same state:
      //debug_uart_->string_print((Text)"H>");
      return;       
      break;
    }
    default: {
      //debug_uart_->string_print((Text)"<Z>");
      // It should be impossible to get here, but just in case, let's force
      // *state_* to be valid for the next time around:
      state_ = STATE_GROUP_DONE_;
      break;
    }
  }
}

void Sonars_Controller::queue_poll(UART *host_uart,
 UInteger time_base, UByte id_offset) {
  for (UByte sonars_index = 0; sonars_index < sonars_size_; sonars_index++) {
    Sonar *sonar = sonars_[sonars_index];
    sonar->queue_poll(host_uart, time_base, id_offset + sonars_index);
  }
}


/// @brief Configure a sonar.
/// @param sonar_index is the identifier for the sonar to be configured.
/// @param sonar_class is one of *CLAS_FRONT*, *CLASS_BACK*, *CLASS_SIDE*,
///        or *CLASS_OFF*.
/// @param left_id is the sonar index for the sonar to the left.
/// @param right_id is the sonar index for the sonar to the right.
///
/// This method will configure the *sonar_index*'th sonar with *sonar_class*,
/// *left_id*, and *right_id*.  The *sonar_class* should be specified as
/// *Sonar::CLASS_OFF* if the sonar is disabled, *Sonar::CLASS_FRONT* if
/// the sonar is on the front of the robot, *Sonar::CLASS_BACK* if the sonar
/// is on the back of the robot, and *Sonar::CLASS_SIDE* if the sonar is on
/// the side of the robot.  *left_id* specify the sonar to the left and
/// right of the *sonar_index*'th id looking out from the robot.  -1 is
/// specified to indicate that there really is not sonar very close to the
/// left or right.
void Sonars_Controller::sonar_configure(UByte sonar_index,
 UByte sonar_class, Byte left_id, Byte right_id) {
  if (sonar_index < sonars_size_) {
    Sonar *sonar = sonars_[sonar_index];
    sonar->configure(sonar_class, left_id, right_id);
  }
}
