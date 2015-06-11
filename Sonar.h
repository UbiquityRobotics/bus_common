// Copyright (c) 2015 by Mark Johnston.  All rights reserved.
// Copyright (c) 2015 by Wayne Gramlich.  All rights reserved.

#ifndef SONAR_H_INCLUDED
#define SONAR_H_INCLUDED 1

#include <Bus_Slave.h>

class Sonar_Queue {
 public:
  // Methods define in Sonar.cpp:
  Sonar_Queue(UByte mask_index,
   volatile uint8_t *echo_registers, UART *debug_uart);
  void input_direction_set(UByte echo_mask);
  void interrupt_service_routine();
  void shut_down();

  // In-line methods:

  /// @brief Advance queue to next available pair:
  ///
  /// This method will "consume" the current (echo_bits, ticks) pair
  /// and advance the queue to the next one.  This method should only
  /// be called if **is_empty**() returns false.
  void consume_one() { consumer_index_ = (consumer_index_ + 1) & QUEUE_MASK_; };

  /// @brief Return the debug uart.
  /// @return the debug uart.
  ///
  /// This method returns the debug uart object that was passed into
  /// the constructor.
  UART *debug_uart_get() {return debug_uart_; } ;

  /// @brief Return the current echo bits from the echo I/O port.
  /// @return the current echo bits from the echo I/O port.
  ///
  /// This method will return the current echo I/O port bit values.
  UByte echo_bits_get() {return echo_registers_[INPUT_]; };

  /// @brief Return the echo bits from the queue without consuming them.
  /// @return the echo bits from the queue without consuming them.
  ///
  /// This method will return the echo bits from the queue without
  /// consuming them.  This method should only be called if the
  /// **is_empty**() returns false.
  UByte echos_peek() {return echos_[consumer_index_]; };

  /// @brief Return true if queue is empty.
  /// @return true if queue is empty.
  ///
  /// This method will return true if the queue is empty.
  Logical is_empty() {return producer_index_ == consumer_index_; };

  /// @brief Return the mask index.
  /// @return the mask index.
  ///
  /// This method will return the mask index that was passed into the
  /// constructor.
  UByte mask_index_get() { return mask_index_; };

  /// @brief Enable a change mask bit.
  /// @param change_mask adds bits to track for pin changes.
  ///
  /// This method will add **change_mask** to pins tracked for pin changes.
  void pin_change_mask_set(UByte change_mask)
   { *change_mask_register_ |= change_mask; };

  /// @brief Return the ticks times stamp from the queue with consuming it.
  /// @return the ticks times stamp from the queue with consuming it.
  ///
  /// This method will return the ticks time stamp bits from the queue
  /// without consuming them.  This method should only be called if  the
  /// **is_empty**() method returns false.
  UShort ticks_peek() {return ticks_[consumer_index_]; };

 private:
  // Register offsets for Input, Direction, and Output I/O registers:
  static const UByte INPUT_ = 0;
  static const UByte DIRECTION_ = 1;
  static const UByte OUTPUT_ = 2;
  // Queue constants:
  static const UByte QUEUE_POWER_ = 4;
  static const UByte QUEUE_SIZE_ = 1 << QUEUE_POWER_;
  static const UByte QUEUE_MASK_ = QUEUE_SIZE_ - 1;

  UByte consumer_index_;
  volatile uint8_t *change_mask_register_;
  UByte echos_[QUEUE_SIZE_];
  UART *debug_uart_;
  volatile uint8_t *echo_registers_;
  UByte interrupt_mask_;
  UByte mask_index_;
  UByte producer_index_;
  UShort ticks_[QUEUE_SIZE_];
};

// Each instance of a *Sonar* class object represents a single
// HC-SR04 sonar object.
class Sonar {
 public:
  // Public constructors and member functions:
  Sonar(volatile uint8_t *trigger_registers, UByte trigger_mask,
   Sonar_Queue *sonar_queue, UByte change_bit, UByte echo_mask);
  void initialize();
  UShort mm_distance_get();
  void time_out();
  void trigger();
  void trigger_setup();
  void update(UShort ticks, UByte echo_bits, Sonar_Queue *sonar_queue);

  // In-line methods:

  /// @brief Return the pin change mask.
  /// @returns the pin change mask.
  ///
  /// This method will return the pin change mask for the sonar.
  UByte pin_change_mask_get() { return pin_change_mask_; };

  /// @brief return the echo mask.
  /// @returns the echo mask.
  ///
  /// This method will return the echo mask for the sonar.
  UByte echo_mask_get() { return echo_mask_; };

  /// @brief Return the sonar queue associated with the sonar.
  /// @returns the sonar queue associated with the sonar.
  ///
  /// This method will return the `sonar_queue` associated with the sonar.
  Sonar_Queue *sonar_queue_get() { return sonar_queue_; };

  /// @brief Return when the sonar measurement is done.
  /// @returns 1 if the sonar measurement is done and 0 otherwise.
  ///
  /// This method will return true (i.e. 1) if the sonar is no longer
  /// waiting for an echo pulse to finish.
  Logical is_done() {return (Logical)((state_ == STATE_OFF_) ? 1 : 0); };

 private:
  // Register access offsets:
  static const UByte INPUT_ = 0;          // Offset to port input register
  static const UByte DIRECTION_ = 1;      // Offset to data direcction reg.
  static const UByte OUTPUT_ = 2;         // Offset to port output register
  // Sonar states:
  static const UByte STATE_OFF_ = 0;
  static const UByte STATE_ECHO_RISE_WAIT_ = 1;
  static const UByte STATE_ECHO_FALL_WAIT_ = 2;
  static const UByte TRIGGER_TICKS_ = 4;  // Number of ticks for trigger pulse

  // Private member variables:
  UART *debug_uart_;			// Debugging UART
  UByte state_;				// Sonar state
  UShort echo_start_ticks_;		// Time when echo pulse rose
  UShort echo_end_ticks_;		// Time when echo pulse lowered
  volatile uint8_t *echo_base_;		// Base address of echo registers
  UByte echo_mask_;			// Mask to use to trigger pin.
  UByte pin_change_mask_;		// Mask for PCINT register
  Sonar_Queue *sonar_queue_;		// Queue for sonar changes
  volatile uint8_t *trigger_registers_;	// trigger registers base
  UByte trigger_mask_;			// Mask to use to trigger pin.
};

class Sonars_Controller {
 public:
  Sonars_Controller(UART *debug_uart,
   Sonar *sonars[], Sonar_Queue *sonar_queues[], UByte sonars_schedule[]);
  unsigned long measurement_trigger(UByte sonar_index);
  UByte pin_change_mask_get(UByte sonar_index);
  UByte echo_mask_get(UByte sonar_index);
  void initialize();
  UByte mask_index_get(UByte sonar_index);
  UShort mm_distance_get(UByte sonar_index);
  void sonar_queues_reset();
  void poll();
  UByte sonars_schedule_size_get() {return sonars_schedule_size_; };

  static const UByte GROUP_END = 250;
  static const UByte SCHEDULE_END = 255;

 private:
  // Constants:
  static const UByte INPUT_ = 0;  // Offset to port input register
  static const UByte DIRECTION_ = 1;  // Offset to data direcection register
  static const UByte OUTPUT_ = 2; // Offset to port output register

  static const UShort TIMEOUT_TICKS_ = 60000;

  // State values for _
  static const UByte STATE_SHUT_DOWN_ = 0;
  static const UByte STATE_GROUP_NEXT_ = 1;
  static const UByte STATE_TRIGGER_SETUP_ = 2;
  static const UByte STATE_TRIGGER_ = 3;
  static const UByte STATE_ECHO_WAIT_ = 4;

  // Member variables:
  UART *debug_uart_;
  UByte first_schedule_index_;
  UByte last_schedule_index_;
  UShort now_ticks_;
  UByte pin_change_interrupts_mask_;
  UShort previous_now_ticks_;
  UByte state_;
  UShort start_ticks_;
  Sonar_Queue **sonar_queues_;
  UByte sonar_queues_size_;
  Sonar **sonars_;
  UByte *sonars_schedule_;
  UByte sonars_schedule_size_;
  UByte sonars_size_;

};

#endif // SONAR_H_INCLUDED
