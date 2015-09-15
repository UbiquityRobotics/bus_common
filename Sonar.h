// Copyright (c) 2015 by Mark Johnston.  All rights reserved.
// Copyright (c) 2015 by Wayne Gramlich.  All rights reserved.

#ifndef SONAR_H_INCLUDED
#define SONAR_H_INCLUDED 1

#include <Bus_Slave.h>

typedef enum {
  class_off,
  class_back,
  class_front,
  class_side,
} Sonar_Class;

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
  Sonar(UByte id, volatile uint8_t *trigger_registers, UByte trigger_mask,
   Sonar_Queue *sonar_queue, UByte pcint_index, UByte echo_mask,
   Sonar_Class sonar_class, Byte left_id, Byte right_id);
  Logical is_close();
  void initialize(UByte sonar_index, UShort *shared_changes_mask);
  UShort mm_distance_get();
  UByte priority_compute(Sonar **sonars, Byte direction);
  void time_out();
  void trigger();
  void trigger_setup();
  void update(UShort ticks, UByte echo_bits, Sonar_Queue *sonar_queue);
  void queue_poll(UART *host_uart, UInteger time_base, UByte id);

  // In-line methods:

  /// @brief Return the pin change mask.
  /// @returns the pin change mask.
  ///
  /// This method will return the pin change mask for the sonar.
  UByte pin_change_mask_get() { return pin_change_mask_; };

  /// @brief Configure a sonar.
  /// @param sonar_class is one of *CLASS_FRONT*, *CLASS_BACK*, *CLASS_SIDE*,
  ///        or *CLASS_OFF*.
  /// @param left_id is the sonar index for the sonar to the left.
  /// @param right_id is the sonar index for the sonar to the right.
  ///
  /// This method will configure the *Sonar* object with *sonar_class*,
  /// *left_id*, and *right_id*.  The *sonar_class* should be specified as
  /// *Sonar::CLASS_OFF* if the sonar is disabled, *Sonar::CLASS_FRONT* if
  /// the sonar is on the front of the robot, *Sonar::CLASS_BACK* if the sonar
  /// is on the back of the robot, and *Sonar::CLASS_SIDE* if the sonar is on
  /// the side of the robot.  *left_id* specify the sonar to the left and
  /// right of the *sonar_index*'th id looking out from the robot.  -1 is
  /// specified to indicate that there really is not sonar very close to the
  /// left or right.
  void configure(Sonar_Class sonar_class, Byte left_id, Byte right_id);

  /// @brief Return the echo mask.
  /// @returns the echo mask.
  ///
  /// This method will return the echo mask for the sonar.
  UByte echo_mask_get() { return echo_mask_; };

  /// @brief Reset the sonar priority to zero.
  ///
  /// This method will reset the sonar priority to zero.
  void priority_reset() { priority_ = 0; };

  /// @brief Set the sonar mask.
  /// @param sonar_mask is the mask for sonar.
  ///
  /// This method will set the sonar mask for this sonar to **sonar_mask**.
  /// This is used for the shared changes mask computation.
  void sonar_mask_set(UShort sonar_mask) { sonar_mask_ = sonar_mask; };

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

  /// @brief Sonar identifier;
  ///
  /// Identifier for this sonar.
  UByte id;

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

  static const UShort SONAR_MEAS_TIMEOUT_START_TICKS_ = 0;
  static const UShort SONAR_MEAS_TIMEOUT_END_TICKS_   = 0xffff;

  // We don't have a mechanism to indicate bad measurements so define special
  // values and the higher level app can deal with these values in app specific
  // ways:
  static const UShort SONAR_MEAS_MAX_RANGE_MM_        = 10000;
  static const UShort SONAR_MEAS_TIMEOUT_MM_          = 10001;

  // These constant are added to the *priority_* field:
  static const UByte PRIORITY_BACKGROUND_ = 1;	// Sonar is not very important
  static const UByte PRIORITY_TRAVEL_DIR_ = 2;	// Sonar is direction of travel
  static const UByte PRIORITY_NEAR_CLOSE_ = 4;	// Next to a close sonar
  static const UByte PRIORITY_CLOSE_ = 8;	// Previous distance was close

  // Private member variables:
  UART *debug_uart_;			// Debugging UART
  UByte state_;				// Sonar state
  UShort echo_end_ticks_;		// Time when echo pulse lowered
  UShort echo_delta_ticks_;		// Total number of ticks for echo pulse
  UShort echo_start_ticks_;		// Time when echo pulse rose
  volatile uint8_t *echo_registers_;	// Base of echo registers
  UByte echo_mask_;			// Mask to use to trigger pin.
  Byte left_id_;			// Index of sonar on left (<0 ==> none)
  UByte pin_change_mask_;		// Mask for PCINT register
  UByte priority_;			// Current priority sum
  Logical queue_available_;		// The current value available for queue
  UInteger queue_time_;			// Time queue value occurred.
  UInteger queue_value_;		// Value for queue response
  Byte right_id_;			// Index of sonar on right (<0 ==> none)
  UShort *shared_changes_mask_;		// Address of shared change mask
  Sonar_Class sonar_class_;		// Class: one of OFF/FRONT/BACK/SIDE
  Sonar_Queue *sonar_queue_;		// Queue for sonar changes
  UShort sonar_mask_;			// 1 << sonar_index
  volatile uint8_t *trigger_registers_;	// trigger registers base
  UByte trigger_mask_;			// Mask to use to trigger pin.
};

class Sonars_Controller {
 public:
  Sonars_Controller(UART *debug_uart,
   Sonar *sonars[], Sonar_Queue *sonar_queues[], UByte sonars_schedule[]);
  void group_advance();
  void group_priorities_reset();
  UByte group_threshold();
  void initialize();
  UByte mask_index_get(UByte sonar_index);
  UShort mm_distance_get(UByte sonar_index);
  void poll();
  void queue_poll(UART *host_uart, UInteger time_base, UByte id_offset);
  void sonar_configure(UByte sonar_index,
   Sonar_Class sonar_class, Byte left_id, Byte right_id);

  // In-line methods:

  /// @brief Return sonar changes mask.
  /// @returns sonar changes mask.
  ///
  /// This method returns a bit mask where each bit that is set to 1
  /// indicates a sonar distance that has changes since the last time
  /// it was read.
  UShort changes_mask_get() { return changes_mask_; };

  /// @brief Return the echo mask associated with a sonar.
  /// @param sonar_index specifies which sonar to use.
  ///
  /// This method returns the echo mask associated with the 
  /// **sonar_index**'th sonar.
  UByte echo_mask_get(UByte sonar_index)
   { return sonars_[sonar_index]->echo_mask_get(); };

  /// @brief Return the pin change mask associated with a sonar.
  /// @param sonar_index specifies which sonar to use.
  ///
  /// This method returns the pin change mask associated with the
  /// **sonar_index**'th sonar.

  UByte pin_change_mask_get(UByte sonar_index)
   { return sonars_[sonar_index]->pin_change_mask_get(); };

  /// @brief Set current robot direction.
  /// @param direction is positive for forward and negative for backward.
  ///
  /// This method will set set the current robot direction.  *direction*
  /// is positive for when the robot is trending forward, negative for
  /// when the robot is trending backwards and zero for when the robot
  /// is turning in place or stopped.
  void direction_set(Byte direction) { direction_ = direction; };

  /// @brief returns the size of the sonars schedule list in bytes.
  ///
  /// This method returns the size of the sonars schedule list.
  UByte sonars_schedule_size_get() { return sonars_schedule_size_; };

  /// @brief returns the number of the sonars schedule groups.
  ///
  /// This method returns the number of groups in the sonars schedule list.
  UByte sonars_schedule_num_groups_get()
    { return sonars_schedule_num_groups_; };

  /// @brief This constant is used to mark the end of group.
  ///
  /// This constant marks the end of sonar group in the sonars schedule
  /// list.
  static const UByte GROUP_END = 250;

  /// @brief This constant is used to mark the end of the sonars schedule.
  ///
  /// This constant marks the end of the sonars schedule list.
  static const UByte SCHEDULE_END = 255;

 private:
  // Constants:
  static const UByte INPUT_ = 0;  // Offset to port input register
  static const UByte DIRECTION_ = 1;  // Offset to data direcection register
  static const UByte OUTPUT_ = 2; // Offset to port output register

  static const UShort TIMEOUT_TICKS_ = 60000;

  // Space the time between sonar groups.  4000 is time for about 2 meter ping
  static const UShort GROUP_SPACING_TICKS_ = 4000;  

  // State values for _
  static const UByte STATE_GROUP_DONE_ = 0;
  static const UByte STATE_GROUP_NEXT_ = 1;
  static const UByte STATE_TRIGGER_SETUP_ = 2;
  static const UByte STATE_TRIGGER_ = 3;
  static const UByte STATE_ECHO_WAIT_ = 4;

  // Member variables:
  UShort changes_mask_;
  UART *debug_uart_;
  Byte direction_;		// >0 => forward and <0 => backward
  UByte first_schedule_index_;
  UByte last_schedule_index_;
  UShort start_ticks_;
  UShort now_ticks_;
  UShort previous_now_ticks_;
  UShort trigger_ticks_;
  UByte pin_change_interrupts_mask_;
  UByte state_;
  Sonar_Queue **sonar_queues_;
  UByte sonar_queues_size_;
  Sonar **sonars_;
  UByte *sonars_schedule_;
  UByte sonars_schedule_size_;
  UByte sonars_schedule_num_groups_;
  UByte sonars_size_;

};

#endif // SONAR_H_INCLUDED
