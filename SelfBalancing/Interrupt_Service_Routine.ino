ISR(TIMER2_COMPA_vect) {  // scr7 --. it's gonna be called every 20uS
  // Left
  throttle_counter_left_motor++;
  if (throttle_counter_left_motor > throttle_left_motor_memory) {
    throttle_counter_left_motor = 0;
    throttle_left_motor_memory = throttle_left_motor;
    if (throttle_left_motor_memory < 0) {
      PORTD &= dirLeftBackward;
      throttle_left_motor_memory *= -1;
    } else PORTD |= dirLeftForward;
  } else if (throttle_counter_left_motor == 1) PORTD |= stepLeftHIGH;
  else if (throttle_counter_left_motor == 2) PORTD &= stepLeftLOW;
  // Right
  throttle_counter_right_motor++;
  if (throttle_counter_right_motor > throttle_right_motor_memory) {
    throttle_counter_right_motor = 0;
    throttle_right_motor_memory = throttle_right_motor;
    if (throttle_right_motor_memory < 0) {
      PORTD |= dirRightBackward;
      throttle_right_motor_memory *= -1;
    } else PORTD &= dirRightForward;
  } else if (throttle_counter_right_motor == 1) PORTD |= stepRightHIGH;
  else if (throttle_counter_right_motor == 2) PORTD &= stepRightLOW;
}