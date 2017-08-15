#ifndef COSPHI_SENSOR_H
#define COSPHI_SENSOR_H
#include <Arduino.h>

// utk akses atomik
#include <util/atomic.h>

// pin voltage dan arus
#define PIN_VOLT_COSPHI 2
#define PIN_CURRENT_COSPHI 3
#define COSPHI_0_DEGREE_DIFF_PULSE_LENGTH 10000

// variabel utk cosphi
volatile uint32_t cosphi_t_v_rising = 0, cosphi_t_i_falling = 0,
                  cosphi_t_diff = 0;
volatile uint8_t cosphi_state = 0;
float cosphi_angle = 0.0;

// proto
void isr_v_pulse();
void isr_i_pulse();
void cosphi_setup_pin();
void cosphi_calculate_angle();

// isr utk pulse v
void isr_v_pulse() {
  if (cosphi_state == 0) {
    cosphi_t_v_rising = micros();
    cosphi_state = 1;
  }
}

// isr utk pulse i
void isr_i_pulse() {
  if (cosphi_state == 1) {
    cosphi_t_i_falling = micros();
    cosphi_state = 2;
  }
}

/**
 * setup cosphi sensor pin
 * @method cosphi_setup_pin
 */
void cosphi_setup_pin() {
  pinMode(PIN_VOLT_COSPHI, INPUT_PULLUP);
  pinMode(PIN_CURRENT_COSPHI, INPUT_PULLUP);

  // setup isr
  attachInterrupt(digitalPinToInterrupt(PIN_VOLT_COSPHI), isr_v_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_CURRENT_COSPHI), isr_i_pulse,
                  FALLING);
}

/**
 * get angle and pulse diff
 * @method cosphi_calculate_angle
 */
void cosphi_calculate_angle() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (cosphi_state == 2) {
      if (cosphi_t_i_falling > cosphi_t_v_rising) {
        cosphi_t_diff = cosphi_t_i_falling - cosphi_t_v_rising;
        cosphi_angle =
            (((float)cosphi_t_diff - (float)COSPHI_0_DEGREE_DIFF_PULSE_LENGTH) /
             (float)COSPHI_0_DEGREE_DIFF_PULSE_LENGTH) *
            180.0;
      }
      cosphi_state = 0;
    }
  }
}
#endif
