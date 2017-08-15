
#include <Arduino.h>

// utk akses atomik
#include <util/atomic.h>

// enum result status
typedef enum cosphi_result_status_en {
  COSPHI_RESULT_STATUS_OK = 0,
  COSPHI_RESULT_STATUS_NO_AC_INPUT = 1,
  COSPHI_RESULT_STATUS_NO_LOAD = 2
} cosphi_result_status_t;

// pin voltage dan arus
#define PIN_VOLT_COSPHI 2
#define PIN_CURRENT_COSPHI 3
#define COSPHI_0_DEGREE_DIFF_PULSE_LENGTH 10000
#define COSPHI_PULSE_DIFF_MAX_TIMEOUT 30000

// variabel utk cosphi
uint8_t cosphi_result_status = COSPHI_RESULT_STATUS_NO_AC_INPUT;
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
    // detection finished?
    if (cosphi_state == 2) {
      cosphi_result_status = COSPHI_RESULT_STATUS_OK;

      if (cosphi_t_i_falling > cosphi_t_v_rising) {
        cosphi_t_diff = cosphi_t_i_falling - cosphi_t_v_rising;
        cosphi_angle =
            (((float)cosphi_t_diff - (float)COSPHI_0_DEGREE_DIFF_PULSE_LENGTH) /
             (float)COSPHI_0_DEGREE_DIFF_PULSE_LENGTH) *
            180.0;
      }

      // reset all value
      cosphi_t_i_falling = cosphi_t_v_rising = 0;
      cosphi_state = 0;
    } else {
      // timeout?
      if (micros() - cosphi_t_v_rising > COSPHI_PULSE_DIFF_MAX_TIMEOUT) {
        if (cosphi_t_v_rising == 0) {
          cosphi_result_status = COSPHI_RESULT_STATUS_NO_AC_INPUT;
        } else {
          if (cosphi_t_i_falling == 0) {
            cosphi_result_status = COSPHI_RESULT_STATUS_NO_LOAD;
          }
        }

        // reset all value
        cosphi_t_i_falling = cosphi_t_v_rising = 0;
        cosphi_state = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  cosphi_setup_pin();
}

void loop() {
  cosphi_calculate_angle();
  // status
  Serial.print(cosphi_result_status);
  Serial.write(9);  //tab
  Serial.print(cosphi_angle, 1);
  Serial.write(176);  //degree sign
  Serial.println();
  delay(100);
}
