#include <driver/rmt_tx.h>
#include <DShotRMT.h>
#include "car_control.h"
#include <Arduino.h>
#include <crsf.h>

#define RXD2 16
#define TXD2 17
#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

int aileronsPin = 12;
int elevatorPin = 13;
int throttlePin = 14; // 123
int rudderPin = 15;

int aileronsPWMChannel = 1;
int elevatorPWMChannel = 2;
int throttlePWMChannel = 3;
int rudderPWMChannel = 4;

// change number here
static constexpr gpio_num_t MOTORr_PIN = GPIO_NUM_27;
static constexpr gpio_num_t MOTORl_PIN = GPIO_NUM_27;
static constexpr dshot_mode_t DSHOT_MODE = dshot_mode_t::DSHOT600;
static constexpr auto IS_BIDIRECTIONAL = true;
static constexpr auto MOTORr_MAGNET_COUNT = 14;
static constexpr auto MOTORl_MAGNET_COUNT = 14;
// change number here

DShotRMT motorRight(MOTORr_PIN, DSHOT_MODE, IS_BIDIRECTIONAL, MOTORr_MAGNET_COUNT);
DShotRMT motorLeft(MOTORl_PIN, DSHOT_MODE, IS_BIDIRECTIONAL, MOTORl_MAGNET_COUNT);

void SetServoPos(float percent, int pwmChannel)
{
    // 50 cycles per second 1,000ms / 50 = 100 /5 = 20ms per cycle
    // 1ms / 20ms = 1/20 duty cycle
    // 2ms / 20ms = 2/20 = 1/10 duty cycle
    // using 16 bit resolution for PWM signal convert to range of 0-65536 (0-100% duty/on time)
    // 1/20th of 65536 = 3276.8
    // 1/10th of 65536 = 6553.6

    uint32_t duty = map(percent, 0, 100, 3276.8, 6553.6);

    ledcWrite(pwmChannel, duty);
}

void setup() {
}

void loop() { //Choose Serial1 or Serial2 as required
  // Serial.println("looping");
  while (Serial2.available()) {
    size_t numBytesRead = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if(numBytesRead > 0)
    {
      crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );
      Serial.print("Channel 1: ");
      Serial.print(_raw_rc_values[0]);
      Serial.print("\tChannel 2: ");
      Serial.print(_raw_rc_values[1]);
      Serial.print("\tChannel 3: ");
      Serial.print(_raw_rc_values[2]);
      Serial.print("\tChannel 4: ");
      Serial.print(_raw_rc_values[3]);
      Serial.print("\tChannel 5: ");
      Serial.println(_raw_rc_values[4]);

      int aileronsMapped = map(_raw_rc_values[0], 1000, 2000, 0, 100);
      int elevatorMapped = map(_raw_rc_values[1], 1000, 2000, 0, 100);
      int throttleMapped = map(_raw_rc_values[2], 1000, 2000, 0, 100);
      int rudderMapped = map(_raw_rc_values[3], 1000, 2000, 0, 100);
      int switchMapped = map(_raw_rc_values[4], 1000, 2000, 0, 100);


      // [0] = right speed [1] = left speed [2] = pole length
      int* ctrlValue;
      ctrlValue = car_control(_raw_rc_values[0], _raw_rc_values[1], _raw_rc_values[2], _raw_rc_values[3], _raw_rc_values[4]);

      if (ctrlValue[0] < 0) {
        motorRight.setMotorSpinDirection(1);
        motorRight.sendThrottlePercent(-ctrlValue[0]);
      } else {
        motorRight.setMotorSpinDirection(0);
        motorRight.sendThrottlePercent(ctrlValue[0]);
      }

      if (ctrlValue[1] < 0) {
        motorLeft.setMotorSpinDirection(1);
        motorLeft.sendThrottlePercent(-ctrlValue[1]);
      } else {
        motorLeft.setMotorSpinDirection(0);
        motorLeft.sendThrottlePercent(ctrlValue[1]);
      }


      SetServoPos(aileronsMapped, aileronsPWMChannel);
      SetServoPos(elevatorMapped, elevatorPWMChannel);
      SetServoPos(throttleMapped, throttlePWMChannel);
      SetServoPos(rudderMapped, rudderPWMChannel);
    }
  }
}