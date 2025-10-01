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

// change number here
static constexpr gpio_num_t MOTORr_PIN = GPIO_NUM_27;
static constexpr gpio_num_t MOTORl_PIN = GPIO_NUM_26;
static constexpr dshot_mode_t DSHOT_MODE = dshot_mode_t::DSHOT300;
static constexpr auto IS_BIDIRECTIONAL = false;
static constexpr auto MOTORr_MAGNET_COUNT = 14;
static constexpr auto MOTORl_MAGNET_COUNT = 14;
// change number here

DShotRMT motorRight(MOTORr_PIN, DSHOT_MODE, IS_BIDIRECTIONAL, MOTORr_MAGNET_COUNT);
DShotRMT motorLeft(MOTORl_PIN, DSHOT_MODE, IS_BIDIRECTIONAL, MOTORl_MAGNET_COUNT);


void setup() {
  Serial.begin(115200);
	Serial2.begin(420000, SERIAL_8N1, RXD2, TXD2);
	motorRight.begin();
	motorLeft.begin();
}

void loop() { //Choose Serial1 or Serial2 as required
  // Serial.println("looping");
  while (Serial2.available())
	{
		size_t numBytesRead = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
		if (numBytesRead > 0)
		{
			crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS);
			Serial.print("Ch1: ");
			Serial.print(_raw_rc_values[0]);
			Serial.print(" Ch2: ");
			Serial.print(_raw_rc_values[1]);
			Serial.print(" Ch3: ");
			Serial.print(_raw_rc_values[2]);
			Serial.print(" Ch4: ");
			Serial.print(_raw_rc_values[3]);
			Serial.print(" Ch5: ");
			Serial.print(_raw_rc_values[4]);
			Serial.print(" Ch9: ");
			Serial.print(_raw_rc_values[8]);

			// [0] = right speed [1] = left speed [2] = pole length
			float *ctrlValue;
			ctrlValue = car_control(_raw_rc_values[2], _raw_rc_values[0], _raw_rc_values[1], _raw_rc_values[3], _raw_rc_values[8]);
			Serial.print(" Right Speed: ");
			Serial.println(ctrlValue[0]);
			uint16_t RightThrottle = map(ctrlValue[0], -100, 100, 48, 2047);	
			uint16_t LeftThrottle = map(ctrlValue[1], -100, 100, 48, 2047);

			if (_raw_rc_values[4] > 1500) //if armed else stop
			{
			}
			else
			{
			}
			
		}

	}
}