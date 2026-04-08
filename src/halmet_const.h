#ifndef HALMET_SRC_HALMET_CONST_H_
#define HALMET_SRC_HALMET_CONST_H_

#include <Arduino.h>

namespace sensesp {

#if defined (BOARD_HALMET)
// I2C pins on HALMET.
const int kSDAPin = 21;
const int kSCLPin = 22;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// OneWire PIN
const int kOneWirePin = 4;

// CAN bus (NMEA 2000) pins on HALMET
const gpio_num_t kCANRxPin = GPIO_NUM_18;
const gpio_num_t kCANTxPin = GPIO_NUM_19;

// HALMET digital input pins
const int kDigitalInputPin1 = GPIO_NUM_23;
const int kDigitalInputPin2 = GPIO_NUM_25;
const int kDigitalInputPin3 = GPIO_NUM_27;
const int kDigitalInputPin4 = GPIO_NUM_26;

#elif defined (BOARD_SAILORHAT)
// I2C pins on SH.
const int kSDAPin = 16;
const int kSCLPin = 17;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// OneWire PIN
const int kOneWirePin = 4;

// CAN bus (NMEA 2000) pins on SH
const gpio_num_t kCANRxPin = GPIO_NUM_34;
const gpio_num_t kCANTxPin = GPIO_NUM_32;

// SH digital input pins
const int kDigitalInputPin1 = GPIO_NUM_15;
const int kDigitalInputPin2 = GPIO_NUM_13;
const int kDigitalInputPin3 = GPIO_NUM_14;
const int kDigitalInputPin4 = GPIO_NUM_12;

#else
#error "No board defined! Please select a PlatformIO environment."
#endif

}  // namespace sensesp

#endif /* HALMET_SRC_HALMET_CONST_H_ */
