#include <cstdint>

#if !defined(RACECHRONO_BIY_BLE_DEVICE_CONFIG_H)
#define RACECHRONO_BIY_BLE_DEVICE_CONFIG_H

// Change the value to customize the name of your device.
#define DEVICE_NAME "BLE CAN device demo"

// We use RaceChronoPidMap to keep track of stuff for each CAN ID.
// In this implementation, we're going to ignore "update intervals" requested by
// RaceChrono, and instead send every Nth CAN message we receive, per CAN ID, where
// N is different for different PIDs.
const uint8_t DEFAULT_UPDATE_RATE_DIVIDER = 1;

// You need to pick one of the provided configurations below, or define your own
// configuration. Your configuration should define the following constant and
// function:

// Defines the baud rate to use for the CAN bus. For example, 500k baud rate
// should be written as 500 * 1e3.
extern const long BAUD_RATE = 1000 * 1e3;  // 500k.

// Returns an "update rate divider for a given CAN ID.
// If the value is N, only every Nth message received from the CAN bus will be
// communicated to RaceChrono via BLE.
uint8_t getUpdateRateDivider(uint32_t can_id){
    return DEFAULT_UPDATE_RATE_DIVIDER;
}


// Here are some configurations you can pick from by uncommenting one of the
// following lines:
//#include "configs/ft86_gen1.h"
//#include "configs/ft86_gen2.h"

// Once you read all the comments and address what they asked of you, please
// uncomment the nest line:
#define I_READ_THE_COMMENTS

#if !defined(I_READ_THE_COMMENTS)
#error Please open config.h and read all the comments!
#endif // I_READ_THE_COMMENTS

#endif // RACECHRONO_BIY_BLE_DEVICE_CONFIG_H

#define PIN_5V_EN 16

#define CAN_TX_PIN 26
#define CAN_RX_PIN 27
#define CAN_SE_PIN 23

#define RS485_EN_PIN 17 // 17 /RE
#define RS485_TX_PIN 22 // 21
#define RS485_RX_PIN 21 // 22
#define RS485_SE_PIN 19 // 22 /SHDN

#define SD_MISO_PIN 2
#define SD_MOSI_PIN 15
#define SD_SCLK_PIN 14
#define SD_CS_PIN 13

#define WS2812_PIN 4