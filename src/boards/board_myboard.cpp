/**
 * @file  board_myboard.cpp
 * @brief Board implementation — custom hardware (ESP32-S3).
 *
 * Implements ares::board::{initBuses, beginAll, get} for the user's
 * custom board.  Selected by defining BOARD_MYBOARD in build flags.
 *
 * Hardware:
 *   GPS     ATGM336H-5N                UART1, 9600 baud
 *   IMU     MPU-9250 + AK8963 (mag)   I2C0 (0x68)
 *   BARO    BMP280 (on MPU9250 board)  I2C0 (0x76)
 *   RADIO   ASR6601 (LoRa)             UART2, 9600 baud
 *   SERVO   RC servo actuator          GPIO PWM
 *   LED     none
 *   PULSE   none
 *
 * TODO: update pin constants below to match your schematic before
 *       flashing.  All GPIO assignments are placeholders.
 */
#ifdef BOARD_MYBOARD

#include "boards/board.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "drivers/baro/bmp280_driver.h"
#include "drivers/gps/atgm336h_driver.h"
#include "drivers/imu/mpu9250_driver.h"
#include "drivers/radio/asr6601_driver.h"
#include "drivers/servo/servo_driver.h"

// ── Pin assignments — TODO: update for your schematic ──────────
namespace myboard
{
    constexpr uint8_t  PIN_I2C_SDA    = 1;    // TODO: I2C data GPIO
    constexpr uint8_t  PIN_I2C_SCL    = 2;    // TODO: I2C clock GPIO
    constexpr uint8_t  PIN_GPS_RX     = 5;    // TODO: UART1 RX ← GPS TX
    constexpr uint8_t  PIN_GPS_TX     = 6;    // TODO: UART1 TX → GPS RX
    constexpr uint8_t  PIN_LORA_TX    = 7;    // TODO: UART2 TX → ASR6601 RX
    constexpr uint8_t  PIN_LORA_RX    = 8;    // TODO: UART2 RX ← ASR6601 TX
    constexpr uint8_t  PIN_LORA_AUX   = 9;    // TODO: ASR6601 AUX (HIGH = idle)
    constexpr uint8_t  PIN_SERVO      = 4;    // TODO: servo PWM GPIO
    constexpr uint8_t  GPS_UART_PORT  = 1;
    constexpr uint8_t  LORA_UART_PORT = 2;
    // I2C addresses
    constexpr uint8_t  MPU9250_ADDR   = 0x68; // AD0 → GND
    // BMP280 address: 0x76 if SDO→GND, 0x77 if SDO→VCC — check your module
    constexpr uint8_t  BMP280_ADDR    = 0x76; // TODO: verify with your module
    // I2C speed — MPU9250 supports 400 kHz, but check pull-ups
    constexpr uint32_t I2C_FREQ       = 400000;
}

// ── Static driver instances (no heap) ──────────────────────────
static HardwareSerial gpsSerial(myboard::GPS_UART_PORT);
static HardwareSerial loraSerial(myboard::LORA_UART_PORT);

// BMP280 and MPU9250 share the same I2C bus (Wire / I2C0).
static Bmp280Driver    baro(Wire,  myboard::BMP280_ADDR);
static Mpu9250Driver   imu(Wire,   myboard::MPU9250_ADDR, true);

static Atgm336hDriver  gps(gpsSerial,
                            myboard::PIN_GPS_RX,
                            myboard::PIN_GPS_TX,
                            ares::GPS_BAUD);
static Asr6601Driver   radio(loraSerial,
                              myboard::PIN_LORA_TX,
                              myboard::PIN_LORA_RX,
                              myboard::PIN_LORA_AUX,
                              ares::LORA_UART_BAUD);
static ServoDriver     servo(myboard::PIN_SERVO);

// ── AMS driver registries ───────────────────────────────────────
static BarometerInterface* const  kBaroIfaces[]  = { &baro };
static GpsInterface* const        kGpsIfaces[]   = { &gps  };
static ImuInterface* const        kImuIfaces[]   = { &imu  };

static const ares::ams::GpsEntry  kGpsDrivers[]  = { { "ATGM336H", kGpsIfaces[0] } };
static const ares::ams::BaroEntry kBaroDrivers[] = { { "BMP280",   kBaroIfaces[0] } };
static const ares::ams::ComEntry  kComDrivers[]  = { { "ASR6601",  &radio } };
static const ares::ams::ImuEntry  kImuDrivers[]  = { { "MPU9250",  kImuIfaces[0] } };

// ── BoardDrivers singleton ──────────────────────────────────────
static ares::board::BoardDrivers kDrivers = {
    kGpsDrivers,  1,
    kBaroDrivers, 1,
    kComDrivers,  1,
    kImuDrivers,  1,
    kBaroIfaces[0],
    kGpsIfaces[0],
    kImuIfaces[0],
    nullptr,        // no pulse actuator
    &servo,
    &radio,
    nullptr,        // no LED
    nullptr,        // no StatusLed
    &Wire,
    &Wire,          // wire1 == wire0: single I2C bus for baro + IMU
    &gpsSerial,
    &loraSerial,
};

// ── Board interface implementation ─────────────────────────────
namespace ares::board {

void initBuses()
{
    // Single I2C bus for BMP280 + MPU9250 + AK8963.
    Wire.begin(myboard::PIN_I2C_SDA, myboard::PIN_I2C_SCL, myboard::I2C_FREQ);
    Wire.setTimeOut(ares::I2C_TIMEOUT_MS);
}

void beginAll()
{
    for (BarometerInterface* iface : kBaroIfaces) { (void)iface->begin(); }
    for (ImuInterface*       iface : kImuIfaces)  { (void)iface->begin(); }
    for (GpsInterface*       iface : kGpsIfaces)  { (void)iface->begin(); }
    (void)servo.begin();
    (void)radio.begin();
    // No LED, no pulse.
}

BoardDrivers& get()
{
    return kDrivers;
}

} // namespace ares::board

#endif // BOARD_MYBOARD
