/**
 * @file  board_ares_v1.cpp
 * @brief Board implementation — ARES v1 (ESP32-S3 Zero Mini).
 *
 * Implements ares::board::{initBuses, beginAll, get} for the original
 * ARES hardware.  Selected by defining BOARD_ARES_V1 in build flags.
 *
 * Hardware:
 *   GPS     BN-220 (u-blox M8030)     UART1, 9600 baud
 *   IMU     MPU-6050                   I2C1 (0x68)
 *   IMU2    ADXL375 (shock)            I2C1 (0x53)
 *   BARO    BMP280                     I2C0 (0x77)
 *   RADIO   DX-LR03 (LoRa 433 MHz)    UART2, 9600 baud
 *   LED     WS2812 NeoPixel            GPIO 21
 *   PULSE   Drogue/main pyro channels  GPIO 4 / 15
 */
#ifdef BOARD_ARES_V1

#include "boards/board.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "drivers/baro/bmp280_driver.h"
#include "drivers/gps/bn220_driver.h"
#include "drivers/imu/mpu6050_driver.h"
#include "drivers/imu/adxl375_driver.h"
#include "drivers/radio/dxlr03_driver.h"
#include "drivers/pulse/pulse_driver.h"
#include "sys/led/neopixel_driver.h"
#include "sys/led/status_led.h"

// ── Static driver instances (no heap) ──────────────────────────
static HardwareSerial gpsSerial(ares::GPS_UART_PORT);
static HardwareSerial loraSerial(ares::LORA_UART_PORT);
static TwoWire        imuWire(1);
static Bmp280Driver   baro(Wire, ares::BMP280_I2C_ADDR);
static Bn220Driver    gps(gpsSerial, ares::PIN_GPS_RX,
                          ares::PIN_GPS_TX, ares::GPS_BAUD);
static DxLr03Driver   radio(loraSerial, ares::PIN_LORA_TX,
                             ares::PIN_LORA_RX, ares::PIN_LORA_AUX,
                             ares::LORA_UART_BAUD);
static Mpu6050Driver  imu(imuWire, ares::MPU6050_I2C_ADDR);
static Adxl375Driver  imu2(imuWire, ares::ADXL375_I2C_ADDR);
static NeopixelDriver led(ares::PIN_LED_RGB);
static StatusLed       statusLed(led);
static PulseDriver    pulse(ares::PIN_DROGUE, ares::PIN_MAIN);

// ── AMS driver registries ───────────────────────────────────────
static BarometerInterface* const  kBaroIfaces[]  = { &baro };
static GpsInterface* const        kGpsIfaces[]   = { &gps  };
static ImuInterface* const        kImuIfaces[]   = { &imu, &imu2 };

static const ares::ams::GpsEntry  kGpsDrivers[]  = { { "BN220",   kGpsIfaces[0] } };
static const ares::ams::BaroEntry kBaroDrivers[] = { { "BMP280",  kBaroIfaces[0] } };
static const ares::ams::ComEntry  kComDrivers[]  = { { "DXLR03",  &radio } };
static const ares::ams::ImuEntry  kImuDrivers[]  = { { "MPU6050", kImuIfaces[0] },
                                                      { "ADXL375", kImuIfaces[1] } };

// ── BoardDrivers singleton ──────────────────────────────────────
static ares::board::BoardDrivers kDrivers = {
    kGpsDrivers,  1,
    kBaroDrivers, 1,
    kComDrivers,  1,
    kImuDrivers,  2,
    kBaroIfaces[0],
    kGpsIfaces[0],
    kImuIfaces[0],
    &pulse,
    nullptr,        // no servo
    &radio,
    &led,
    &statusLed,
    &Wire,
    &imuWire,
    &gpsSerial,
    &loraSerial,
};

// ── Board interface implementation ─────────────────────────────
namespace ares::board {

void initBuses()
{
    // I2C0 (Wire): shared sensors — BMP280 on GPIO 1/2, 400 kHz fast mode.
    Wire.begin(ares::PIN_I2C_SDA, ares::PIN_I2C_SCL, ares::I2C_FREQ);
    Wire.setTimeOut(ares::I2C_TIMEOUT_MS);
    // I2C1: dedicated IMU bus — MPU-6050/ADXL375 on GPIO 12/13, 50 kHz.
    // GY-521 uses 10 kΩ pull-ups; standard mode is marginal — use 50 kHz.
    imuWire.begin(ares::PIN_IMU_SDA, ares::PIN_IMU_SCL, ares::I2C_FREQ_IMU);
    imuWire.setTimeOut(ares::I2C_TIMEOUT_MS);
}

void beginAll()
{
    for (BarometerInterface* iface : kBaroIfaces) { (void)iface->begin(); }
    for (ImuInterface*       iface : kImuIfaces)  { (void)iface->begin(); }
    for (GpsInterface*       iface : kGpsIfaces)  { (void)iface->begin(); }
    (void)pulse.begin();
    (void)radio.begin();
    (void)led.begin();
    led.setBrightness(ares::DEFAULT_LED_BRIGHTNESS);
    statusLed.begin();  // starts RTOS status-LED task (solid green = IDLE)
}

BoardDrivers& get()
{
    return kDrivers;
}

} // namespace ares::board

#endif // BOARD_ARES_V1
