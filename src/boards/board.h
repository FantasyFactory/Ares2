/**
 * @file  board.h
 * @brief Board abstraction interface.
 *
 * Decouples the application layer (main.cpp, AMS engine, API server) from
 * specific hardware choices.  Each board provides a .cpp file that
 * implements the three functions declared here.  main.cpp selects the
 * active board via a compile-time flag (e.g. -DBOARD_ARES_V1).
 *
 * Usage in main.cpp:
 *   ares::board::initBuses();
 *   ares::board::beginAll();
 *   auto& b = ares::board::get();
 *   // use b.gpsDrivers, b.pulse, b.radio, etc.
 */
#pragma once

#include <cstdint>

// Forward-declare Arduino bus types so application headers do not pull in
// the full Arduino SDK.  Board .cpp files include the actual headers.
class TwoWire;
class HardwareSerial;

#include "hal/baro/barometer_interface.h"
#include "hal/gps/gps_interface.h"
#include "hal/imu/imu_interface.h"
#include "hal/led/led_interface.h"
#include "hal/pulse/pulse_interface.h"
#include "hal/radio/radio_interface.h"
#include "hal/servo/servo_interface.h"
#include "ams/ams_driver_registry.h"

// Forward-declare StatusLed (sys layer) — full definition not required here.
class StatusLed;

namespace ares::board {

/**
 * All board-specific driver instances and registries, aggregated for
 * handoff to the application layer.
 *
 * Pointer members that are nullptr indicate hardware not present on
 * the active board.  Application code must null-check before use.
 */
struct BoardDrivers {
    // AMS sensor registries — passed to MissionScriptEngine constructor.
    const ams::GpsEntry*  gpsDrivers;   uint8_t gpsCount;
    const ams::BaroEntry* baroDrivers;  uint8_t baroCount;
    const ams::ComEntry*  comDrivers;   uint8_t comCount;
    const ams::ImuEntry*  imuDrivers;   uint8_t imuCount;

    // Primary sensor instances (index 0 of each type) for ApiServer.
    BarometerInterface* primaryBaro;
    GpsInterface*       primaryGps;
    ImuInterface*       primaryImu;

    // Actuators — either may be nullptr if not present on this board.
    PulseInterface*  pulse;
    ServoInterface*  servo;

    // Interfaces used outside AMS registries.
    RadioInterface*  radio;
    LedInterface*    led;        ///< nullptr if no LED hardware.
    ::StatusLed*     statusLed;  ///< nullptr if no LED hardware.

    // I2C buses (needed by ApiServer for diagnostics and setup).
    TwoWire* wire0;    ///< I2C0 — general sensors (e.g. barometer).
    TwoWire* wire1;    ///< I2C1 — dedicated IMU bus; nullptr if same as wire0.

    // UART buses (needed by ApiServer for diagnostics).
    HardwareSerial* gpsSerial;
    HardwareSerial* radioSerial;
};

/**
 * Initialise all I2C and UART buses.
 * Must be called once at the very start of setup(), before any driver.
 */
void initBuses();

/**
 * Call begin() on every registered sensor, actuator, and radio driver.
 * Must be called after initBuses().
 * Storage (LittleFS) and WiFi are NOT initialised here — they are
 * application-level and remain the responsibility of main.cpp.
 */
void beginAll();

/**
 * Return a reference to the fully-populated BoardDrivers for this board.
 * Valid only after initBuses() and beginAll() have been called.
 */
BoardDrivers& get();

} // namespace ares::board
