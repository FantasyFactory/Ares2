/**
 * @file  servo_driver.h
 * @brief ESP32 PWM RC servo driver.
 *
 * Drives a standard RC servo using the ESP32Servo library (50 Hz PWM,
 * 1000–2000 µs pulse range).
 *
 * Thread safety: NOT thread-safe.  Must be accessed from a single
 *                task or protected externally (CERT-13).
 */
#pragma once

#include <cstdint>

#include "hal/servo/servo_interface.h"

/**
 * Concrete ServoInterface for a standard RC servo on ESP32.
 *
 * Uses the ESP32Servo library which maps onto the LEDC PWM hardware.
 * Default timing: 50 Hz, 1000 µs (0°) to 2000 µs (180°).
 */
class ServoDriver : public ServoInterface
{
public:
    /**
     * Construct a servo driver instance.
     * @param[in] pin    GPIO pin connected to the servo signal wire.
     * @param[in] minUs  Minimum pulse width in µs (default 1000).
     * @param[in] maxUs  Maximum pulse width in µs (default 2000).
     */
    explicit ServoDriver(uint8_t pin,
                         uint16_t minUs = 1000U,
                         uint16_t maxUs = 2000U);

    // Non-copyable, non-movable (CERT-18.3)
    ServoDriver(const ServoDriver&)            = delete;
    ServoDriver& operator=(const ServoDriver&) = delete;

    /**
     * Attach the servo to its GPIO pin and drive to neutral (90°).
     * @return true on success.
     */
    bool begin() override;

    /**
     * Set servo angle.
     * @param[in] degrees  Target angle [0, 180].
     * @return true on success; false if begin() was not called.
     */
    bool setAngle(uint8_t degrees) override;

    /**
     * Set servo position via raw pulse width.
     * @param[in] us  Pulse width in µs [minUs, maxUs].
     * @return true on success; false if begin() was not called.
     */
    bool setMicroseconds(uint16_t us) override;

    /**
     * Detach the servo PWM signal.
     * @return true on success.
     */
    bool disable() override;

    const char* driverModel() const override { return "SERVO"; }

private:
    uint8_t  pin_;
    uint16_t minUs_;
    uint16_t maxUs_;
    bool     attached_ = false;

    // ESP32Servo Servo object — included in .cpp to keep Arduino headers
    // out of this interface header.
    struct Impl;
    // Storage for the Servo object (avoids heap allocation).
    // alignas(4): Servo is typically 4-byte aligned.
    alignas(4) uint8_t servoStorage_[96];  // sizeof(Servo) == 88 on ESP32Servo 3.x
};
