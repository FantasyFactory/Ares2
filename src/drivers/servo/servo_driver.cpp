/**
 * @file  servo_driver.cpp
 * @brief ESP32 PWM RC servo driver implementation.
 *
 * Uses the ESP32Servo library (LEDC-based PWM).
 * Add to platformio.ini: lib_deps = madhephaestus/ESP32Servo
 */

#include "drivers/servo/servo_driver.h"

#include <ESP32Servo.h>
#include <new>

// Servo object is placement-new'd into servoStorage_ to avoid pulling
// ESP32Servo.h into the interface header.
static_assert(sizeof(Servo) <= 96,
              "ServoDriver::servoStorage_ too small — increase array size");

static inline Servo* srv(uint8_t* storage)
{
    return reinterpret_cast<Servo*>(storage);
}

ServoDriver::ServoDriver(uint8_t pin, uint16_t minUs, uint16_t maxUs)
    : pin_(pin), minUs_(minUs), maxUs_(maxUs)
{
    new (servoStorage_) Servo();
}

bool ServoDriver::begin()
{
    srv(servoStorage_)->attach(pin_, static_cast<int>(minUs_),
                               static_cast<int>(maxUs_));
    attached_ = true;
    srv(servoStorage_)->write(90);  // neutral position
    return true;
}

bool ServoDriver::setAngle(uint8_t degrees)
{
    if (!attached_) { return false; }
    if (degrees > 180U) { degrees = 180U; }
    srv(servoStorage_)->write(static_cast<int>(degrees));
    return true;
}

bool ServoDriver::setMicroseconds(uint16_t us)
{
    if (!attached_) { return false; }
    srv(servoStorage_)->writeMicroseconds(static_cast<int>(us));
    return true;
}

bool ServoDriver::disable()
{
    if (attached_)
    {
        srv(servoStorage_)->detach();
        attached_ = false;
    }
    return true;
}
