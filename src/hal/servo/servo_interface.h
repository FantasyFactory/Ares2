/**
 * @file  servo_interface.h
 * @brief Hardware-agnostic RC servo interface (pure virtual).
 *
 * Abstracts a single PWM-driven RC servo actuator.  Concrete drivers
 * (e.g. ServoDriver) implement this interface; application code (AMS engine,
 * REST API) accesses the servo only through this interface.
 *
 * Thread safety: Implementations are NOT thread-safe.
 *                The AMS engine calls setAngle() inside a mutex-protected
 *                section (CERT-13).
 */
#pragma once

#include <cstdint>

/**
 * Abstract RC servo interface.
 *
 * Provides position control via angle (degrees) or raw pulse width
 * (microseconds).  The AMS engine uses setAngle() to position the servo
 * during on_enter: execution.
 */
class ServoInterface
{
public:
    virtual ~ServoInterface() = default;

    // Non-copyable, non-movable (CERT-18.3)
    ServoInterface(const ServoInterface&)            = delete;
    ServoInterface& operator=(const ServoInterface&) = delete;
    ServoInterface(ServoInterface&&)                 = delete;
    ServoInterface& operator=(ServoInterface&&)      = delete;

    /**
     * Initialise the servo hardware.
     *
     * Attaches the PWM output to the configured pin and drives it to
     * the neutral position (90°).
     *
     * @pre  GPIO subsystem is initialised.
     * @return true on success, false on hardware initialisation failure.
     */
    virtual bool begin() = 0;

    /**
     * Set servo position in degrees.
     *
     * @param[in] degrees  Target angle in the range [0, 180].
     * @pre   begin() returned true.
     * @return true on success, false if begin() was not called or range
     *         is exceeded.
     */
    virtual bool setAngle(uint8_t degrees) = 0;

    /**
     * Set servo position using a raw PWM pulse width.
     *
     * Allows sub-degree precision or use of extended-range servos.
     *
     * @param[in] us  Pulse width in microseconds (typical range 1000–2000).
     * @pre   begin() returned true.
     * @return true on success, false if begin() was not called.
     */
    virtual bool setMicroseconds(uint16_t us) = 0;

    /**
     * Detach the PWM signal and stop driving the servo.
     *
     * After calling disable() the servo holds its last position
     * (mechanically) but draws no current from the PWM driver.
     * begin() must be called again to re-enable.
     *
     * @return true on success.
     */
    virtual bool disable() = 0;

    /**
     * Return the driver model identifier (e.g. "SERVO").
     * Used by the AMS engine to validate 'include <MODEL> as SERVO'.
     * @return Null-terminated model name string (static storage).
     */
    virtual const char* driverModel() const = 0;

protected:
    ServoInterface() = default;  ///< Protected: only subclasses may construct.
};
