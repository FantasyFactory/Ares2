/**
 * @file  mpu9250_driver.h
 * @brief MPU-9250 I2C IMU driver with optional AK8963 magnetometer.
 *
 * Implements ImuInterface for the InvenSense MPU-9250 (accel + gyro +
 * integrated AK8963 magnetometer).
 *
 * Magnetometer access uses I2C bypass mode: after begin() the MPU9250's
 * internal I2C master is disabled and the AK8963 is accessible directly
 * on the host I2C bus at address 0x0C.
 *
 * Magnetometer support is optional at runtime:
 *   - hasMagnetometer()         → always true
 *   - setMagnetometerEnabled()  → enable/disable AK8963
 *   - readMag()                 → read AK8963 when enabled
 *
 * Callers that do not use the magnetometer need not change — the default
 * ImuInterface methods return false/ERROR for non-override methods,
 * providing full retrocompatibility.
 *
 * Thread safety: thread-safe.  An internal FreeRTOS mutex serialises
 *                concurrent read() and readMag() calls (CERT-13).
 */
#pragma once

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "hal/imu/imu_interface.h"

/**
 * Concrete ImuInterface for the InvenSense MPU-9250.
 *
 * Init sequence (begin()):
 *   1. Wake MPU9250: PWR_MGMT_1 (0x6B) = 0x00.
 *   2. Verify WHO_AM_I (0x75) == 0x71.
 *   3. Enable I2C bypass: INT_PIN_CFG (0x37) bit 1 = 1.
 *   4. If magnetometer enabled: init AK8963, read sensitivity ADC.
 *
 * Reading (read()):
 *   Burst-read 14 bytes from ACCEL_XOUT_H (0x3B) — identical to MPU6050.
 *   Conversion:
 *     Accel: raw / 2048.0 * 9.80665  (±16 g full scale, m/s²)
 *     Gyro : raw / 16.4               (±2000 deg/s full scale)
 *     Temp : raw / 333.87 + 21.0      (MPU9250 datasheet formula)
 *
 * Reading (readMag()):
 *   Poll AK8963 ST1 for DRDY, burst-read 7 bytes (HX–HZ + ST2).
 *   Conversion: raw * (ASA/256 + 0.5) * 4912/32760  (µT, 16-bit mode)
 */
class Mpu9250Driver : public ImuInterface
{
public:
    /**
     * Construct an MPU-9250 driver instance.
     * @param[in] wire           I2C bus (must be initialised before begin()).
     * @param[in] addr           7-bit I2C address (0x68 if AD0→GND).
     * @param[in] magEnabledInit true to enable magnetometer in begin().
     */
    explicit Mpu9250Driver(TwoWire& wire,
                           uint8_t  addr           = 0x68U,
                           bool     magEnabledInit  = true);

    // Non-copyable, non-movable (CERT-18.3)
    Mpu9250Driver(const Mpu9250Driver&)            = delete;
    Mpu9250Driver& operator=(const Mpu9250Driver&) = delete;

    /**
     * Initialise MPU-9250 and (optionally) the AK8963 magnetometer.
     * @return true on success; false if WHO_AM_I check fails.
     */
    bool begin() override;

    /**
     * Read accelerometer, gyroscope, and die temperature.
     * @param[out] out  Populated on ImuStatus::OK.
     * @return Status code (OK, ERROR, or NOT_READY).
     */
    ImuStatus read(ImuReading& out) override;
    const char* driverModel() const override { return "MPU9250"; }

    // ── Magnetometer overrides ──────────────────────────────
    bool       hasMagnetometer()                    const override { return true; }
    bool       isMagnetometerEnabled()              const override;
    bool       setMagnetometerEnabled(bool enabled)       override;

    /**
     * Read the latest AK8963 magnetometer sample.
     * @param[out] out  Populated on ImuStatus::OK.
     * @return ImuStatus::OK on success; ImuStatus::ERROR if disabled or fault.
     * @pre isMagnetometerEnabled() == true
     */
    ImuStatus  readMag(MagReading& out) override;

private:
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    bool writeMagReg(uint8_t reg, uint8_t value);
    bool readMagRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    bool initMagnetometer();
    ImuStatus readLocked(ImuReading& out);
    ImuStatus readMagLocked(MagReading& out);

    TwoWire& wire_;
    uint8_t  addr_;               ///< MPU9250 I2C address.
    bool     ready_           = false;
    bool     magEnabled_      = false;
    bool     magReady_        = false;
    bool     magEnabledInit_  = true;

    // AK8963 sensitivity adjustment values read from fuse ROM.
    float    magScaleX_ = 1.0f;
    float    magScaleY_ = 1.0f;
    float    magScaleZ_ = 1.0f;

    uint32_t lastReinitAttemptMs_    = 0U;
    uint8_t  consecutiveErrors_      = 0U;
    SemaphoreHandle_t imuMutex_      = nullptr;

    // MPU9250 register map (subset used by this driver)
    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG        = 0x1A;
    static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
    static constexpr uint8_t REG_INT_PIN_CFG   = 0x37;
    static constexpr uint8_t REG_USER_CTRL     = 0x6A;
    static constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I      = 0x75;
    static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;  ///< Burst read start.
    static constexpr uint8_t WHOAMI_EXPECTED    = 0x71;  ///< MPU9250 WHO_AM_I.
    static constexpr uint8_t AK8963_ADDR        = 0x0C;  ///< Magnetometer I2C addr.

    // AK8963 register map
    static constexpr uint8_t AK_WHO_AM_I = 0x00;  ///< Should return 0x48.
    static constexpr uint8_t AK_ST1      = 0x02;  ///< Status 1 (bit0 = DRDY).
    static constexpr uint8_t AK_HXL      = 0x03;  ///< Measurement data start.
    static constexpr uint8_t AK_CNTL1   = 0x0A;  ///< Control: mode.
    static constexpr uint8_t AK_ASAX    = 0x10;  ///< Sensitivity adj X (fuse ROM).

    // Conversion constants
    static constexpr float ACCEL_SCALE  = 9.80665f / 2048.0f;  ///< ±16 g → m/s²
    static constexpr float GYRO_SCALE   = 1.0f / 16.4f;        ///< ±2000 deg/s
    static constexpr float MAG_SCALE_16 = 4912.0f / 32760.0f;  ///< 16-bit µT/LSB
};
