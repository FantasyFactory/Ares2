/**
 * @file  mpu9250_driver.cpp
 * @brief MPU-9250 IMU + AK8963 magnetometer driver implementation.
 */

#include "drivers/imu/mpu9250_driver.h"
#include "config.h"
#include "debug/ares_log.h"
#include "rtos_guard.h"

#include <cinttypes>
#include <cstring>

static constexpr const char* TAG = "MPU9250";

Mpu9250Driver::Mpu9250Driver(TwoWire& wire, uint8_t addr, bool magEnabledInit)
    : wire_(wire), addr_(addr), magEnabledInit_(magEnabledInit)
{
}

bool Mpu9250Driver::begin()
{
    // CERT-13: create the driver mutex on first begin() call.
    if (imuMutex_ == nullptr)
    {
        imuMutex_ = xSemaphoreCreateMutex();
        if (imuMutex_ == nullptr)
        {
            LOG_E(TAG, "mutex create failed");
            return false;
        }
    }

    // Wake MPU9250 from sleep.
    if (!writeReg(REG_PWR_MGMT_1, 0x00U)) { return false; }
    delay(ares::MPU6050_WAKE_DELAY_MS);

    // Verify WHO_AM_I.
    uint8_t whoami = 0;
    if (!readRegs(REG_WHO_AM_I, &whoami, 1U) || whoami != WHOAMI_EXPECTED)
    {
        LOG_E(TAG, "WHO_AM_I 0x%02X (expected 0x%02X)", whoami, WHOAMI_EXPECTED);
        return false;
    }

    // Sample rate = 1 kHz / (1 + 9) = 100 Hz.
    (void)writeReg(REG_SMPLRT_DIV, 0x09U);
    // DLPF 92 Hz bandwidth.
    (void)writeReg(REG_CONFIG, 0x02U);

    // Enable I2C bypass so AK8963 is accessible on the host I2C bus.
    uint8_t intPinCfg = 0;
    (void)readRegs(REG_INT_PIN_CFG, &intPinCfg, 1U);
    intPinCfg |= 0x02U;  // I2C_BYPASS_EN
    (void)writeReg(REG_INT_PIN_CFG, intPinCfg);
    // Disable MPU9250 I2C master (not needed in bypass mode).
    uint8_t userCtrl = 0;
    (void)readRegs(REG_USER_CTRL, &userCtrl, 1U);
    userCtrl &= ~0x20U;
    (void)writeReg(REG_USER_CTRL, userCtrl);

    ready_ = true;
    consecutiveErrors_ = 0U;
    LOG_I(TAG, "MPU9250 ready (addr=0x%02X)", addr_);

    if (magEnabledInit_)
    {
        magReady_   = initMagnetometer();
        magEnabled_ = magReady_;
        if (!magReady_) { LOG_W(TAG, "AK8963 init failed — mag disabled"); }
    }

    return true;
}

bool Mpu9250Driver::initMagnetometer()
{
    // Verify AK8963 WHO_AM_I.
    uint8_t whoami = 0;
    if (!readMagRegs(AK_WHO_AM_I, &whoami, 1U) || whoami != 0x48U)
    {
        LOG_E(TAG, "AK8963 WHO_AM_I 0x%02X (expected 0x48)", whoami);
        return false;
    }

    // Enter Fuse ROM access mode to read sensitivity adjustment values.
    (void)writeMagReg(AK_CNTL1, 0x0FU);
    delay(1);

    uint8_t asa[3] = {};
    if (!readMagRegs(AK_ASAX, asa, 3U)) { return false; }

    // AK8963 sensitivity adjustment: H_adj = H * (ASA - 128) / 256 + H
    magScaleX_ = (static_cast<float>(asa[0]) - 128.0f) / 256.0f + 1.0f;
    magScaleY_ = (static_cast<float>(asa[1]) - 128.0f) / 256.0f + 1.0f;
    magScaleZ_ = (static_cast<float>(asa[2]) - 128.0f) / 256.0f + 1.0f;

    // Power down before changing mode.
    (void)writeMagReg(AK_CNTL1, 0x00U);
    delay(1);
    // Continuous measurement mode 2: 100 Hz, 16-bit output.
    (void)writeMagReg(AK_CNTL1, 0x16U);
    delay(1);

    LOG_I(TAG, "AK8963 ready, ASA=[%.3f %.3f %.3f]",
          static_cast<double>(magScaleX_),
          static_cast<double>(magScaleY_),
          static_cast<double>(magScaleZ_));
    return true;
}

// ── ImuInterface ─────────────────────────────────────────────

ImuStatus Mpu9250Driver::read(ImuReading& out)
{
    if (imuMutex_ == nullptr) { return ImuStatus::NOT_READY; }
    ScopedLock lk(imuMutex_, pdMS_TO_TICKS(ares::IMU_LOCK_TIMEOUT_MS));
    if (!lk.acquired()) { return ImuStatus::NOT_READY; }
    return readLocked(out);
}

ImuStatus Mpu9250Driver::readLocked(ImuReading& out)
{
    if (!ready_)
    {
        const uint32_t now = millis();
        if (now - lastReinitAttemptMs_ >= ares::IMU_REINIT_INTERVAL_MS)
        {
            lastReinitAttemptMs_ = now;
            ready_ = begin();
        }
        return ImuStatus::NOT_READY;
    }

    uint8_t buf[14] = {};
    if (!readRegs(REG_ACCEL_XOUT_H, buf, 14U))
    {
        if (++consecutiveErrors_ >= ares::IMU_MAX_CONSECUTIVE_ERRORS)
        {
            ready_ = false;
            lastReinitAttemptMs_ = millis();
        }
        return ImuStatus::ERROR;
    }
    consecutiveErrors_ = 0U;

    auto toInt16 = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8U) | lo);
    };

    out.accelX = static_cast<float>(toInt16(buf[0],  buf[1]))  * ACCEL_SCALE;
    out.accelY = static_cast<float>(toInt16(buf[2],  buf[3]))  * ACCEL_SCALE;
    out.accelZ = static_cast<float>(toInt16(buf[4],  buf[5]))  * ACCEL_SCALE;
    out.gyroX  = static_cast<float>(toInt16(buf[8],  buf[9]))  * GYRO_SCALE;
    out.gyroY  = static_cast<float>(toInt16(buf[10], buf[11])) * GYRO_SCALE;
    out.gyroZ  = static_cast<float>(toInt16(buf[12], buf[13])) * GYRO_SCALE;
    // MPU9250 temp formula (datasheet §4.19): T = TEMP_OUT / 333.87 + 21.0
    out.tempC  = static_cast<float>(toInt16(buf[6],  buf[7]))  / 333.87f + 21.0f;

    return ImuStatus::OK;
}

// ── Magnetometer ─────────────────────────────────────────────

bool Mpu9250Driver::isMagnetometerEnabled() const
{
    return magEnabled_ && magReady_;
}

bool Mpu9250Driver::setMagnetometerEnabled(bool enabled)
{
    if (!ready_) { return false; }
    if (enabled && !magReady_) { magReady_ = initMagnetometer(); }
    magEnabled_ = enabled && magReady_;
    return true;
}

ImuStatus Mpu9250Driver::readMag(MagReading& out)
{
    if (imuMutex_ == nullptr) { return ImuStatus::NOT_READY; }
    ScopedLock lk(imuMutex_, pdMS_TO_TICKS(ares::IMU_LOCK_TIMEOUT_MS));
    if (!lk.acquired()) { return ImuStatus::NOT_READY; }
    return readMagLocked(out);
}

ImuStatus Mpu9250Driver::readMagLocked(MagReading& out)
{
    if (!magEnabled_ || !magReady_) { return ImuStatus::ERROR; }

    // Check data-ready bit in ST1.
    uint8_t st1 = 0;
    if (!readMagRegs(AK_ST1, &st1, 1U)) { return ImuStatus::ERROR; }
    if ((st1 & 0x01U) == 0U) { return ImuStatus::NOT_READY; }

    // Read HXL..HZH + ST2 (7 bytes total). Reading ST2 unlatches next sample.
    uint8_t buf[7] = {};
    if (!readMagRegs(AK_HXL, buf, 7U)) { return ImuStatus::ERROR; }

    // HOFL bit in ST2: magnetic sensor overflow.
    if ((buf[6] & 0x08U) != 0U)
    {
        LOG_W(TAG, "AK8963 magnetic overflow");
        return ImuStatus::ERROR;
    }

    // AK8963 output is little-endian.
    auto toInt16le = [](uint8_t lo, uint8_t hi) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8U) | lo);
    };

    out.mx = static_cast<float>(toInt16le(buf[0], buf[1])) * magScaleX_ * MAG_SCALE_16;
    out.my = static_cast<float>(toInt16le(buf[2], buf[3])) * magScaleY_ * MAG_SCALE_16;
    out.mz = static_cast<float>(toInt16le(buf[4], buf[5])) * magScaleZ_ * MAG_SCALE_16;

    return ImuStatus::OK;
}

// ── I2C helpers ──────────────────────────────────────────────

bool Mpu9250Driver::writeReg(uint8_t reg, uint8_t value)
{
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Mpu9250Driver::readRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    wire_.beginTransmission(addr_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) { return false; }
    const uint8_t n = wire_.requestFrom(addr_, len);
    if (n != len) { return false; }
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = static_cast<uint8_t>(wire_.read());
    }
    return true;
}

bool Mpu9250Driver::writeMagReg(uint8_t reg, uint8_t value)
{
    wire_.beginTransmission(AK8963_ADDR);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission() == 0;
}

bool Mpu9250Driver::readMagRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{
    wire_.beginTransmission(AK8963_ADDR);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0) { return false; }
    const uint8_t n = wire_.requestFrom(AK8963_ADDR, len);
    if (n != len) { return false; }
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = static_cast<uint8_t>(wire_.read());
    }
    return true;
}
