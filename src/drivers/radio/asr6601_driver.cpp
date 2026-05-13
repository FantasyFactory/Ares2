/**
 * @file  asr6601_driver.cpp
 * @brief ASR6601 LoRa UART driver implementation.
 *
 * Transparent UART mode — identical behavior to DX-LR03.
 * AUX advisory flow-control follows the same pattern (see dxlr03_driver.cpp).
 */

#include "drivers/radio/asr6601_driver.h"
#include "debug/ares_log.h"

#include <freertos/task.h>
#include <cinttypes>

static constexpr const char* TAG = "ASR6601";

Asr6601Driver::Asr6601Driver(HardwareSerial& serial,
                              int8_t txPin, int8_t rxPin, int8_t auxPin,
                              uint32_t baud)
    : serial_(serial), txPin_(txPin), rxPin_(rxPin), auxPin_(auxPin), baud_(baud)
{
}

bool Asr6601Driver::begin()
{
    pinMode(static_cast<uint8_t>(auxPin_), INPUT_PULLUP);
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);

    auxAvailable_ = waitReady(AUX_TIMEOUT_MS);
    if (!auxAvailable_)
    {
        LOG_W(TAG, "AUX stayed LOW — flow control disabled");
    }

    constexpr uint16_t MAX_FLUSH_BYTES = 512;
    for (uint16_t i = 0; i < MAX_FLUSH_BYTES && serial_.available() > 0; ++i)
    {
        (void)serial_.read();
    }

    ready_ = true;
    LOG_I(TAG, "ASR6601 ready (baud=%" PRIu32 ", AUX=GPIO%d, flow_ctrl=%s)",
          baud_, static_cast<int>(auxPin_), auxAvailable_ ? "on" : "off");
    return true;
}

RadioStatus Asr6601Driver::send(const uint8_t* data, uint16_t len)
{
    if (!ready_)          { return RadioStatus::NOT_READY; }
    if (data == nullptr)  { return RadioStatus::ERROR; }
    if (len == 0)         { return RadioStatus::OK; }
    if (len > MODULE_MTU) { return RadioStatus::OVERFLOW; }

    if (auxAvailable_ && !waitReady(AUX_TIMEOUT_MS))
    {
        auxAvailable_ = false;
        LOG_W(TAG, "AUX timeout during TX — flow control disabled");
    }

    const size_t written = serial_.write(data, len);
    if (written != len)
    {
        LOG_E(TAG, "TX short write: %u/%u",
              static_cast<unsigned>(written), static_cast<unsigned>(len));
        return RadioStatus::ERROR;
    }

    serial_.flush();
    return RadioStatus::OK;
}

RadioStatus Asr6601Driver::receive(uint8_t* buf, uint16_t bufSize,
                                    uint16_t& received)
{
    received = 0;
    if (!ready_)        { return RadioStatus::NOT_READY; }
    if (buf == nullptr) { return RadioStatus::ERROR; }
    if (bufSize == 0)   { return RadioStatus::OK; }

    while (serial_.available() > 0 && received < bufSize)
    {
        const int32_t rxByte = static_cast<int32_t>(serial_.read());
        if (rxByte < 0) { break; }
        buf[received] = static_cast<uint8_t>(rxByte);
        ++received;
    }
    return RadioStatus::OK;
}

bool Asr6601Driver::ready() const
{
    if (!ready_) { return false; }
    if (!auxAvailable_) { return true; }
    return digitalRead(static_cast<uint8_t>(auxPin_)) == HIGH;
}

uint16_t Asr6601Driver::mtu() const { return MODULE_MTU; }

bool Asr6601Driver::waitReady(uint32_t timeoutMs) const
{
    static constexpr uint32_t POLL_PERIOD_MS = 1U;
    static constexpr uint16_t MAX_AUX_POLLS  = 2000U;

    uint16_t pollsLeft = (timeoutMs < MAX_AUX_POLLS)
                         ? static_cast<uint16_t>(timeoutMs)
                         : MAX_AUX_POLLS;

    while (pollsLeft > 0U)
    {
        if (digitalRead(static_cast<uint8_t>(auxPin_)) != LOW) { return true; }
        pollsLeft--;
        if (pollsLeft == 0U) { return false; }
        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
    return false;
}
