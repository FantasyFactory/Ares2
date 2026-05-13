/**
 * @file  asr6601_driver.h
 * @brief ASR6601 LoRa UART transceiver driver.
 *
 * Drives an ASR6601-based LoRa module in NORMAL (transparent) mode
 * via UART.  Interface is identical to DX-LR03 — both modules use
 * UART transparent mode with an AUX busy indicator.
 *
 * AUX pin handling mirrors DxLr03Driver: probed during begin(), falls
 * back to unconditional TX if AUX is permanently LOW.
 *
 * Thread safety: NOT thread-safe.  Must be accessed from the comms
 *                task only (CERT-13).
 */
#pragma once

#include <Arduino.h>
#include "config.h"

#include "hal/radio/radio_interface.h"

/**
 * Concrete RadioInterface for ASR6601-based LoRa modules.
 *
 * Operates in NORMAL (transparent) mode:
 *   - Bytes written to UART are immediately transmitted over-air.
 *   - Received over-air bytes appear in the UART RX FIFO.
 *   - AUX goes LOW during TX and returns HIGH when complete.
 */
class Asr6601Driver : public RadioInterface
{
public:
    /**
     * Construct an ASR6601 LoRa driver instance.
     * @param[in] serial  HardwareSerial port — not owned.
     * @param[in] txPin   ESP32 GPIO for UART TX (→ module RX).
     * @param[in] rxPin   ESP32 GPIO for UART RX (← module TX).
     * @param[in] auxPin  ESP32 GPIO for AUX (input, HIGH = idle).
     * @param[in] baud    UART baud rate (default 9600).
     */
    Asr6601Driver(HardwareSerial& serial,
                  int8_t txPin, int8_t rxPin, int8_t auxPin,
                  uint32_t baud);

    // Non-copyable, non-movable (CERT-18.3)
    Asr6601Driver(const Asr6601Driver&)            = delete;
    Asr6601Driver& operator=(const Asr6601Driver&) = delete;

    bool         begin()                                                     override;
    RadioStatus  send(const uint8_t* data, uint16_t len)                    override;
    RadioStatus  receive(uint8_t* buf, uint16_t bufSize,
                         uint16_t& received)                                 override;
    bool         ready()    const                                            override;
    uint16_t     mtu()      const                                            override;
    const char*  driverModel() const override { return "ASR6601"; }

private:
    bool waitReady(uint32_t timeoutMs) const;

    HardwareSerial& serial_;
    int8_t   txPin_;
    int8_t   rxPin_;
    int8_t   auxPin_;
    uint32_t baud_;
    bool     ready_        = false;
    bool     auxAvailable_ = false;

    static constexpr uint16_t MODULE_MTU    = 240;
    static constexpr uint32_t AUX_TIMEOUT_MS = ares::LORA_AUX_TIMEOUT_MS;
};
