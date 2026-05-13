/**
 * @file  atgm336h_driver.h
 * @brief ATGM336H-5N GPS driver with NMEA 0183 sentence parser.
 *
 * Thread safety: NOT thread-safe.  Must be accessed from a single
 *                task or protected externally (CERT-13).
 */
#pragma once

#include <Arduino.h>

#include "hal/gps/gps_interface.h"

/**
 * Concrete GpsInterface for the ATGM336H-5N GNSS module.
 *
 * Parses standard NMEA 0183 sentences over UART.  The module also
 * emits CASIC proprietary sentences ($PCAS*) for configuration
 * feedback; these are silently ignored by the parser.
 *
 * Sentence types consumed:
 *   - **GGA** — position, altitude, fix quality, HDOP, satellites.
 *   - **RMC** — ground speed and course over ground.
 *
 * Talker IDs (GP, GN, GL, GA) are accepted transparently; only the
 * last 3 characters of the sentence type are matched.
 *
 * The driver is poll-based: call update() each loop iteration to
 * drain the hardware UART FIFO, then read() to get the latest fix.
 * If no valid GGA arrives within GPS_TIMEOUT_MS the fix is
 * automatically invalidated.
 */
class Atgm336hDriver : public GpsInterface
{
public:
    /**
     * Construct an ATGM336H driver instance.
     * @param[in] serial  HardwareSerial port connected to the receiver.
     * @param[in] rxPin   ESP32 GPIO pin for UART RX (← GPS TX).
     * @param[in] txPin   ESP32 GPIO pin for UART TX (→ GPS RX).
     * @param[in] baud    UART baud rate (default 9600).
     */
    Atgm336hDriver(HardwareSerial& serial, int8_t rxPin, int8_t txPin, uint32_t baud);

    // Non-copyable, non-movable (CERT-18.3)
    Atgm336hDriver(const Atgm336hDriver&)            = delete;
    Atgm336hDriver& operator=(const Atgm336hDriver&) = delete;

    bool       begin()                    override;
    void       update()                   override;
    GpsStatus  read(GpsReading& out)      override;
    bool       hasFix()            const  override;
    const char* driverModel()      const  override { return "ATGM336H"; }

private:
    void processByte(char c);
    void processSentence();
    bool validateChecksum();
    void parseGGA(char* fields[], uint8_t count);
    void parseRMC(char* fields[], uint8_t count);
    static float   parseCoordinate(const char* raw, const char* hem);
    static float   parseFloat(const char* str);
    static int32_t parseInt(const char* str);

    HardwareSerial& serial_;
    int8_t  rxPin_;
    int8_t  txPin_;
    uint32_t baud_;
    bool    ready_ = false;

    static constexpr uint8_t NMEA_MAX_LEN = 83;
    static constexpr uint8_t MAX_FIELDS   = 20;
    char    sentence_[NMEA_MAX_LEN] = {};
    uint8_t sentenceLen_ = 0;
    bool    inSentence_  = false;

    GpsReading reading_ = {};
    bool       hasFix_  = false;
    uint32_t   lastFixMs_ = 0;

    static constexpr uint32_t GPS_TIMEOUT_MS = 5000;
};
