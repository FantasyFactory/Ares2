/**
 * @file  atgm336h_driver.cpp
 * @brief ATGM336H-5N GPS driver implementation (NMEA 0183 parser).
 *
 * Parsing logic is identical to bn220_driver.cpp — both modules emit
 * standard NMEA 0183 GGA and RMC sentences.  CASIC proprietary
 * sentences ($PCAS*) are silently discarded by the sentence dispatcher.
 */

#include "drivers/gps/atgm336h_driver.h"
#include "config.h"

#include <climits>
#include <cmath>
#include <cstring>
#include <cstdlib>

namespace nmea_atgm
{
    constexpr float   KNOTS_TO_KMH    = 1.852f;
    constexpr float   DEG_DIVISOR     = 100.0f;
    constexpr float   MINUTES_PER_DEG = 60.0f;
    constexpr uint8_t MIN_GGA_FIELDS  = 15;
    constexpr uint8_t MIN_RMC_FIELDS  = 12;
    constexpr uint8_t MIN_TYPE_LEN    = 3;
    constexpr uint8_t MIN_SATS_3D     = 4;
}

Atgm336hDriver::Atgm336hDriver(HardwareSerial& serial,
                                int8_t rxPin, int8_t txPin, uint32_t baud)
    : serial_(serial), rxPin_(rxPin), txPin_(txPin), baud_(baud)
{
}

bool Atgm336hDriver::begin()
{
    serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
    ready_ = true;
    return true;
}

void Atgm336hDriver::update()
{
    if (!ready_) { return; }

    uint16_t bytesRead = 0;
    while (serial_.available() > 0 && bytesRead < ares::MAX_SERIAL_READ)
    {
        char c = static_cast<char>(serial_.read());
        processByte(c);
        bytesRead++;
    }
}

GpsStatus Atgm336hDriver::read(GpsReading& out)
{
    if (!ready_) { return GpsStatus::NOT_READY; }

    const uint32_t now = millis();
    if (hasFix_ && (now - lastFixMs_ > GPS_TIMEOUT_MS))
    {
        hasFix_ = false;
        reading_.fixType = GpsFixType::NONE;
    }

    out = reading_;

    if (!hasFix_)
    {
        return (lastFixMs_ != 0 && (now - lastFixMs_ > GPS_TIMEOUT_MS))
               ? GpsStatus::TIMEOUT
               : GpsStatus::NO_FIX;
    }
    return GpsStatus::OK;
}

bool Atgm336hDriver::hasFix() const { return hasFix_; }

// ── NMEA state machine ────────────────────────────────────────

void Atgm336hDriver::processByte(char c)
{
    if (c == '$') { sentenceLen_ = 0; inSentence_ = true; return; }
    if (!inSentence_) { return; }
    if (c == '\r' || c == '\n')
    {
        if (sentenceLen_ > 0) { sentence_[sentenceLen_] = '\0'; processSentence(); }
        inSentence_ = false;
        return;
    }
    if (sentenceLen_ < NMEA_MAX_LEN - 1) { sentence_[sentenceLen_++] = c; }
    else { inSentence_ = false; }
}

void Atgm336hDriver::processSentence()
{
    if (!validateChecksum()) { return; }

    char* star = strchr(sentence_, '*');
    if (star != nullptr) { *star = '\0'; }

    char* fields[MAX_FIELDS] = {};
    uint8_t fieldCount = 0;
    char* ptr = sentence_;
    while (fieldCount < MAX_FIELDS)
    {
        fields[fieldCount++] = ptr;
        ptr = strchr(ptr, ',');
        if (ptr == nullptr) { break; }
        *ptr = '\0'; ptr++;
    }

    if (fieldCount < 2) { return; }

    const char* type = fields[0];
    size_t len = strnlen(type, NMEA_MAX_LEN);
    if (len < nmea_atgm::MIN_TYPE_LEN) { return; }

    const char* suffix = type + len - nmea_atgm::MIN_TYPE_LEN;
    if (strcmp(suffix, "GGA") == 0 && fieldCount >= nmea_atgm::MIN_GGA_FIELDS)
        { parseGGA(fields, fieldCount); }
    else if (strcmp(suffix, "RMC") == 0 && fieldCount >= nmea_atgm::MIN_RMC_FIELDS)
        { parseRMC(fields, fieldCount); }
    // CASIC ($PCAS*) and other proprietary sentences are silently ignored.
}

bool Atgm336hDriver::validateChecksum()
{
    const char* star = strchr(sentence_, '*');
    if (star == nullptr) { return false; }
    ptrdiff_t starIdx = star - sentence_;
    if (starIdx + 2 >= static_cast<ptrdiff_t>(sentenceLen_)) { return false; }
    uint8_t computed = 0;
    for (const char* p = sentence_; p < star; p++) { computed ^= static_cast<uint8_t>(*p); }
    const char hexStr[3] = { star[1], star[2], '\0' };
    return computed == static_cast<uint8_t>(strtoul(hexStr, nullptr, 16));
}

void Atgm336hDriver::parseGGA(char* fields[], uint8_t count)
{
    (void)count;
    if (parseInt(fields[6]) == 0)
    {
        reading_.fixType = GpsFixType::NONE; hasFix_ = false; return;
    }
    reading_.latitude  = parseCoordinate(fields[2], fields[3]);
    reading_.longitude = parseCoordinate(fields[4], fields[5]);
    if (reading_.latitude  < -90.0f  || reading_.latitude  > 90.0f  ||
        reading_.longitude < -180.0f || reading_.longitude > 180.0f)
    {
        reading_.fixType = GpsFixType::NONE; hasFix_ = false; return;
    }
    int32_t satRaw = parseInt(fields[7]);
    if (satRaw < 0) { satRaw = 0; }
    if (satRaw > UINT8_MAX) { satRaw = UINT8_MAX; }
    reading_.satellites = static_cast<uint8_t>(satRaw);
    reading_.hdop       = parseFloat(fields[8]);
    reading_.altitudeM  = parseFloat(fields[9]);
    reading_.fixType    = (reading_.satellites >= nmea_atgm::MIN_SATS_3D)
                        ? GpsFixType::FIX_3D : GpsFixType::FIX_2D;
    reading_.timestampMs = millis();
    lastFixMs_ = reading_.timestampMs;
    hasFix_    = true;
}

void Atgm336hDriver::parseRMC(char* fields[], uint8_t count)
{
    (void)count;
    if (fields[2][0] != 'A') { return; }
    reading_.speedKmh   = parseFloat(fields[7]) * nmea_atgm::KNOTS_TO_KMH;
    reading_.courseDeg  = parseFloat(fields[8]);
    reading_.timestampMs = millis();
}

float Atgm336hDriver::parseCoordinate(const char* raw, const char* hem)
{
    if (raw == nullptr || raw[0] == '\0') { return 0.0f; }
    float rawVal = strtof(raw, nullptr);
    if (!isfinite(rawVal)) { return 0.0f; }
    int32_t degrees = static_cast<int32_t>(rawVal / nmea_atgm::DEG_DIVISOR);
    float minutes   = rawVal - static_cast<float>(degrees) * nmea_atgm::DEG_DIVISOR;
    float decimal   = static_cast<float>(degrees) + (minutes / nmea_atgm::MINUTES_PER_DEG);
    if (hem != nullptr && (hem[0] == 'S' || hem[0] == 'W')) { decimal = -decimal; }
    return decimal;
}

float Atgm336hDriver::parseFloat(const char* str)
{
    if (str == nullptr || str[0] == '\0') { return 0.0f; }
    float val = strtof(str, nullptr);
    return isfinite(val) ? val : 0.0f;
}

int32_t Atgm336hDriver::parseInt(const char* str)
{
    if (str == nullptr || str[0] == '\0') { return 0; }
    return static_cast<int32_t>(strtol(str, nullptr, 10));
}
