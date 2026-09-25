#include "CellularGnss.h"

#if defined(ARDUINO) && defined(OFFBAND_OBSERVER_CELLULAR)

#include <Arduino.h>
#include <cstdlib>
#include <cstring>

#include <helpers/ClockSanity.h>
#include <helpers/diagnostics/CrashLog.h>

namespace offband {

void CellularGnssLocationProvider::begin() {
    enabled_ = cellularBootstrap().isGnssEnabled() ||
               cellularBootstrap().setGnssEnabled(true);
    next_poll_ms_ = 0;
}

void CellularGnssLocationProvider::stop() {
    if (enabled_) cellularBootstrap().setGnssEnabled(false);
    enabled_ = false;
    fix_ = CellularGnssFix{};
    had_fix_ = false;
}

void CellularGnssLocationProvider::reset() {
    fix_ = CellularGnssFix{};
    had_fix_ = false;
    next_poll_ms_ = 0;
}

void CellularGnssLocationProvider::setPollIntervalSeconds(uint32_t seconds) {
    if (seconds == 0) seconds = 1;
    if (seconds > 86400) seconds = 86400;
    poll_interval_ms_ = seconds * 1000UL;
}

void CellularGnssLocationProvider::loop() {
    if (!enabled_) return;
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - next_poll_ms_) < 0) return;
    next_poll_ms_ = now + poll_interval_ms_;

    CellularGnssFix candidate;
    const bool valid = cellularBootstrap().readGnssFix(candidate);
    if (valid) {
        fix_ = candidate;
        if (!had_fix_) {
            crashLogf("[GNSS] fix acquired; mode=%u satellites=%u",
                      (unsigned)fix_.mode, (unsigned)fix_.satellites);
        }
        had_fix_ = true;
    } else {
        fix_.valid = false;
        if (had_fix_) crashLogf("[GNSS] fix lost");
        had_fix_ = false;
    }
}

bool CellularGnssSensorManager::begin() {
    // Probe power control once. The persisted MeshCore GPS preference is applied
    // immediately after begin() and may turn the receiver back off.
    location_.begin();
    detected_ = location_.isEnabled();
    active_ = detected_;
    return detected_;
}

void CellularGnssSensorManager::loop() {
    if (!active_) return;
    location_.loop();
    if (!location_.isValid()) return;

    node_lat = static_cast<double>(location_.getLatitude()) / 1000000.0;
    node_lon = static_cast<double>(location_.getLongitude()) / 1000000.0;
    node_altitude = static_cast<double>(location_.getAltitude()) / 1000.0;

    const uint32_t timestamp = static_cast<uint32_t>(location_.getTimestamp());
    mesh::RTCClock* clock = location_.getClock();
    if (clock != nullptr && timestamp >= GPS_CLOCK_SANE_MIN &&
        plausibleEpoch(timestamp) &&
        (last_clock_sync_ms_ == 0 ||
         millis() - last_clock_sync_ms_ > GPS_CLOCK_SYNC_INTERVAL)) {
        const uint32_t old_time = clock->getCurrentTime();
        clock->setCurrentTime(timestamp);
        logClockSet("cellular-gnss", old_time, timestamp);
        last_clock_sync_ms_ = millis();
    }
}

bool CellularGnssSensorManager::querySensors(
    uint8_t requester_permissions, CayenneLPP& telemetry) {
    if ((requester_permissions & TELEM_PERM_LOCATION) &&
        active_ && location_.isValid()) {
        telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
    }
    return true;
}

const char* CellularGnssSensorManager::getSettingName(int i) const {
    return i == 0 ? "gps" : nullptr;
}

const char* CellularGnssSensorManager::getSettingValue(int i) const {
    return i == 0 ? (active_ ? "1" : "0") : nullptr;
}

bool CellularGnssSensorManager::setSettingValue(
    const char* name, const char* value) {
    if (strcmp(name, "gps") == 0) {
        if (strcmp(value, "0") == 0) {
            location_.stop();
            active_ = false;
        } else {
            location_.begin();
            active_ = location_.isEnabled();
            detected_ = detected_ || active_;
        }
        return true;
    }
    if (strcmp(name, "gps_interval") == 0) {
        location_.setPollIntervalSeconds(strtoul(value, nullptr, 10));
        return true;
    }
    return false;
}

size_t CellularGnssSensorManager::getGpsStatusText(char* out, size_t cap) {
    const bool valid = active_ && location_.isValid();
    const int n = snprintf(
        out, cap,
        "detected=%d active=%d fix=%d baud=0 lat=%ld lon=%ld alt_cm=%ld sats=%ld time=%ld",
        detected_ ? 1 : 0, active_ ? 1 : 0, valid ? 1 : 0,
        (long)location_.getLatitude(), (long)location_.getLongitude(),
        (long)(location_.getAltitude() / 10),
        (long)location_.satellitesCount(), (long)location_.getTimestamp());
    return n < 0 ? 0 : static_cast<size_t>(n);
}

}  // namespace offband

#endif
