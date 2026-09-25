#pragma once

#include <helpers/SensorManager.h>

#include "CellularBootstrap.h"

namespace offband {

class CellularGnssLocationProvider : public LocationProvider {
public:
    explicit CellularGnssLocationProvider(mesh::RTCClock* clock)
        : clock_(clock) {}

    long getLatitude() override { return fix_.latitude_udeg; }
    long getLongitude() override { return fix_.longitude_udeg; }
    long getAltitude() override { return fix_.altitude_mm; }
    long satellitesCount() override { return fix_.satellites; }
    bool isValid() override { return enabled_ && fix_.valid; }
    long getTimestamp() override { return fix_.timestamp; }
    mesh::RTCClock* getClock() override { return clock_; }
    void sendSentence(const char*) override {}
    void reset() override;
    void begin() override;
    void stop() override;
    void loop() override;
    bool isEnabled() override { return enabled_; }
    void setPollIntervalSeconds(uint32_t seconds);

private:
    mesh::RTCClock* clock_;
    CellularGnssFix fix_;
    uint32_t next_poll_ms_ = 0;
    uint32_t poll_interval_ms_ = 5000;
    bool enabled_ = false;
    bool had_fix_ = false;
};

class CellularGnssSensorManager : public SensorManager {
public:
    explicit CellularGnssSensorManager(CellularGnssLocationProvider& location)
        : location_(location) {}

    bool begin() override;
    void loop() override;
    bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
    int getNumSettings() const override { return 1; }
    const char* getSettingName(int i) const override;
    const char* getSettingValue(int i) const override;
    bool setSettingValue(const char* name, const char* value) override;
    LocationProvider* getLocationProvider() override { return &location_; }
    uint32_t getGpsClockSyncTime() const override { return last_clock_sync_ms_; }
    size_t getGpsStatusText(char* out, size_t cap) override;
    bool gpsHasFix() { return active_ && location_.isValid(); }

private:
    CellularGnssLocationProvider& location_;
    uint32_t last_clock_sync_ms_ = 0;
    bool detected_ = false;
    bool active_ = false;
};

}  // namespace offband
