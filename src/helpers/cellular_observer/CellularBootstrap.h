#pragma once

#include <stdint.h>

namespace offband {

struct CellularGnssFix {
    bool valid = false;
    int32_t latitude_udeg = 0;
    int32_t longitude_udeg = 0;
    int32_t altitude_mm = 0;
    uint32_t timestamp = 0;
    uint16_t satellites = 0;
    uint8_t mode = 0;
};

enum class CellularBootstrapState : uint8_t {
    Boot,
    Starting,
    Registering,
    Connecting,
    Connected,
    Failed,
};

class CellularBootstrap {
public:
    void begin();
    void loop();
    bool isConnected() const;
    bool setGnssEnabled(bool enabled);
    bool readGnssFix(CellularGnssFix& fix);
    bool isGnssEnabled() const { return gnss_enabled_; }
    CellularBootstrapState state() const { return state_; }

private:
    CellularBootstrapState state_ = CellularBootstrapState::Boot;
    uint32_t state_since_ms_ = 0;
    bool cmux_started_ = false;
    bool gnss_enabled_ = false;
};

CellularBootstrap& cellularBootstrap();

}  // namespace offband
