#include "CellularBootstrap.h"

#if defined(ARDUINO) && defined(OFFBAND_OBSERVER_CELLULAR)

#include <Arduino.h>
#include <Network.h>
#include <PPP.h>
#include <Preferences.h>
#include <RTClib.h>
#include <cmath>

#include "../diagnostics/CrashLog.h"
#include "../prefs/PrefsRead.h"

#ifndef OFFBAND_LTE_MODEM_RX
  #error "OFFBAND_LTE_MODEM_RX must be defined by the board variant"
#endif
#ifndef OFFBAND_LTE_MODEM_TX
  #error "OFFBAND_LTE_MODEM_TX must be defined by the board variant"
#endif
#ifndef OFFBAND_LTE_MODEM_PWRKEY
  #error "OFFBAND_LTE_MODEM_PWRKEY must be defined by the board variant"
#endif
#ifndef OFFBAND_LTE_MODEM_RESET
  #error "OFFBAND_LTE_MODEM_RESET must be defined by the board variant"
#endif
#ifndef OFFBAND_LTE_MODEM_DTR
  #error "OFFBAND_LTE_MODEM_DTR must be defined by the board variant"
#endif

#ifndef OFFBAND_LTE_APN
  #define OFFBAND_LTE_APN "internet"
#endif
#ifndef OFFBAND_LTE_SIM_PIN
  #define OFFBAND_LTE_SIM_PIN ""
#endif

namespace offband {
namespace {

constexpr int kModemUart = 1;
constexpr int kModemBaud = 115200;
constexpr uint32_t kRegistrationTimeoutMs = 180000;
constexpr uint32_t kPppTimeoutMs = 60000;
constexpr uint32_t kRecoveryDelayMs = 10000;

int splitCsv(const String& line, String* fields, int maximum_fields) {
    int count = 0;
    int start = 0;
    while (count < maximum_fields) {
        const int comma = line.indexOf(',', start);
        if (comma < 0) {
            fields[count++] = line.substring(start);
            break;
        }
        fields[count++] = line.substring(start, comma);
        start = comma + 1;
    }
    for (int i = 0; i < count; ++i) fields[i].trim();
    return count;
}

double nmeaCoordinateToDegrees(const String& coordinate, char hemisphere) {
    const double nmea = coordinate.toDouble();
    const double degrees = floor(nmea / 100.0);
    const double minutes = nmea - degrees * 100.0;
    double decimal = degrees + minutes / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
    return decimal;
}

uint32_t gnssTimestamp(const String& date, const String& utc_time) {
    // SIM7670 reports DDMMYY and HHMMSS.s. DateTime avoids dependence on the
    // process timezone (unlike mktime) and matches the firmware RTC convention.
    if (date.length() < 6 || utc_time.length() < 6) return 0;
    const int day = date.substring(0, 2).toInt();
    const int month = date.substring(2, 4).toInt();
    const int year = 2000 + date.substring(4, 6).toInt();
    const int hour = utc_time.substring(0, 2).toInt();
    const int minute = utc_time.substring(2, 4).toInt();
    const int second = utc_time.substring(4, 6).toInt();
    if (day < 1 || day > 31 || month < 1 || month > 12 ||
        hour > 23 || minute > 59 || second > 60) {
        return 0;
    }
    return DateTime(year, month, day, hour, minute, second).unixtime();
}

bool parseGnssInfo(const String& response, CellularGnssFix& fix) {
    fix = CellularGnssFix{};
    const int marker = response.indexOf("+CGNSSINFO:");
    if (marker < 0) return false;

    int end = response.indexOf('\r', marker);
    if (end < 0) end = response.indexOf('\n', marker);
    if (end < 0) end = response.length();

    String payload = response.substring(marker + strlen("+CGNSSINFO:"), end);
    payload.trim();
    String fields[16];
    if (splitCsv(payload, fields, 16) < 16 || fields[0].isEmpty() ||
        fields[4].isEmpty() || fields[5].isEmpty() || fields[6].isEmpty() ||
        fields[7].isEmpty()) {
        return false;
    }

    const int mode = fields[0].toInt();
    if (mode != 2 && mode != 3) return false;
    const double latitude = nmeaCoordinateToDegrees(fields[4], fields[5][0]);
    const double longitude = nmeaCoordinateToDegrees(fields[6], fields[7][0]);
    if (!std::isfinite(latitude) || !std::isfinite(longitude)) return false;

    fix.valid = true;
    fix.mode = static_cast<uint8_t>(mode);
    fix.satellites = static_cast<uint16_t>(
        fields[1].toInt() + fields[2].toInt() + fields[3].toInt());
    fix.latitude_udeg = static_cast<int32_t>(lround(latitude * 1000000.0));
    fix.longitude_udeg = static_cast<int32_t>(lround(longitude * 1000000.0));
    fix.altitude_mm = static_cast<int32_t>(lround(fields[10].toDouble() * 1000.0));
    fix.timestamp = gnssTimestamp(fields[8], fields[9]);
    return true;
}

String rawAtCommand(HardwareSerial& uart, const char* command,
                    uint32_t timeout_ms) {
    while (uart.available()) uart.read();
    uart.print(command);
    uart.print("\r\n");

    String response;
    const uint32_t started = millis();
    while (millis() - started < timeout_ms) {
        while (uart.available()) {
            response += static_cast<char>(uart.read());
            if (response.indexOf("\r\nOK\r\n") >= 0 ||
                response.indexOf("\r\nERROR\r\n") >= 0) {
                return response;
            }
        }
        delay(10);
    }
    return response;
}

bool ensureModemPowered() {
    pinMode(OFFBAND_LTE_MODEM_RESET, OUTPUT);
    digitalWrite(OFFBAND_LTE_MODEM_RESET, HIGH);
    pinMode(OFFBAND_LTE_MODEM_DTR, OUTPUT);
    digitalWrite(OFFBAND_LTE_MODEM_DTR, LOW);

    HardwareSerial probe(kModemUart);
    probe.begin(kModemBaud, SERIAL_8N1,
                OFFBAND_LTE_MODEM_RX, OFFBAND_LTE_MODEM_TX);
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (rawAtCommand(probe, "AT", 800).indexOf("OK") >= 0) {
            rawAtCommand(probe, "ATE0", 1000);
            probe.end();
            return true;
        }
    }

    pinMode(OFFBAND_LTE_MODEM_PWRKEY, OUTPUT);
    digitalWrite(OFFBAND_LTE_MODEM_PWRKEY, LOW);
    delay(100);
    digitalWrite(OFFBAND_LTE_MODEM_PWRKEY, HIGH);
    delay(1000);
    digitalWrite(OFFBAND_LTE_MODEM_PWRKEY, LOW);

    const uint32_t started = millis();
    while (millis() - started < 20000) {
        if (rawAtCommand(probe, "AT", 1000).indexOf("OK") >= 0) {
            rawAtCommand(probe, "ATE0", 1000);
            probe.end();
            return true;
        }
        delay(250);
    }
    probe.end();
    return false;
}

void onNetworkEvent(arduino_event_id_t event, arduino_event_info_t) {
    switch (event) {
        case ARDUINO_EVENT_PPP_START:
            crashLogf("[Cellular] PPP interface started");
            break;
        case ARDUINO_EVENT_PPP_CONNECTED:
            crashLogf("[Cellular] PPP link connected");
            break;
        case ARDUINO_EVENT_PPP_GOT_IP:
            crashLogf("[Cellular] PPP got IPv4 %s",
                      PPP.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_PPP_LOST_IP:
            crashLogf("[Cellular] PPP lost IP");
            break;
        case ARDUINO_EVENT_PPP_DISCONNECTED:
            crashLogf("[Cellular] PPP link disconnected");
            break;
        case ARDUINO_EVENT_PPP_STOP:
            crashLogf("[Cellular] PPP interface stopped");
            break;
        default:
            break;
    }
}

}  // namespace

void CellularBootstrap::begin() {
    state_ = CellularBootstrapState::Starting;
    state_since_ms_ = millis();
    cmux_started_ = false;
    gnss_enabled_ = false;

    crashLogf("[Cellular] starting SIM7670G on UART%d RX=%d TX=%d",
              kModemUart, OFFBAND_LTE_MODEM_RX, OFFBAND_LTE_MODEM_TX);
    if (!ensureModemPowered()) {
        crashLogf("[Cellular] modem did not answer AT commands");
        state_ = CellularBootstrapState::Failed;
        state_since_ms_ = millis();
        return;
    }

    String apn = OFFBAND_LTE_APN;
    String pin = OFFBAND_LTE_SIM_PIN;
    Preferences prefs;
    if (prefs.begin("lte", true)) {
        String saved_apn = prefStr(prefs, "apn");
        String saved_pin = prefStr(prefs, "pin");
        if (!saved_apn.isEmpty()) apn = saved_apn;
        if (!saved_pin.isEmpty()) pin = saved_pin;
        prefs.end();
    }

    Network.onEvent(onNetworkEvent);
    PPP.setApn(apn.c_str());
    PPP.setPin(pin.isEmpty() ? nullptr : pin.c_str());
    PPP.setPins(OFFBAND_LTE_MODEM_TX, OFFBAND_LTE_MODEM_RX,
                -1, -1, ESP_MODEM_FLOW_CONTROL_NONE);
    if (!PPP.begin(PPP_MODEM_SIM7600, kModemUart, kModemBaud)) {
        crashLogf("[Cellular] PPP driver initialization failed");
        state_ = CellularBootstrapState::Failed;
        state_since_ms_ = millis();
        return;
    }

    String response;
    PPP.cmd("ATE0", response, 5000);
    PPP.cmd("AT+CMEE=2", response, 5000);
    crashLogf("[Cellular] modem=%s SIM=%s APN configured (%u chars)",
              PPP.moduleName().c_str(), PPP.attached() ? "attached" : "registering",
              (unsigned)apn.length());
    state_ = CellularBootstrapState::Registering;
    state_since_ms_ = millis();
}

void CellularBootstrap::loop() {
    const uint32_t now = millis();
    switch (state_) {
        case CellularBootstrapState::Registering:
            if (PPP.attached()) {
                crashLogf("[Cellular] registered; entering CMUX/PPP data mode");
                cmux_started_ = PPP.mode(ESP_MODEM_MODE_CMUX);
                if (!cmux_started_) {
                    crashLogf("[Cellular] could not enter CMUX mode");
                    state_ = CellularBootstrapState::Failed;
                } else {
                    state_ = CellularBootstrapState::Connecting;
                }
                state_since_ms_ = now;
            } else if (now - state_since_ms_ >= kRegistrationTimeoutMs) {
                crashLogf("[Cellular] LTE registration timed out");
                state_ = CellularBootstrapState::Failed;
                state_since_ms_ = now;
            }
            break;

        case CellularBootstrapState::Connecting:
            if (PPP.connected()) {
                crashLogf("[Cellular] online; IP=%s RSSI=%d",
                          PPP.localIP().toString().c_str(), PPP.RSSI());
                state_ = CellularBootstrapState::Connected;
                state_since_ms_ = now;
            } else if (now - state_since_ms_ >= kPppTimeoutMs) {
                crashLogf("[Cellular] PPP IP connection timed out");
                state_ = CellularBootstrapState::Failed;
                state_since_ms_ = now;
            }
            break;

        case CellularBootstrapState::Connected:
            if (!PPP.connected()) {
                crashLogf("[Cellular] connection lost; recovery reboot pending");
                state_ = CellularBootstrapState::Failed;
                state_since_ms_ = now;
            }
            break;

        case CellularBootstrapState::Failed:
            if (now - state_since_ms_ >= kRecoveryDelayMs) {
                crashLogf("[Cellular] restarting to recover modem state");
                Serial.flush();
                ESP.restart();
            }
            break;

        default:
            break;
    }
}

bool CellularBootstrap::isConnected() const {
    return state_ == CellularBootstrapState::Connected && PPP.connected();
}

bool CellularBootstrap::setGnssEnabled(bool enabled) {
    if (state_ == CellularBootstrapState::Boot ||
        state_ == CellularBootstrapState::Starting ||
        state_ == CellularBootstrapState::Failed) {
        return false;
    }

    String response;
    const bool ok = PPP.cmd(enabled ? "AT+CGNSSPWR=1" : "AT+CGNSSPWR=0",
                            response, 15000);
    if (ok) gnss_enabled_ = enabled;
    crashLogf("[GNSS] receiver power %s: %s",
              enabled ? "on" : "off", ok ? "ok" : "failed");
    return ok;
}

bool CellularBootstrap::readGnssFix(CellularGnssFix& fix) {
    fix = CellularGnssFix{};
    if (!gnss_enabled_) return false;
    String response;
    // The modem answers immediately even while acquiring. Keep the timeout
    // bounded so a sick AT channel cannot starve MeshCore's radio/BLE loop.
    if (!PPP.cmd("AT+CGNSSINFO", response, 1000)) return false;
    return parseGnssInfo(response, fix);
}

CellularBootstrap& cellularBootstrap() {
    static CellularBootstrap instance;
    return instance;
}

}  // namespace offband

#else

namespace offband {
void CellularBootstrap::begin() {}
void CellularBootstrap::loop() {}
bool CellularBootstrap::isConnected() const { return false; }
bool CellularBootstrap::setGnssEnabled(bool) { return false; }
bool CellularBootstrap::readGnssFix(CellularGnssFix& fix) {
    fix = CellularGnssFix{};
    return false;
}
CellularBootstrap& cellularBootstrap() {
    static CellularBootstrap instance;
    return instance;
}
}  // namespace offband

#endif
