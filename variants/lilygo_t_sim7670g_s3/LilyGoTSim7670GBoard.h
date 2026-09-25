#pragma once

#include <helpers/ESP32Board.h>

class LilyGoTSim7670GBoard : public ESP32Board {
public:
  const char* getManufacturerName() const override {
    return "LilyGo T-SIM7670G-S3 + T-Sim Shield";
  }
};
