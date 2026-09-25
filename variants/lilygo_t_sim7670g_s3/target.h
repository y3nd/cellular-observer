#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include <helpers/radiolib/CustomSX1262Wrapper.h>
#include <helpers/AutoDiscoverRTCClock.h>
#include <helpers/cellular_observer/CellularGnss.h>
#include "LilyGoTSim7670GBoard.h"

extern LilyGoTSim7670GBoard board;
extern WRAPPER_CLASS radio_driver;
extern AutoDiscoverRTCClock rtc_clock;
extern offband::CellularGnssSensorManager sensors;

bool radio_init();
mesh::LocalIdentity radio_new_identity();
