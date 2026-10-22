#ifndef GAS_SENSOR_H
#define GAS_SENSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"

// Gas level: passive BLE advertising sensor (default: Rotarex DIMES / SRG WAVE)

bool gasInit();
void gasLoop();

// The BLE scanner is also started when only the pump is enabled (device list in the pump tab).
// While the pump connects, the scan is paused (NimBLE cannot scan and initiate at the same time).
void gasScanPause();
void gasScanResume();

// BLE address type of a device from the scan list (false = not seen yet)
bool gasFindAddrType(const char* mac, uint8_t* type);

// Call after the sensor source / MAC / mapping was changed in the web UI
void gasReset();

// Call after tank size or fill stop changed (keeps the last reading)
void gasRecalculate();

// Nearby BLE devices for the web UI (address, name, rssi, company ID, service data UUID, pump flag)
void gasFillSeenList(JsonArray arr);
void gasPrintSeenList();

#endif // GAS_SENSOR_H
