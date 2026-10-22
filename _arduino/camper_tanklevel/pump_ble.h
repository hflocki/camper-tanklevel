#ifndef PUMP_BLE_H
#define PUMP_BLE_H

#include <Arduino.h>
#include "config.h"

// Optional water pump: CaraWater pump (tested protocol: ISP 15) via Bluetooth LE (GATT connection).
// Protocol ported from the Home Assistant integration "carawater_isp15":
//   notify FF01: status frame  AA 93 <status> .. <U16LE values at 6..19>
//   write  FF02: 60 byte frame AA 39 <status> .. 22 12 07 .. <checksum>
// Requires the BLE stack (gasInit() starts it, also when gas is disabled).

bool pumpInit();
void pumpLoop();

// Call after the pump MAC was changed in the web UI (reconnects to the new device)
void pumpReset();

// Commands - safe to call from the async web task or the MQTT callback.
// They are queued and written to the pump in pumpLoop().
void pumpSetEnabled(bool on);
void pumpSetLeakDetection(bool on);
void pumpSetPressureLevel(uint8_t level);   // 1-3
void pumpSetReleased(bool released);        // true = disconnect so the CaraWater app can connect

#endif // PUMP_BLE_H
