#include "pump_ble.h"
#include "gas_sensor.h"
#include <NimBLEDevice.h>

// ============================================================
// State shared between the NimBLE host task (notify callback),
// the async web task (commands) and loop()
// ============================================================
static portMUX_TYPE pumpMux = portMUX_INITIALIZER_UNLOCKED;

#define PUMP_FRAME_MAX 64
static uint8_t frameBuf[PUMP_FRAME_MAX];
static size_t frameLen = 0;
static bool framePending = false;

// Queued commands (applied together in one write)
struct PumpCommands {
    bool hasEnable;   bool enable;
    bool hasLeak;     bool leak;
    bool hasLevel;    uint8_t level;
    bool hasRelease;  bool release;
    bool reset;
};
static PumpCommands cmds = {};

static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic* writeChr = nullptr;
static volatile bool linkLost = false;      // Set by the disconnect callback
static volatile int connectResult = 0;      // Async connect: 0 = pending, 1 = connected, -1 = failed
static bool connecting = false;             // Async connect in progress
static unsigned long connectStarted = 0;
static uint8_t tryType = BLE_ADDR_PUBLIC;   // Address type to try next if the scan has not seen the pump

static unsigned long lastAttempt = 0;
static unsigned long retryDelay = 0;        // 0 = try right away
static unsigned long releasedAt = 0;
static unsigned long lastRssi = 0;
static bool lastAvailable = false;
static bool lastRunning = false;

static const NimBLEUUID UUID_SERVICE(PUMP_SERVICE_UUID);
static const NimBLEUUID UUID_NOTIFY(PUMP_NOTIFY_UUID);
static const NimBLEUUID UUID_WRITE(PUMP_WRITE_UUID);

// ============================================================
// Helpers
// ============================================================

static void setState(const char* s) {
    if (strcmp(sensorData.pump.state, s) != 0) {
        strlcpy(sensorData.pump.state, s, sizeof(sensorData.pump.state));
        sensorData.pump.updates++;
        Serial.printf("[PUMP] State: %s\n", s);
    }
}

static uint16_t u16le(const uint8_t* d, size_t off) {
    return (uint16_t)d[off] | ((uint16_t)d[off + 1] << 8);
}

// Derived fields from the status byte (also used after an optimistic write)
static void applyStatus(uint8_t status) {
    PumpData& p = sensorData.pump;
    p.status_raw = status;
    p.pressure_level = (status & 0x60) >> 5;
    p.pump_enabled = (status & 0x01) != 0;
    p.leak_detection = (status & 0x04) != 0;
}

// Status frame: AA 93 <status> ... U16LE at 6 voltage, 8 current, 10 pressure raw,
// 12 temperature, 14 flow, 16 daily liters, 18 total liters
static void parseFrame(const uint8_t* d, size_t n) {
    if (n < 20 || d[0] != 0xAA || d[1] != 0x93) return;

    PumpData& p = sensorData.pump;
    uint8_t oldStatus = p.status_raw;
    bool hadData = p.has_data;

    applyStatus(d[2]);
    p.voltage = u16le(d, 6) / 10.0f;
    p.current = u16le(d, 8) / 10.0f;
    p.pressure_raw = u16le(d, 10);
    p.pressure = p.pressure_raw * 0.0070992366f - 0.063740458f;
    p.temperature = u16le(d, 12) / 10.0f;
    p.flow = u16le(d, 14) / 10.0f;
    p.daily_liters = u16le(d, 16);
    p.total_liters = u16le(d, 18);
    p.running = (p.current > 0.2f || p.flow > 0.0f);
    p.last_update = millis();
    p.has_data = true;

    // Publish immediately on switch/level changes or pump start/stop; values follow periodically
    if (!hadData || oldStatus != p.status_raw || lastRunning != p.running) {
        lastRunning = p.running;
        p.updates++;
    }
}

// Notify callback (runs in the NimBLE host task - only copy the data)
static void onNotify(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t length, bool isNotify) {
    portENTER_CRITICAL(&pumpMux);
    frameLen = length > PUMP_FRAME_MAX ? PUMP_FRAME_MAX : length;
    memcpy(frameBuf, data, frameLen);
    framePending = true;
    portEXIT_CRITICAL(&pumpMux);
}

class PumpClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* c) override {
        connectResult = 1;
    }
    void onConnectFail(NimBLEClient* c, int reason) override {
        connectResult = -1;
    }
    void onDisconnect(NimBLEClient* c, int reason) override {
        if (connecting) connectResult = -1;
        else linkLost = true;
    }
};
static PumpClientCallbacks clientCallbacks;

static void markDisconnected() {
    writeChr = nullptr;
    sensorData.pump.connected = false;
    sensorData.pump.ble_rssi = 0;
}

static void disconnect() {
    if (connecting) {
        if (client) client->cancelConnect();
        connecting = false;
        gasScanResume();
    }
    if (client && client->isConnected()) client->disconnect();
    markDisconnected();
}

// Starts a non-blocking connection attempt; the result arrives via the client callbacks.
// The scan is paused meanwhile: NimBLE cannot scan and initiate a connection at the same time.
static bool startConnect() {
    if (!client) {
        client = NimBLEDevice::createClient();
        client->setClientCallbacks(&clientCallbacks, false);
        client->setConnectTimeout(PUMP_CONNECT_TIMEOUT);
    }

    // Address type from the scan list (public vs. random); otherwise alternate between attempts
    uint8_t type = tryType;
    bool knownType = gasFindAddrType(config.pump_mac, &type);
    if (!knownType) tryType = (tryType == BLE_ADDR_PUBLIC) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

    gasScanPause();
    setState("Verbinde...");
    Serial.printf("[PUMP] Connecting to %s (type %u%s)\n", config.pump_mac, type, knownType ? "" : ", guessed");

    connectResult = 0;
    if (!client->connect(NimBLEAddress(std::string(config.pump_mac), type), true, true)) {
        gasScanResume();
        return false;
    }
    connecting = true;
    connectStarted = millis();
    return true;
}

// Called once the link is up: service discovery and subscription (short, typically < 1 s)
static bool finishConnect() {
    NimBLERemoteCharacteristic* notifyChr = nullptr;
    NimBLERemoteService* svc = client->getService(UUID_SERVICE);
    if (svc) {
        notifyChr = svc->getCharacteristic(UUID_NOTIFY);
        writeChr = svc->getCharacteristic(UUID_WRITE);
    }
    if (!notifyChr || !writeChr || !notifyChr->subscribe(true, onNotify)) {
        Serial.println("[PUMP] Service / characteristics not found - wrong device?");
        disconnect();
        return false;
    }

    linkLost = false;
    sensorData.pump.connected = true;
    sensorData.pump.mtu = client->getMTU();
    sensorData.pump.ble_rssi = client->getRssi();
    sensorData.pump.updates++;
    Serial.printf("[PUMP] Connected, MTU %u\n", (unsigned)sensorData.pump.mtu);
    return true;
}

// Retry later; back off while the pump is out of range
static void connectFailed() {
    retryDelay = retryDelay ? min(retryDelay * 2, (unsigned long)PUMP_RETRY_MAX_MS) : PUMP_RETRY_MS;
    lastAttempt = millis();
    setState("Nicht erreichbar");
}

// 60 byte command frame, same layout as the Home Assistant integration
static bool writeStatus(uint8_t status) {
    if (!writeChr || !client || !client->isConnected()) return false;

    uint8_t cmd[60] = {0};
    cmd[0] = 0xAA;
    cmd[1] = 0x39;
    cmd[2] = status;
    cmd[6] = 0x22;
    cmd[7] = 0x12;
    cmd[8] = 0x07;
    uint8_t sum = 0;
    for (int i = 0; i < 59; i++) sum += cmd[i];
    cmd[59] = sum;

    // With response; NimBLE falls back to a long write if the MTU is smaller than the frame
    bool ok = writeChr->writeValue(cmd, sizeof(cmd), true);
    Serial.printf("[PUMP] Write status 0x%02X: %s\n", status, ok ? "OK" : "FAILED");
    return ok;
}

// Applies all queued switch/level commands in one write
static void processCommands() {
    PumpCommands c;
    portENTER_CRITICAL(&pumpMux);
    c = cmds;
    cmds.hasEnable = cmds.hasLeak = cmds.hasLevel = false;
    cmds.hasRelease = false;
    cmds.reset = false;
    portEXIT_CRITICAL(&pumpMux);

    PumpData& p = sensorData.pump;

    if (c.reset) {
        disconnect();
        p.has_data = false;
        p.available = false;
        p.updates++;
        retryDelay = 0;
        lastAttempt = 0;
    }

    if (c.hasRelease && c.release != p.released) {
        p.released = c.release;
        p.updates++;
        if (p.released) {
            releasedAt = millis();
            disconnect();
            Serial.println("[PUMP] Bluetooth released for the app");
        } else {
            retryDelay = 0;
            lastAttempt = 0;
            Serial.println("[PUMP] Bluetooth taken back");
        }
    }

    if (!(c.hasEnable || c.hasLeak || c.hasLevel)) return;
    if (!p.connected || !p.has_data) {
        Serial.println("[PUMP] Command ignored - pump not connected or status unknown");
        p.updates++;   // Re-publish the real state so HA switches snap back
        return;
    }

    uint8_t status = p.status_raw;
    if (c.hasEnable) status = c.enable ? (status | 0x01) : (status & ~0x01);
    if (c.hasLeak)   status = c.leak   ? (status | 0x04) : (status & ~0x04);
    if (c.hasLevel)  status = (status & ~0x60) | ((c.level & 0x03) << 5);

    if (status != p.status_raw && writeStatus(status)) {
        applyStatus(status);   // Optimistic; the next status frame confirms it
    }
    p.updates++;
}

// ============================================================
// Public API
// ============================================================

bool pumpInit() {
    memset(&sensorData.pump, 0, sizeof(sensorData.pump));
    if (!NimBLEDevice::isInitialized()) NimBLEDevice::init("");
    NimBLEDevice::setMTU(247);   // Fits the 60 byte command frame in one write
    strlcpy(sensorData.pump.state, config.pump_mac[0] ? "Getrennt" : "Kein Gerät", sizeof(sensorData.pump.state));
    Serial.printf("[PUMP] CaraWater %s  MAC: %s\n", config.pump_model[0] ? config.pump_model : "pump",
        config.pump_mac[0] ? config.pump_mac : "(not set)");
    return true;
}

void pumpLoop() {
    PumpData& p = sensorData.pump;

    // New status frame from the notify callback
    uint8_t frame[PUMP_FRAME_MAX];
    size_t len = 0;
    portENTER_CRITICAL(&pumpMux);
    if (framePending) {
        len = frameLen;
        memcpy(frame, frameBuf, len);
        framePending = false;
    }
    portEXIT_CRITICAL(&pumpMux);
    if (len) parseFrame(frame, len);

    if (linkLost) {
        linkLost = false;
        if (p.connected) Serial.println("[PUMP] Disconnected");
        markDisconnected();
        p.updates++;
    }

    processCommands();

    // App release ends automatically after PUMP_RELEASE_MS
    if (p.released && PUMP_RELEASE_MS > 0 && millis() - releasedAt > PUMP_RELEASE_MS) {
        pumpSetReleased(false);
    }

    if (config.pump_mac[0] == '\0') {
        setState("Kein Gerät");
    } else if (p.released) {
        setState("Für App freigegeben");
    } else if (connecting) {
        // Async connect in progress - loop() keeps running meanwhile
        int result = connectResult;
        if (result == 0 && millis() - connectStarted > PUMP_CONNECT_TIMEOUT + 2000) {
            client->cancelConnect();   // Safety net if the stack never reports a result
            result = -1;
        }
        if (result != 0) {
            connecting = false;
            gasScanResume();
            if (result == 1 && finishConnect()) {
                retryDelay = PUMP_RETRY_MS;
                lastAttempt = millis();
            } else {
                connectFailed();
            }
        }
    } else if (!p.connected) {
        if (lastAttempt == 0 || millis() - lastAttempt > retryDelay) {
            if (!startConnect()) connectFailed();
        }
    } else {
        unsigned long age = millis() - p.last_update;
        // Connected but silent for a long time: drop the link and reconnect
        if (!p.has_data || age > PUMP_OFFLINE_MS * 3) {
            if (millis() - lastAttempt > PUMP_OFFLINE_MS * 3) {
                Serial.println("[PUMP] No status frames - reconnecting");
                disconnect();
                lastAttempt = 0;
            }
            setState("Keine Daten");
        } else {
            setState(age <= PUMP_OFFLINE_MS ? "Verbunden" : "Keine Daten");
        }
        if (millis() - lastRssi > 5000 && client && client->isConnected()) {
            lastRssi = millis();
            p.ble_rssi = client->getRssi();
        }
    }

    p.available = p.connected && !p.released && p.has_data && (millis() - p.last_update <= PUMP_OFFLINE_MS);
    if (p.available != lastAvailable) {
        lastAvailable = p.available;
        p.updates++;
    }
}

void pumpReset() {
    portENTER_CRITICAL(&pumpMux);
    cmds.reset = true;
    portEXIT_CRITICAL(&pumpMux);
}

void pumpSetEnabled(bool on) {
    portENTER_CRITICAL(&pumpMux);
    cmds.hasEnable = true; cmds.enable = on;
    portEXIT_CRITICAL(&pumpMux);
}

void pumpSetLeakDetection(bool on) {
    portENTER_CRITICAL(&pumpMux);
    cmds.hasLeak = true; cmds.leak = on;
    portEXIT_CRITICAL(&pumpMux);
}

void pumpSetPressureLevel(uint8_t level) {
    if (level < 1 || level > 3) return;
    portENTER_CRITICAL(&pumpMux);
    cmds.hasLevel = true; cmds.level = level;
    portEXIT_CRITICAL(&pumpMux);
}

void pumpSetReleased(bool released) {
    portENTER_CRITICAL(&pumpMux);
    cmds.hasRelease = true; cmds.release = released;
    portEXIT_CRITICAL(&pumpMux);
}
