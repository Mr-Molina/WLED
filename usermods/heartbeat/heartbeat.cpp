#warning **** Included USERMOD_HEARTBEAT ****

#include "wled.h"
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

class HeartbeatUsermod : public Usermod {
private:
  bool enabled = true;
  bool initDone = false;
  bool sensorFound = false;

  // configurable pins (defaults for common ESP32-C3 boards)
  int8_t sdaPin = -1; // use global I2C by default
  int8_t sclPin = -1;

  MAX30105 particleSensor;

  // beat detection state
  uint8_t  bpm = 0;
  uint8_t  beatDetected = 0;
  uint16_t ibi = 0; // inter-beat interval in ms

  // rolling BPM average
  static constexpr uint8_t RATE_SIZE = 4;
  uint8_t rates[RATE_SIZE] = {};
  uint8_t rateIndex = 0;
  uint8_t rateCount = 0;

  unsigned long lastBeatTime = 0;
  unsigned long lastLoopTime = 0;

  // finger-on threshold (IR value below this = no finger)
  static constexpr long IR_THRESHOLD = 50000;

  static const char _name[];
  static const char _enabled[];

  void initSensor() {
    if (sdaPin >= 0 && sclPin >= 0) {
      Wire.begin(sdaPin, sclPin);
    }

    sensorFound = particleSensor.begin(Wire, I2C_SPEED_FAST);
    if (!sensorFound) {
      DEBUG_PRINTLN(F("MAX30102 not found"));
      return;
    }

    particleSensor.setup(
      0x1F, // LED brightness (~6.4mA, moderate power)
      4,    // sample average
      2,    // LED mode: Red + IR
      100,  // sample rate 100Hz
      411,  // pulse width 411us (18-bit resolution)
      4096  // ADC range
    );
    particleSensor.setPulseAmplitudeRed(0x0A); // low red for indicator
    particleSensor.setPulseAmplitudeIR(0x1F);  // main sensing LED

    DEBUG_PRINTLN(F("MAX30102 initialized"));
  }

public:
  void setup() override {
    if (!initDone) {
      // allocate um_data for sharing with effects
      um_data = new um_data_t;
      um_data->u_size = 3;
      um_data->u_type = new um_types_t[um_data->u_size];
      um_data->u_data = new void*[um_data->u_size];
      um_data->u_data[0] = &bpm;
      um_data->u_type[0] = UMT_BYTE;
      um_data->u_data[1] = &beatDetected;
      um_data->u_type[1] = UMT_BYTE;
      um_data->u_data[2] = &ibi;
      um_data->u_type[2] = UMT_UINT16;
    }

    if (enabled) {
      initSensor();
    }

    initDone = true;
  }

  void loop() override {
    if (!enabled || !sensorFound || strip.isUpdating()) return;

    // clear beat flag from previous frame
    beatDetected = 0;

    long irValue = particleSensor.getIR();

    // no finger on sensor
    if (irValue < IR_THRESHOLD) {
      bpm = 0;
      ibi = 0;
      return;
    }

    if (checkForBeat(irValue)) {
      unsigned long now = millis();
      long delta = now - lastBeatTime;
      lastBeatTime = now;

      // sanity check: reject if interval is outside 300-2000ms (30-200 BPM)
      if (delta > 300 && delta < 2000) {
        ibi = (uint16_t)delta;
        beatDetected = 1;

        rates[rateIndex++] = (uint8_t)(60000UL / delta);
        rateIndex %= RATE_SIZE;
        if (rateCount < RATE_SIZE) rateCount++;

        // compute rolling average
        uint16_t sum = 0;
        for (uint8_t i = 0; i < rateCount; i++) {
          sum += rates[i];
        }
        bpm = (uint8_t)(sum / rateCount);
      }
    }
  }

  bool getUMData(um_data_t **data) override {
    if (!data || !enabled) return false;
    *data = um_data;
    return true;
  }

  void addToJsonInfo(JsonObject& root) override {
    JsonObject user = root[F("u")];
    if (user.isNull()) user = root.createNestedObject(F("u"));

    if (!sensorFound) {
      JsonArray arr = user.createNestedArray(F("Heartbeat Sensor"));
      arr.add(F("Not Found"));
      return;
    }

    JsonArray bpmArr = user.createNestedArray(F("Heart Rate"));
    if (bpm > 0) {
      bpmArr.add(bpm);
      bpmArr.add(F(" BPM"));
    } else {
      bpmArr.add(F("No finger"));
    }
  }

  void addToConfig(JsonObject& root) override {
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top[FPSTR(_enabled)] = enabled;
    top[F("SDA pin")] = sdaPin;
    top[F("SCL pin")] = sclPin;
  }

  bool readFromConfig(JsonObject& root) override {
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull()) {
      DEBUG_PRINT(FPSTR(_name));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }

    bool configComplete = true;
    configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled, true);
    configComplete &= getJsonValue(top[F("SDA pin")], sdaPin, -1);
    configComplete &= getJsonValue(top[F("SCL pin")], sclPin, -1);

    if (initDone) {
      // settings changed at runtime — reinitialize
      sensorFound = false;
      bpm = 0;
      ibi = 0;
      beatDetected = 0;
      rateCount = 0;
      rateIndex = 0;
      if (enabled) initSensor();
    }

    return configComplete;
  }

  uint16_t getId() override {
    return USERMOD_ID_HEARTBEAT;
  }
};

const char HeartbeatUsermod::_name[]    PROGMEM = "Heartbeat";
const char HeartbeatUsermod::_enabled[] PROGMEM = "enabled";

static HeartbeatUsermod heartbeat_usermod;
REGISTER_USERMOD(heartbeat_usermod);
