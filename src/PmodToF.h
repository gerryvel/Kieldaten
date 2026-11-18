#ifndef PMOD_TOF_H
#define PMOD_TOF_H

#include <Arduino.h>
#include <Wire.h>

namespace PmodToF {

constexpr uint8_t SensorAddress = 0x57;
constexpr uint8_t EepromAddress = 0x50;
constexpr uint8_t DeviceIdRegister = 0x00;
constexpr uint8_t SampleControlRegister = 0x13;
constexpr uint8_t InterruptControlRegister = 0x60;
constexpr uint8_t CommandRegister = 0xB0;
constexpr uint8_t DistanceRegister = 0xD1;
constexpr uint8_t InterruptClearRegister = 0x69;
constexpr uint8_t MeasurementSampleControl = 0x7D;
constexpr uint8_t DataReadyInterrupt = 0x01;
constexpr uint8_t SoftClearCommand = 0xD1;
constexpr uint8_t SoftStartCommand = 0x49;
constexpr uint16_t MeasurementTimeMs = 20;

inline bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  const bool bytesQueued = Wire.write(reg) == 1 && Wire.write(value) == 1;
  const uint8_t result = Wire.endTransmission();
  return bytesQueued && result == 0;
}

inline bool readRegisters(uint8_t address, uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(address);
  if (Wire.write(reg) != 1) {
    Wire.endTransmission();
    return false;
  }
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(address, static_cast<uint8_t>(length), static_cast<uint8_t>(true)) != length) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!Wire.available()) {
      return false;
    }
    data[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

inline bool loadCalibrationBlock(uint8_t offset) {
  uint8_t data[16];
  if (!readRegisters(EepromAddress, offset, data, sizeof(data)) || data[0] != 0xEB) {
    return false;
  }

  const uint8_t storedChecksum = data[14];
  data[14] = 0;
  uint8_t checksum = 0;
  for (uint8_t value : data) {
    checksum += value;
  }
  if (checksum != storedChecksum) {
    return false;
  }

  for (uint8_t i = 0; i < 13; ++i) {
    if (!writeRegister(SensorAddress, static_cast<uint8_t>(0x24 + i), data[1 + i])) {
      return false;
    }
  }
  return true;
}

inline bool begin(bool &calibrationLoaded) {
  calibrationLoaded = false;

  uint8_t deviceId = 0;
  if (!readRegisters(SensorAddress, DeviceIdRegister, &deviceId, 1) || deviceId != 0x0A) {
    return false;
  }
  if (!writeRegister(SensorAddress, CommandRegister, SoftClearCommand)) {
    return false;
  }

  const uint8_t initRegisters[][2] = {
      {0x10, 0x04},
      {0x11, 0x6E},
      {0x12, 0x00},
      {SampleControlRegister, 0x71},
      {0x18, 0x22},
      {0x19, 0x22},
      {0x90, 0x0F},
      {0x91, 0xFF},
      {InterruptControlRegister, 0x00},
  };
  for (const auto &reg : initRegisters) {
    if (!writeRegister(SensorAddress, reg[0], reg[1])) {
      return false;
    }
  }

  calibrationLoaded = loadCalibrationBlock(0x10) || loadCalibrationBlock(0x20);
  return true;
}

inline bool readDistanceMeters(float &distanceMeters) {
  if (!writeRegister(SensorAddress, SampleControlRegister, MeasurementSampleControl) ||
      !writeRegister(SensorAddress, InterruptControlRegister, DataReadyInterrupt)) {
    return false;
  }

  uint8_t pendingInterrupt = 0;
  if (!readRegisters(SensorAddress, InterruptClearRegister, &pendingInterrupt, 1)) {
    return false;
  }
  if (!writeRegister(SensorAddress, CommandRegister, SoftStartCommand)) {
    return false;
  }
  delay(MeasurementTimeMs);

  uint8_t rawDistance[2];
  if (!readRegisters(SensorAddress, DistanceRegister, rawDistance, sizeof(rawDistance))) {
    return false;
  }

  const uint16_t raw = (static_cast<uint16_t>(rawDistance[0]) << 8) | rawDistance[1];
  distanceMeters = (static_cast<float>(raw) / 65536.0f) * 33.31f;
  return true;
}

} // namespace PmodToF

#endif
