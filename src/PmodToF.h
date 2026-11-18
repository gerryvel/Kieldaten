#ifndef PMOD_TOF_H
#define PMOD_TOF_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

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
constexpr uint8_t CalibrationRegister = 0x24;
constexpr uint8_t UserCalibrationOffset = 0x10;

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

inline bool startMeasurement(uint8_t sampleControl) {
  if (!writeRegister(SensorAddress, SampleControlRegister, sampleControl) ||
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
  for (uint8_t attempts = 0; attempts < 80; ++attempts) {
    uint8_t status = 0;
    if (!readRegisters(SensorAddress, InterruptClearRegister, &status, 1)) return false;
    if (status != 0) return true;
    delay(1);
  }
  return false;
}

inline bool readDistanceMeters(float &distanceMeters) {
  if (!startMeasurement(MeasurementSampleControl)) {
    return false;
  }

  uint8_t rawDistance[2];
  if (!readRegisters(SensorAddress, DistanceRegister, rawDistance, sizeof(rawDistance))) {
    return false;
  }

  const uint16_t raw = (static_cast<uint16_t>(rawDistance[0]) << 8) | rawDistance[1];
  distanceMeters = (static_cast<float>(raw) / 65536.0f) * 33.31f;
  return true;
}

inline bool writeCalibrationRegisters(const uint8_t *data, size_t length, uint8_t start = CalibrationRegister) {
  for (size_t i = 0; i < length; ++i) {
    if (!writeRegister(SensorAddress, static_cast<uint8_t>(start + i), data[i])) {
      return false;
    }
  }
  return true;
}

inline bool calibrateMagnitude() {
  uint8_t data[3];
  return startMeasurement(0x61) &&
         readRegisters(SensorAddress, 0xF6, data, sizeof(data)) &&
         writeCalibrationRegisters(data, sizeof(data), 0x2C);
}

inline double decodeSignal(const uint8_t *data) {
  const uint16_t raw = (static_cast<uint16_t>(data[1]) << 8) | data[2];
  const int32_t mantissa = raw >= 0x8000 ? static_cast<int32_t>(raw) - 65536 : raw;
  return ldexp(static_cast<double>(mantissa), data[0]);
}

inline bool encodeSignal(double value, uint8_t *data) {
  if (!isfinite(value)) return false;
  int exponent = 0;
  while (fabs(value) >= 32768.0 && exponent < 255) {
    value /= 2.0;
    ++exponent;
  }
  if (fabs(value) >= 32768.0) return false;
  const int32_t mantissa = static_cast<int32_t>(value);
  const uint16_t raw = static_cast<uint16_t>(mantissa);
  data[0] = static_cast<uint8_t>(exponent);
  data[1] = static_cast<uint8_t>(raw >> 8);
  data[2] = static_cast<uint8_t>(raw);
  return true;
}

inline bool calibrateCrosstalk() {
  double iSum = 0, qSum = 0;
  uint32_t gainSum = 0;
  for (int i = 0; i < 100; ++i) {
    uint8_t data[14];
    if (!startMeasurement(MeasurementSampleControl) ||
        !readRegisters(SensorAddress, 0xDA, data, sizeof(data))) return false;
    iSum += decodeSignal(data);
    qSum += decodeSignal(data + 3);
    gainSum += (static_cast<uint16_t>(data[12]) << 8) | data[13];
  }
  uint8_t calibration[8];
  if (!encodeSignal(iSum / 100, calibration) ||
      !encodeSignal(qSum / 100, calibration + 3)) return false;
  const uint16_t gain = gainSum / 100;
  calibration[6] = static_cast<uint8_t>(gain >> 8);
  calibration[7] = static_cast<uint8_t>(gain);
  return writeCalibrationRegisters(calibration, sizeof(calibration));
}

inline bool calibrateDistance(double distanceMeters) {
  uint32_t phaseSum = 0;
  for (int i = 0; i < 100; ++i) {
    uint8_t data[2];
    if (!startMeasurement(MeasurementSampleControl) ||
        !readRegisters(SensorAddress, 0xD8, data, sizeof(data))) return false;
    phaseSum += (static_cast<uint16_t>(data[0]) << 8) | data[1];
  }
  const int32_t offset = static_cast<int32_t>(phaseSum / 100.0 - distanceMeters / 33.31 * 65536.0);
  const uint16_t raw = static_cast<uint16_t>(offset);
  const uint8_t data[] = {static_cast<uint8_t>(raw >> 8), static_cast<uint8_t>(raw)};
  return writeCalibrationRegisters(data, sizeof(data), 0x2F);
}

inline bool saveCalibration() {
  uint8_t data[16] = {0xEB};
  if (!readRegisters(SensorAddress, CalibrationRegister, data + 1, 13)) return false;
  uint8_t checksum = 0;
  for (uint8_t value : data) checksum += value;
  data[14] = checksum;

  Wire.beginTransmission(EepromAddress);
  bool queued = Wire.write(UserCalibrationOffset) == 1;
  queued = Wire.write(data, sizeof(data)) == sizeof(data) && queued;
  const uint8_t result = Wire.endTransmission();
  if (!queued || result != 0) return false;
  delay(10);
  uint8_t verify[16];
  return readRegisters(EepromAddress, UserCalibrationOffset, verify, sizeof(verify)) &&
         memcmp(data, verify, sizeof(data)) == 0;
}

} // namespace PmodToF

#endif
