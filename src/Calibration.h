#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "PmodToF.h"

namespace Calibration {

enum class State { Idle, Busy, Cover, Distance, Complete, Error };
enum class Action { None, Start, Crosstalk, Distance, Cancel };

portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
State state = State::Idle;
Action pending = Action::None;
double referenceMeters = 0;
uint8_t previousRegisters[13];
bool previousValid = false;

inline const char *stateName(State value) {
  switch (value) {
    case State::Idle: return "idle";
    case State::Busy: return "busy";
    case State::Cover: return "cover";
    case State::Distance: return "distance";
    case State::Complete: return "complete";
    case State::Error: return "error";
  }
  return "error";
}

inline State getState() {
  portENTER_CRITICAL(&lock);
  const State result = state;
  portEXIT_CRITICAL(&lock);
  return result;
}

inline bool submit(Action action, double distance, bool sensorAvailable) {
  portENTER_CRITICAL(&lock);
  const bool allowed = sensorAvailable && pending == Action::None &&
      ((action == Action::Start && (state == State::Idle || state == State::Complete)) ||
       (action == Action::Crosstalk && state == State::Cover) ||
       (action == Action::Distance && state == State::Distance) ||
       (action == Action::Cancel && (state == State::Cover || state == State::Distance)));
  if (allowed) {
    pending = action;
    referenceMeters = distance;
    state = State::Busy;
  }
  portEXIT_CRITICAL(&lock);
  return allowed;
}

inline void process() {
  portENTER_CRITICAL(&lock);
  const Action action = pending;
  const double distance = referenceMeters;
  pending = Action::None;
  portEXIT_CRITICAL(&lock);
  if (action == Action::None) return;

  bool ok = false;
  State next = State::Error;
  if (action == Action::Start) {
    previousValid = PmodToF::readRegisters(PmodToF::SensorAddress,
        PmodToF::CalibrationRegister, previousRegisters, sizeof(previousRegisters));
    ok = previousValid && PmodToF::calibrateMagnitude();
    next = State::Cover;
  } else if (action == Action::Crosstalk) {
    ok = PmodToF::calibrateCrosstalk();
    next = State::Distance;
  } else if (action == Action::Distance) {
    ok = PmodToF::calibrateDistance(distance) && PmodToF::saveCalibration();
    next = State::Complete;
  } else if (action == Action::Cancel) {
    ok = previousValid && PmodToF::writeCalibrationRegisters(previousRegisters, sizeof(previousRegisters));
    next = State::Idle;
  }

  if (!ok) {
    Serial.println("Pmod ToF calibration failed");
    if (previousValid && action != Action::Cancel &&
        !PmodToF::writeCalibrationRegisters(previousRegisters, sizeof(previousRegisters))) {
      Serial.println("Pmod ToF calibration rollback failed; restart required");
    }
    next = State::Error;
  }
  if (next == State::Complete || next == State::Idle) previousValid = false;
  portENTER_CRITICAL(&lock);
  state = next;
  portEXIT_CRITICAL(&lock);
}

inline bool measurementAllowed() {
  const State current = getState();
  return current == State::Idle || current == State::Complete;
}

} // namespace Calibration

#endif
