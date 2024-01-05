// Copyright (c) FRC Team 122. All Rights Reserved.

#include "util/NKFalcon.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <string>

using namespace ctre::phoenix;

NKFalcon::NKFalcon(int CANId) : m_id{CANId}, m_motor{CANId} {
  SetPositionConversionFactor(1.0);
  SetVelocityConversionFactor(1.0);
}

void NKFalcon::SetPositionConversionFactor(double conversion) {
  // Falcon units start in ticks
  //
  // rotations = ticks * (1 rotation / 2048 ticks)
  double falconToRotations = 1.0 / 2048.0;
  m_positionConversionFactor = conversion * falconToRotations;
}

void NKFalcon::SetVelocityConversionFactor(double conversion) {
  // Falcon units start in ticks / 100 ms
  //
  // rot / min = ticks / 100 ms * (60,000 ms / 1 min) * (1 rot / 2048 ticks)
  double falconToRPM = 600.0 / 2048.0;
  m_velocityConversionFactor = conversion * falconToRPM;
}

void NKFalcon::ConfigFactoryDefault() { m_motor.ConfigFactoryDefault(); }

void NKFalcon::ConfigAllSettings() { m_motor.ConfigAllSettings(m_config); }

void NKFalcon::SetInverted(bool inverted) { m_motor.SetInverted(inverted); }

void NKFalcon::SetSelectedFeedbackSensor(motorcontrol::FeedbackDevice device) {
  m_config.primaryPID.selectedFeedbackSensor = device;
}

void NKFalcon::SetSelectedFeedbackCoefficient(double coeff) {
  m_config.primaryPID.selectedFeedbackCoefficient = coeff;
}

void NKFalcon::SetPIDF(double p, double i, double d, double f) {
  m_config.slot0.kP = p;
  m_config.slot0.kI = i;
  m_config.slot0.kD = d;
  m_config.slot0.kF = f;
}

void NKFalcon::SetNeutralMode(motorcontrol::NeutralMode neutralMode) {
  m_motor.SetNeutralMode(neutralMode);
}

void NKFalcon::SetSupplyCurrentLimit(
    motorcontrol::SupplyCurrentLimitConfiguration currentConfig) {
  m_config.supplyCurrLimit = currentConfig;
}

void NKFalcon::SetSensorInitializationStrategy(
    sensors::SensorInitializationStrategy strategy) {
  m_config.initializationStrategy = strategy;
}

void NKFalcon::SetVoltageCompensation(double voltage) {
  m_config.voltageCompSaturation = voltage;
}

void NKFalcon::SetOpenLoopRampRate(double rampRate) {
  m_config.openloopRamp = rampRate;
}

void NKFalcon::SetClosedLoopRampRate(double rampRate) {
  m_config.closedloopRamp = rampRate;
}

void NKFalcon::SetSelectedSensorPosition(double sensorPos, int pidIdx,
                                         int timeoutMs) {
  m_motor.SetSelectedSensorPosition(sensorPos / m_positionConversionFactor,
                                    pidIdx, timeoutMs);
}

double NKFalcon::GetPosition() {
  return m_motor.GetSelectedSensorPosition() * m_positionConversionFactor;
}

double NKFalcon::GetVelocity() {
  return m_motor.GetSelectedSensorVelocity() * m_velocityConversionFactor;
}

void NKFalcon::SetVelocityMeasurementPeriod(
    sensors::SensorVelocityMeasPeriod measurementPeriod) {
  m_config.velocityMeasurementPeriod = measurementPeriod;
}

void NKFalcon::Set(motorcontrol::ControlMode mode, double value) {
  switch (mode) {
  case motorcontrol::ControlMode::Position:
    m_motor.Set(mode, value / m_positionConversionFactor);
    break;
  case motorcontrol::ControlMode::Velocity:
    m_motor.Set(mode, value / m_velocityConversionFactor);
    break;
  default:
    m_motor.Set(mode, value);
  }
}

void NKFalcon::Set(motorcontrol::ControlMode mode, double demand0,
                   motorcontrol::DemandType demand1Type, double demand1) {
  switch (mode) {
  case motorcontrol::ControlMode::Position:
    m_motor.Set(mode, demand0 / m_positionConversionFactor, demand1Type,
                demand1);
    break;
  case motorcontrol::ControlMode::Velocity:
    m_motor.Set(mode, demand0 / m_velocityConversionFactor, demand1Type,
                demand1);
    break;
  default:
    m_motor.Set(mode, demand1, demand1Type, demand1);
  }
}

void NKFalcon::ConfigSelectedFeedbackSensor(motorcontrol::FeedbackDevice device,
                                            int pidIdx, int timeoutMs) {
  m_motor.ConfigSelectedFeedbackSensor(device, pidIdx, timeoutMs);
}

bool NKFalcon::HasResetOccured() { return m_motor.HasResetOccurred(); }
