// Copyright (c) FRC Team 122. All Rights Reserved.

#include "SwerveDrive/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"

using namespace ctre::phoenix;

SwerveModule::SwerveModule(int driveMotorId, int steerMotorId,
                           int steerEncoderId, frc::Rotation2d angleOffset)
    : m_id{driveMotorId / 10}, m_driveMotor{driveMotorId},
      m_steerMotor{steerMotorId}, m_steerEncoder{steerEncoderId},
      m_angleOffset{angleOffset} {
  InitEncoder(steerEncoderId);
  InitDriveMotor(driveMotorId);
  InitTurnMotor(steerMotorId);

  ResetToAbsolute();
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {}

void SwerveModule::ResetToAbsolute() {
  double angle = (GetAbsoluteRotation() - m_angleOffset).Degrees().value();
  double absolutePosition =
      Conversions::degreesToFalcon(angle, ModuleConstants::kTurnGearRatio);
  m_steerMotor.SetSelectedSensorPosition(absolutePosition);
}

frc::SwerveModuleState SwerveModule::GetCurrentState() {
  double velocity = Conversions::falconToMPS(
      m_driveMotor.GetSelectedSensorVelocity(),
      ModuleConstants::kWheelCircumference, ModuleConstants::kDriveGearRatio);
  return {units::meters_per_second_t{velocity}, GetRotation()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  double distance = Conversions::falconToMeters(
      m_driveMotor.GetSelectedSensorPosition(),
      ModuleConstants::kWheelCircumference, ModuleConstants::kDriveGearRatio);
  return {units::meter_t{distance}, GetRotation()};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
  if (m_steerMotor.HasResetOccurred()) {
    ResetToAbsolute();
  }

  frc::Rotation2d rotation = GetRotation();

  // TODO: May throw error
  state = frc::SwerveModuleState::Optimize(state, rotation);
  state = CheckForWrapAround(state, rotation);

  double velocity = Conversions::MPSToFalcon(
      state.speed.value(), ModuleConstants::kWheelCircumference,
      ModuleConstants::kDriveGearRatio);

  m_steerMotor.Set(
      motorcontrol::ControlMode::Position,
      Conversions::degreesToFalcon(state.angle.Degrees().value(),
                                   ModuleConstants::kTurnGearRatio));
  m_driveMotor.Set(motorcontrol::ControlMode::Velocity, velocity,
                   motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                   m_feedForward.Calculate(state.speed, 0_mps_sq).value());
}

void SwerveModule::SetOpenLoopState(frc::SwerveModuleState state) {
  if (m_steerMotor.HasResetOccurred()) {
    ResetToAbsolute();
  }

  frc::Rotation2d rotation = GetRotation();

  // TODO: May throw error
  state = frc::SwerveModuleState::Optimize(state, rotation);
  state = CheckForWrapAround(state, rotation);

  double speed = (state.speed / ModuleConstants::kMaxSpeed).value();

  m_steerMotor.Set(
      motorcontrol::ControlMode::Position,
      Conversions::degreesToFalcon(state.angle.Degrees().value(),
                                   ModuleConstants::kTurnGearRatio));
  m_driveMotor.Set(motorcontrol::ControlMode::PercentOutput, speed);
}

void SwerveModule::ResetDriveEncoders() {
  m_driveMotor.SetSelectedSensorPosition(0, 0, 0);
}

void SwerveModule::InitDriveMotor(int driveMotorID) {
  m_DriveMotorConfig.slot0.kP = ModuleConstants::kDriveP;
  m_DriveMotorConfig.slot0.kI = ModuleConstants::kDriveI;
  m_DriveMotorConfig.slot0.kD = ModuleConstants::kDriveD;
  m_DriveMotorConfig.slot0.kF = ModuleConstants::kDriveF;
  m_DriveMotorConfig.supplyCurrLimit =
      motorcontrol::SupplyCurrentLimitConfiguration(
          ModuleConstants::kDriveEnableCurrentLimit,
          ModuleConstants::kDriveContinuousCurrentLimit,
          ModuleConstants::kDrivePeakCurrentLimit,
          ModuleConstants::kDrivePeakCurrentDuration);
  m_DriveMotorConfig.initializationStrategy =
      sensors::SensorInitializationStrategy::BootToZero;
  m_DriveMotorConfig.openloopRamp = ModuleConstants::kOpenLoopRamp;
  m_DriveMotorConfig.closedloopRamp = ModuleConstants::kClosedLoopRamp;
  m_DriveMotorConfig.voltageCompSaturation = ModuleConstants::kDriveVoltageComp;
  m_DriveMotorConfig.velocityMeasurementPeriod =
      sensors::SensorVelocityMeasPeriod::Period_10Ms;

  m_driveMotor.ConfigAllSettings(m_DriveMotorConfig);
  m_driveMotor.SetInverted(ModuleConstants::kDriveMotorInverted);
  m_driveMotor.SetNeutralMode(ModuleConstants::kDriveMotorNeutral);
  m_driveMotor.SetSelectedSensorPosition(0);
}

void SwerveModule::InitTurnMotor(int turnMotorID) {
  m_steerMotor.ConfigFactoryDefault();

  m_TurnMotorConfig.primaryPID.selectedFeedbackSensor =
      motorcontrol::FeedbackDevice::IntegratedSensor;
  m_TurnMotorConfig.primaryPID.selectedFeedbackCoefficient = 1;
  m_TurnMotorConfig.slot0.kP = ModuleConstants::kTurnP;
  m_TurnMotorConfig.slot0.kI = ModuleConstants::kTurnI;
  m_TurnMotorConfig.slot0.kD = ModuleConstants::kTurnD;
  m_TurnMotorConfig.slot0.kF = ModuleConstants::kTurnF;
  m_TurnMotorConfig.supplyCurrLimit =
      motorcontrol::SupplyCurrentLimitConfiguration(
          ModuleConstants::kTurnEnableCurrentLimit,
          ModuleConstants::kTurnContinuousCurrentLimit,
          ModuleConstants::kTurnPeakCurrentLimit,
          ModuleConstants::kTurnPeakCurrentDuration);

  m_TurnMotorConfig.initializationStrategy =
      sensors::SensorInitializationStrategy::BootToZero;

  m_TurnMotorConfig.voltageCompSaturation = ModuleConstants::kTurnVoltageComp;

  m_steerMotor.ConfigAllSettings(m_TurnMotorConfig);
  m_steerMotor.SetInverted(ModuleConstants::kTurnMotorInverted);
  m_steerMotor.SetNeutralMode(ModuleConstants::kTurnMotorNeutral);
  m_steerMotor.ConfigSelectedFeedbackSensor(
      motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);

  ResetToAbsolute();
}

void SwerveModule::InitEncoder(int encoderID) {
  m_steerEncoder.ConfigFactoryDefault();

  m_EncoderConfig.absoluteSensorRange =
      sensors::AbsoluteSensorRange::Unsigned_0_to_360;
  m_EncoderConfig.sensorDirection = ModuleConstants::kEncoderInverted;
  m_EncoderConfig.initializationStrategy =
      sensors::SensorInitializationStrategy::BootToAbsolutePosition;
  m_EncoderConfig.sensorTimeBase = sensors::SensorTimeBase::PerSecond;

  m_steerEncoder.ConfigAllSettings(m_EncoderConfig);
}

void SwerveModule::UpdateSmartDash() {
  frc::SmartDashboard::PutNumber(std::to_string(m_id) + "PureRaw Angle",
                                 GetAbsoluteRotation().Degrees().value());
  frc::SmartDashboard::PutNumber(std::to_string(m_id) + "Magnet offset",
                                 m_angleOffset.Degrees().value());
}

frc::SwerveModuleState
SwerveModule::CheckForWrapAround(frc::SwerveModuleState desiredState,
                                 frc::Rotation2d currentState) {
  double currentRotation = currentState.Degrees().value();
  double desiredRotation = desiredState.angle.Degrees().value();
  double remainder = std::remainder(desiredRotation - currentRotation, 180);
  double newPos = remainder + currentRotation;
  double minDist = std::abs(std::remainder(newPos - desiredRotation, 180));

  double speedMulti = minDist < 0.001 ? 1 : -1;

  return {desiredState.speed * speedMulti,
          frc::Rotation2d(units::degree_t{newPos})};
}

frc::Rotation2d SwerveModule::GetRotation() {
  return units::degree_t{
      Conversions::falconToDegrees(m_steerMotor.GetSelectedSensorPosition(),
                                   ModuleConstants::kTurnGearRatio)};
}

frc::Rotation2d SwerveModule::GetAbsoluteRotation() {
  return units::degree_t{m_steerEncoder.GetAbsolutePosition()};
}

void SwerveModule::SetOffset(double offset) {
  m_angleOffset = units::degree_t{offset};
  ResetToAbsolute();
}
