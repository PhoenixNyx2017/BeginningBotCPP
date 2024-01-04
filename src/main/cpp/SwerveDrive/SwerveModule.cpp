// Copyright (c) FRC Team 122. All Rights Reserved.

#include "SwerveDrive/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/RobotBase.h>

#include "Constants.h"

using namespace ctre::phoenix;

SwerveModule::SwerveModule(int driveMotorID, int steerMotorID,
                           int steerEncoderId, frc::Rotation2d angleOffset)
    : m_id{driveMotorID / 10}, m_driveMotor{driveMotorID},
      m_steerMotor{steerMotorID}, m_steerEncoder{steerEncoderId},
      m_angleOffset{angleOffset},
      m_driveSim("SPARK MAX ", driveMotorID),
      m_steerSim("SPARK MAX ", steerMotorID),
      m_driveSimVelocity(m_driveSim.GetDouble("Velocity")),
      m_driveSimPosition(m_driveSim.GetDouble("Position")),
      m_steerSimPosition(m_steerSim.GetDouble("Position")) {
  InitEncoder(steerEncoderId);

  m_steerMotor.ConfigFactoryDefault();
  m_driveMotor.ConfigFactoryDefault();

  m_steerMotor.SetSelectedFeedbackSensor(
      motorcontrol::FeedbackDevice::IntegratedSensor);
  m_steerMotor.SetSelectedFeedbackCoefficient(1);

  m_steerMotor.SetPIDF(ModuleConstants::kTurnP, ModuleConstants::kTurnI,
                       ModuleConstants::kTurnD, ModuleConstants::kTurnF);
  m_driveMotor.SetPIDF(ModuleConstants::kDriveP, ModuleConstants::kDriveI,
                       ModuleConstants::kDriveD, ModuleConstants::kDriveF);

  m_steerMotor.SetSupplyCurrentLimit(
      motorcontrol::SupplyCurrentLimitConfiguration(
          ModuleConstants::kTurnEnableCurrentLimit,
          ModuleConstants::kTurnContinuousCurrentLimit,
          ModuleConstants::kTurnPeakCurrentLimit,
          ModuleConstants::kTurnPeakCurrentDuration));
  m_driveMotor.SetSupplyCurrentLimit(
      motorcontrol::SupplyCurrentLimitConfiguration(
          ModuleConstants::kDriveEnableCurrentLimit,
          ModuleConstants::kDriveContinuousCurrentLimit,
          ModuleConstants::kDrivePeakCurrentLimit,
          ModuleConstants::kDrivePeakCurrentDuration));

  m_steerMotor.SetSensorInitializationStrategy(
      sensors::SensorInitializationStrategy::BootToZero);
  m_driveMotor.SetSensorInitializationStrategy(
      sensors::SensorInitializationStrategy::BootToZero);

  m_steerMotor.SetVoltageCompensation(ModuleConstants::kTurnVoltageComp);
  m_driveMotor.SetVoltageCompensation(ModuleConstants::kDriveVoltageComp);

  m_steerMotor.SetInverted(ModuleConstants::kTurnMotorInverted);
  m_driveMotor.SetInverted(ModuleConstants::kDriveMotorInverted);

  m_steerMotor.SetNeutralMode(ModuleConstants::kTurnMotorNeutral);
  m_driveMotor.SetNeutralMode(ModuleConstants::kDriveMotorNeutral);

  m_steerMotor.ConfigSelectedFeedbackSensor(
      motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);

  m_driveMotor.SetOpenLoopRampRate(ModuleConstants::kOpenLoopRamp);
  m_driveMotor.SetClosedLoopRampRate(ModuleConstants::kClosedLoopRamp);
  m_driveMotor.SetVelocityMeasurementPeriod(
      sensors::SensorVelocityMeasPeriod::Period_10Ms);

  m_steerMotor.ConfigAllSettings();
  m_driveMotor.ConfigAllSettings();

  m_driveMotor.SetSelectedSensorPosition(0);

  m_steerMotor.SetPositionConversionFactor(360 *
                                           ModuleConstants::kTurnGearRatio);
  m_driveMotor.SetPositionConversionFactor(
      ModuleConstants::kWheelCircumference * ModuleConstants::kDriveGearRatio);
  m_driveMotor.SetVelocityConversionFactor(
      60 * ModuleConstants::kWheelCircumference *
      ModuleConstants::kDriveGearRatio);

  if constexpr (frc::RobotBase::IsSimulation()) {
    m_simTimer.Start();
  }
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {
  SyncEncoders();

  frc::SmartDashboard::PutNumber(std::to_string(m_id) + "PureRaw Angle",
                                 GetAbsoluteRotation().Degrees().value());
  frc::SmartDashboard::PutNumber(std::to_string(m_id) + "Magnet offset",
                                 m_angleOffset.Degrees().value());
}

void SwerveModule::SimulationPeriodic() {
  units::second_t dt = m_simTimer.Get();
  m_simTimer.Reset();
  m_driveSimPosition.Set(m_driveSimPosition.Get() +
                         m_driveSimVelocity.Get() * dt.value());
}

void SwerveModule::SyncEncoders() {
  m_steerMotor.SetSelectedSensorPosition(
      (GetAbsoluteRotation() - m_angleOffset).Degrees().value());
}

frc::SwerveModuleState SwerveModule::GetCurrentState() {
  return {units::meters_per_second_t{m_driveMotor.GetVelocity()},
          GetRotation()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveMotor.GetPosition()}, GetRotation()};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
  frc::Rotation2d rotation = GetRotation();

  // TODO: May throw error
  state = frc::SwerveModuleState::Optimize(state, rotation);
  state = CheckForWrapAround(state, rotation);

  m_steerMotor.Set(motorcontrol::ControlMode::Position,
                   state.angle.Degrees().value());
  m_driveMotor.Set(motorcontrol::ControlMode::Velocity, state.speed.value(),
                   motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                   m_feedForward.Calculate(state.speed, 0_mps_sq).value());

  if constexpr (frc::RobotBase::IsSimulation()) {
    m_steerSimPosition.Set(state.angle.Radians().value());
  }
}

void SwerveModule::SetOpenLoopState(frc::SwerveModuleState state) {
  frc::Rotation2d rotation = GetRotation();

  // TODO: May throw error
  state = frc::SwerveModuleState::Optimize(state, rotation);
  state = CheckForWrapAround(state, rotation);

  m_steerMotor.Set(motorcontrol::ControlMode::Position,
                   state.angle.Degrees().value());
  m_driveMotor.Set(motorcontrol::ControlMode::PercentOutput,
                   (state.speed / ModuleConstants::kMaxSpeed).value());
}

void SwerveModule::ResetDriveEncoders() {
  m_driveMotor.SetSelectedSensorPosition(0, 0, 0);
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
  return units::degree_t{m_steerMotor.GetPosition()};
}

frc::Rotation2d SwerveModule::GetAbsoluteRotation() {
  return units::degree_t{m_steerEncoder.GetAbsolutePosition()};
}

void SwerveModule::SetOffset(double offset) {
  m_angleOffset = units::degree_t{offset};
  SyncEncoders();
}
