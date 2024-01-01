// Copyright (c) FRC Team 122. All Rights Reserved.

#include "SwerveDrive/SwerveModule.h"

#include "Constants.h"

using namespace ctre::phoenix;

SwerveModule::SwerveModule(int driveMotorId, int steerMotorId,
                           int steerEncoderId, frc::Rotation2d angleOffset)
    : driveMotor(driveMotorId), steerMotor(steerMotorId),
      steerEncoder(steerEncoderId), angleOffset(angleOffset)
// feedForward()
{
  id = (driveMotorId / 10);

  InitEncoder(steerEncoderId);
  InitDriveMotor(driveMotorId);
  InitTurnMotor(steerMotorId);

  ResetToAbsolute();
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {}

void SwerveModule::ResetToAbsolute() {
  lastAngle = double{(GetAbsoluteRotation() - angleOffset).Degrees()};
  double absolutePosition =
      Conversions::degreesToFalcon(lastAngle, ModuleConstants::kTurnGearRatio);
  steerMotor.SetSelectedSensorPosition(absolutePosition);
}

frc::SwerveModuleState SwerveModule::GetCurrentState() {
  double velocity = Conversions::falconToMPS(
      driveMotor.GetSelectedSensorVelocity(),
      ModuleConstants::kWheelCircumference, ModuleConstants::kDriveGearRatio);
  return {units::meters_per_second_t{velocity}, GetRotation()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  double distance = Conversions::falconToMeters(
      driveMotor.GetSelectedSensorPosition(),
      ModuleConstants::kWheelCircumference, ModuleConstants::kDriveGearRatio);
  return {units::meter_t{distance}, GetRotation()};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState desiredState,
                                   bool isOpenLoop) {
  if (steerMotor.HasResetOccurred()) {
    ResetToAbsolute();
  }

  frc::Rotation2d rotation = GetRotation();

  // TODO: May throw error
  desiredState = frc::SwerveModuleState::Optimize(desiredState, rotation);
  desiredState = CheckForWrapAround(desiredState, rotation);

  // sets Angle and velocity of the wheels
  SetAngle(desiredState, rotation);
  if (isOpenLoop) {
    SetSpeed((desiredState.speed / ModuleConstants::kMaxSpeed).value());
  } else {
    SetVelocity(desiredState.speed);
  }
}

void SwerveModule::ResetDriveEncoders() {
  driveMotor.SetSelectedSensorPosition(0, 0, 0);
}

void SwerveModule::SetAngle(frc::SwerveModuleState desiredState,
                            frc::Rotation2d currentAngleRotation2d) {
  double angle = desiredState.angle.Degrees().value();
  if (std::abs(desiredState.speed.value()) <
      ModuleConstants::kMaxSpeed * 0.01) {
    angle = lastAngle;
  }

  if (std::abs(
          ((currentAngleRotation2d - desiredState.angle).Degrees()).value()) >
      ModuleConstants::kAllowableAngleTolerance.Degrees().value()) {
    steerMotor.Set(
        motorcontrol::ControlMode::Position,
        Conversions::degreesToFalcon(angle, ModuleConstants::kTurnGearRatio));
  } else {
    steerMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
  }
  lastAngle = angle;
}

void SwerveModule::SetSpeed(double speed) {
  driveMotor.Set(motorcontrol::ControlMode::PercentOutput, speed);
}

void SwerveModule::SetVelocity(units::meters_per_second_t velocity) {
  double tick_velocity = Conversions::MPSToFalcon(
      velocity.value(), ModuleConstants::kWheelCircumference,
      ModuleConstants::kDriveGearRatio);
  units::volt_t ffCalculate = feedForward.Calculate(velocity, 0_mps_sq);

  driveMotor.Set(motorcontrol::ControlMode::Velocity, tick_velocity,
                 motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                 ffCalculate.value());
}

void SwerveModule::InitDriveMotor(int driveMotorID) {
  driveMotor.ConfigFactoryDefault();

  kDriveMotorConfig.slot0.kP = ModuleConstants::kDriveP;
  kDriveMotorConfig.slot0.kI = ModuleConstants::kDriveI;
  kDriveMotorConfig.slot0.kD = ModuleConstants::kDriveD;
  kDriveMotorConfig.slot0.kF = ModuleConstants::kDriveF;
  kDriveMotorConfig.supplyCurrLimit =
      motorcontrol::SupplyCurrentLimitConfiguration(
          ModuleConstants::kDriveEnableCurrentLimit,
          ModuleConstants::kDriveContinuousCurrentLimit,
          ModuleConstants::kDrivePeakCurrentLimit,
          ModuleConstants::kDrivePeakCurrentDuration);
  kDriveMotorConfig.initializationStrategy =
      sensors::SensorInitializationStrategy::BootToZero;
  kDriveMotorConfig.openloopRamp = ModuleConstants::kOpenLoopRamp;
  kDriveMotorConfig.closedloopRamp = ModuleConstants::kClosedLoopRamp;
  kDriveMotorConfig.voltageCompSaturation = ModuleConstants::kDriveVoltageComp;
  kDriveMotorConfig.velocityMeasurementPeriod =
      sensors::SensorVelocityMeasPeriod::Period_10Ms;

  driveMotor.ConfigAllSettings(kDriveMotorConfig);
  driveMotor.SetInverted(ModuleConstants::kDriveMotorInverted);
  driveMotor.SetNeutralMode(ModuleConstants::kDriveMotorNeutral);
  driveMotor.SetSelectedSensorPosition(0);
}

void SwerveModule::InitTurnMotor(int turnMotorID) {
  steerMotor.ConfigFactoryDefault();

  kTurnMotorConfig.primaryPID.selectedFeedbackSensor =
      motorcontrol::FeedbackDevice::IntegratedSensor;
  kTurnMotorConfig.primaryPID.selectedFeedbackCoefficient = 1;
  kTurnMotorConfig.slot0.kP = ModuleConstants::kTurnP;
  kTurnMotorConfig.slot0.kI = ModuleConstants::kTurnI;
  kTurnMotorConfig.slot0.kD = ModuleConstants::kTurnD;
  kTurnMotorConfig.slot0.kF = ModuleConstants::kTurnF;
  kTurnMotorConfig.supplyCurrLimit =
      motorcontrol::SupplyCurrentLimitConfiguration(
          ModuleConstants::kTurnEnableCurrentLimit,
          ModuleConstants::kTurnContinuousCurrentLimit,
          ModuleConstants::kTurnPeakCurrentLimit,
          ModuleConstants::kTurnPeakCurrentDuration);

  kTurnMotorConfig.initializationStrategy =
      sensors::SensorInitializationStrategy::BootToZero;
  
  kTurnMotorConfig.voltageCompSaturation = ModuleConstants::kTurnVoltageComp;

  steerMotor.ConfigAllSettings(kTurnMotorConfig);
  steerMotor.SetInverted(ModuleConstants::kTurnMotorInverted);
  steerMotor.SetNeutralMode(ModuleConstants::kTurnMotorNeutral);
  steerMotor.ConfigSelectedFeedbackSensor(
      motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);

  ResetToAbsolute();
}

void SwerveModule::InitEncoder(int encoderID) {
  steerEncoder.ConfigFactoryDefault();

  kEncoderConfig.absoluteSensorRange =
      sensors::AbsoluteSensorRange::Unsigned_0_to_360;
  kEncoderConfig.sensorDirection = ModuleConstants::kEncoderInverted;
  kEncoderConfig.initializationStrategy =
      sensors::SensorInitializationStrategy::BootToAbsolutePosition;
  kEncoderConfig.sensorTimeBase = sensors::SensorTimeBase::PerSecond;

  steerEncoder.ConfigAllSettings(kEncoderConfig);
}

void SwerveModule::UpdateSmartDash() {
  frc::SmartDashboard::PutNumber(std::to_string(id) + "Last Angle", lastAngle);
  frc::SmartDashboard::PutNumber(std::to_string(id) + "PureRaw Angle",
                                 GetAbsoluteRotation().Degrees().value());
  frc::SmartDashboard::PutNumber(std::to_string(id) + "Magnet offset",
                                 angleOffset.Degrees().value());
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
  return units::degree_t{Conversions::falconToDegrees(
      steerMotor.GetSelectedSensorPosition(), ModuleConstants::kTurnGearRatio)};
}

frc::Rotation2d SwerveModule::GetAbsoluteRotation() {
  return units::degree_t{steerEncoder.GetAbsolutePosition()};
}

void SwerveModule::SetTurnCoast() {
  steerMotor.SetNeutralMode(motorcontrol::NeutralMode::Coast);
}

void SwerveModule::SetTurnBrake() {
  steerMotor.SetNeutralMode(motorcontrol::NeutralMode::Brake);
}

void SwerveModule::SetOffset(double offset) {
  angleOffset = units::degree_t{offset};
  ResetToAbsolute();
}
