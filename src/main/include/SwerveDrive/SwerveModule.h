// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <units/math.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"

class SwerveModule : public frc2::SubsystemBase {
public:
  SwerveModule(int driveMotorId, int steerMotorId, int steerEncoderId,
               frc::Rotation2d angleOffset);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void ResetToAbsolute();
  frc::SwerveModuleState GetCurrentState();
  frc::SwerveModulePosition GetPosition();
  void SetDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop);
  void ResetDriveEncoders();
  void SetAngle(frc::SwerveModuleState desiredState, frc::Rotation2d angle);
  void SetSpeed(double speed);
  void SetVelocity(units::meters_per_second_t velocity);
  void InitDriveMotor(int driveMotorID);
  void InitTurnMotor(int turnMotorID);
  void InitEncoder(int encoderID);
  void UpdateSmartDash();
  frc::SwerveModuleState CheckForWrapAround(frc::SwerveModuleState desiredState,
                                            frc::Rotation2d currentState);
  frc::Rotation2d GetRotation();
  frc::Rotation2d GetAbsoluteRotation();
  void SetTurnCoast();
  void SetTurnBrake();
  void SetOffset(double offset);

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  int id;
  double lastAngle;

  ctre::phoenix::motorcontrol::can::TalonFXConfiguration kDriveMotorConfig =
      ctre::phoenix::motorcontrol::can::TalonFXConfiguration();

  ctre::phoenix::motorcontrol::can::TalonFXConfiguration kTurnMotorConfig =
      ctre::phoenix::motorcontrol::can::TalonFXConfiguration();

  ctre::phoenix::sensors::CANCoderConfiguration kEncoderConfig =
      ctre::phoenix::sensors::CANCoderConfiguration();

  ctre::phoenix::motorcontrol::can::WPI_TalonFX driveMotor;
  ctre::phoenix::motorcontrol::can::WPI_TalonFX steerMotor;

  ctre::phoenix::sensors::CANCoder steerEncoder;
  frc::Rotation2d angleOffset;
  frc::SimpleMotorFeedforward<units::meters> feedForward{
      ModuleConstants::kDriveS, ModuleConstants::kDriveV,
      ModuleConstants::kDriveA};
};
