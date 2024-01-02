// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

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
  void SetDesiredState(frc::SwerveModuleState desiredState);
  void SetOpenLoopState(frc::SwerveModuleState desiredState);
  void ResetDriveEncoders();
  void InitDriveMotor(int driveMotorID);
  void InitTurnMotor(int turnMotorID);
  void InitEncoder(int encoderID);
  void UpdateSmartDash();
  frc::SwerveModuleState CheckForWrapAround(frc::SwerveModuleState desiredState,
                                            frc::Rotation2d currentState);
  frc::Rotation2d GetRotation();
  frc::Rotation2d GetAbsoluteRotation();
  void SetOffset(double offset);

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  int m_id;

  ctre::phoenix::motorcontrol::can::TalonFXConfiguration m_DriveMotorConfig =
      ctre::phoenix::motorcontrol::can::TalonFXConfiguration();

  ctre::phoenix::motorcontrol::can::TalonFXConfiguration m_TurnMotorConfig =
      ctre::phoenix::motorcontrol::can::TalonFXConfiguration();

  ctre::phoenix::sensors::CANCoderConfiguration m_EncoderConfig =
      ctre::phoenix::sensors::CANCoderConfiguration();

  ctre::phoenix::motorcontrol::can::TalonFX m_driveMotor;
  ctre::phoenix::motorcontrol::can::TalonFX m_steerMotor;

  ctre::phoenix::sensors::CANCoder m_steerEncoder;
  frc::Rotation2d m_angleOffset;
  frc::SimpleMotorFeedforward<units::meters> m_feedForward{
      ModuleConstants::kDriveS, ModuleConstants::kDriveV,
      ModuleConstants::kDriveA};
};
