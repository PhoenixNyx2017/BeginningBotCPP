// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include <units/velocity.h>
#include <units/voltage.h>


#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <frc/smartdashboard/SmartDashboard.h>




class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(int driveMotorId, int steerMotorId, int steerEncoderId, frc::Rotation2d angleOffset);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void resetToAbsolute();
  frc::SwerveModuleState getCurrentState();
  frc::SwerveModulePosition getPosition();
  void setDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop);
  void resetDriveEncoders();
  void setAngle(frc::SwerveModuleState desiredState, frc::Rotation2d angle);
  void setVelocity(frc::SwerveModuleState desiredState, bool isOpenLoop);
  void initDriveMotor(int driveMotorID);
  void initTurnMotor(int turnMotorID);
  void initEncoder(int encoderID);
  void updateSmartDash();
  frc::SwerveModuleState checkForWrapAround(frc::SwerveModuleState desiredState, frc::Rotation2d currentState);
  frc::Rotation2d getCANCoder();
  frc::Rotation2d getAngleRotation2d();
  frc::Rotation2d getAbsoluteAngle();
  double getDistanceMeters();
  double getVelocityMPS();
  void setTurnCoast();
  void setTurnBrake();
  void setOffset(double offset);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  int id;
  double lastAngle;

  ctre::phoenix::motorcontrol::can::TalonFXConfiguration kDriveMotorConfig = ctre::phoenix::motorcontrol::can::TalonFXConfiguration();

  ctre::phoenix::motorcontrol::can::TalonFXConfiguration kTurnMotorConfig = ctre::phoenix::motorcontrol::can::TalonFXConfiguration();

  ctre::phoenix::sensors::CANCoderConfiguration kEncoderConfig = ctre::phoenix::sensors::CANCoderConfiguration();

  ctre::phoenix::motorcontrol::can::WPI_TalonFX driveMotor;
  ctre::phoenix::motorcontrol::can::WPI_TalonFX steerMotor;

  ctre::phoenix::sensors::CANCoder steerEncoder;
  frc::Rotation2d angleOffset;
  // frc::SimpleMotorFeedforward<units::meter_t> feedForward;

};
