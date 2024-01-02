// Copyright (c) FRC Team 122. All Rights Reserved.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_swerveDrive.SetDefaultCommand(Drive(
      &m_swerveDrive,
      [this] {
        return MathUtilNK::calculateAxis(
            m_driverController.GetX(), GeneralConstants::kDefaultAxisDeadband,
            GeneralConstants::kDriveLimit *
                GeneralConstants::kMaxTranslationalVelocity);
      },
      [this] {
        return MathUtilNK::calculateAxis(
            m_driverController.GetY(), GeneralConstants::kDefaultAxisDeadband,
            GeneralConstants::kDriveLimit *
                GeneralConstants::kMaxTranslationalVelocity);
      },
      [this] {
        return MathUtilNK::calculateAxis(
            m_driverController.GetZ(), GeneralConstants::kDefaultAxisDeadband,
            GeneralConstants::kRotationLimit *
                GeneralConstants::kMaxRotationalVelocity);
      }));
  // TODO: test

  // Configure the button bindings
  ConfigureBindings();
  m_swerveDrive.resetHeading();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  frc2::JoystickButton(&m_driverController, 1)
      .OnTrue(frc2::CommandPtr((frc2::RunCommand([this] {
        return m_swerveDrive.resetHeading();
      })))); // TODO assign as test
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous

  // return autos::ExampleAuto(&m_subsystem);
  return frc2::CommandPtr(frc2::InstantCommand());
}

void RobotContainer::UpdateDashboard() {
  frc::SmartDashboard::PutNumber("driver X", m_driverController.GetX());
  frc::SmartDashboard::PutNumber(
      "adjusted X",
      MathUtilNK::calculateAxis(
          m_driverController.GetX(), GeneralConstants::kDefaultAxisDeadband,
          GeneralConstants::kDriveLimit *
              GeneralConstants::kMaxTranslationalVelocity));
}
