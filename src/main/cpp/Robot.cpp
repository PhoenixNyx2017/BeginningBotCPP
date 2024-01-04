// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/JoystickButton.h>

#include "SwerveDrive/commands/Drive.h"

Robot::Robot() { this->CreateRobot(); }

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  this->UpdateDashboard();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = this->GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

/**
 * Initializes the robot subsystems and binds commands
 */
void Robot::CreateRobot() {
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
  BindCommands();
  m_swerveDrive.resetHeading();
}

/**
 * Binds commands to Joystick buttons
 */
void Robot::BindCommands() {
  frc2::JoystickButton(&m_driverController, 1)
      .OnTrue(frc2::CommandPtr((frc2::RunCommand([this] {
        return m_swerveDrive.resetHeading();
      })))); // TODO assign as test
}

/**
 * Returns the Autonomous Command
 */
frc2::CommandPtr Robot::GetAutonomousCommand() {
  return frc2::CommandPtr(frc2::InstantCommand());
}

/**
 * Updates the data on the dashboard
 */
void Robot::UpdateDashboard() {
  frc::SmartDashboard::PutNumber("driver X", m_driverController.GetX());
  frc::SmartDashboard::PutNumber(
      "adjusted X",
      MathUtilNK::calculateAxis(
          m_driverController.GetX(), GeneralConstants::kDefaultAxisDeadband,
          GeneralConstants::kDriveLimit *
              GeneralConstants::kMaxTranslationalVelocity));
  frc::SmartDashboard::PutNumber("Swerve Drive Heading",
                                 m_swerveDrive.getHeading().Degrees().value());
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
