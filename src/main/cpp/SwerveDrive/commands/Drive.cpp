// Copyright (c) FRC Team 122. All Rights Reserved.

#include "SwerveDrive/commands/Drive.h"

Drive::Drive(SwerveDrive *i_swerve, std::function<double()> i_x,
             std::function<double()> i_y, std::function<double()> i_rot)
    : swerve(i_swerve), x(i_x), y(i_y), rot(i_rot) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve);
}

// Called when the command is initially scheduled.
void Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {

  if (GeneralConstants::kIsFieldRelative) {
    speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{x()}, units::meters_per_second_t{y()},
        units::radians_per_second_t{rot()}, swerve->getHeading());
  } else {
    speeds = frc::ChassisSpeeds{units::meters_per_second_t{x()},
                                units::meters_per_second_t{y()},
                                units::radians_per_second_t{rot()}};
  }

  // System.out.println("DriveCommand(" + xSpeed + ", " + ySpeed + ", " +
  // thetaSpeed + ")");
  swerve->drive(speeds, GeneralConstants::kIsOpenLoop);
  frc::SmartDashboard::PutNumber("Inside?", x());
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {
  swerve->drive(frc::ChassisSpeeds(), GeneralConstants::kIsOpenLoop);
}

// Returns true when the command should end.
bool Drive::IsFinished() { return false; }
