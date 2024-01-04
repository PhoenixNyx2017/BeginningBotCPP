// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "SwerveDrive/SwerveDrive.h"
#include "SwerveDrive/SwerveModule.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Drive : public frc2::CommandHelper<frc2::CommandBase, Drive> {
public:
  Drive(SwerveDrive *swerve, std::function<double()> x,
        std::function<double()> y, std::function<double()> rot);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  SwerveDrive *swerve;
  frc::ChassisSpeeds speeds;
  std::function<double()> x;
  std::function<double()> y;
  std::function<double()> rot;
  // double x;
  // double y;
  // double rot;
};
