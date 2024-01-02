// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once
#include <cmath>
#include <cstdlib>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/acceleration.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include "SDSModuleType.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace GeneralConstants {
const int kDriverPort = 0;
const int kOperatorPort = 1;

const SDSModuleType mk3_standard{0.1016,
                                 (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
                                 true, (15.0 / 32.0) * (10.0 / 60.0), true};
const SDSModuleType mk3_fast{0.1016,
                             (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
                             true, (15.0 / 32.0) * (10.0 / 60.0), true};

const SDSModuleType mk4_l1{0.10033,
                           (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0), true,
                           (15.0 / 32.0) * (10.0 / 60.0), true};
const SDSModuleType mk4_l2{0.10033,
                           (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), true,
                           (15.0 / 32.0) * (10.0 / 60.0), true};
const SDSModuleType mk4_l3{0.10033,
                           (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0), true,
                           (15.0 / 32.0) * (10.0 / 60.0), true};
const SDSModuleType mk4_l4{0.10033,
                           (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0), true,
                           (15.0 / 32.0) * (10.0 / 60.0), true};

const SDSModuleType mk4i_l1{0.10033,
                            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0), true,
                            (14.0 / 50.0) * (10.0 / 60.0), false};
const SDSModuleType mk4i_l2{0.10033,
                            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), true,
                            (14.0 / 50.0) * (10.0 / 60.0), false};
const SDSModuleType mk4i_l3{0.10033,
                            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0), true,
                            (14.0 / 50.0) * (10.0 / 60.0), false};

const SDSModuleType kSDSModule = mk4i_l3;

const double kTrackwidthMeters = 0.4;
const double kWheelbaseMeters = 0.4;

const double kDefaultAxisDeadband = 0.15;
const double kMaxTranslationalVelocity = ((6380.0) / 60) *
                                         kSDSModule.wheelDiameter * M_PI *
                                         kSDSModule.driveReduction;

const double kMaxRotationalVelocity =
    (kMaxTranslationalVelocity) /
    std::hypot(kTrackwidthMeters / 2, kWheelbaseMeters / 2);
const bool kIsFieldRelative = true;
const bool kIsOpenLoop = false;

const int kFrontLeftDriveMotorID = 10;
const int kFrontLeftTurnMotorID = 11;
const int kFrontLeftEncoderID = 12;
const frc::Rotation2d kFrontLeftOffset =
    frc::Rotation2d(units::degree_t{103.711}); // module 1
//     public static final Rotation2d kFrontLeftOffset =
//     Rotation2d.fromDegrees(106.172); // 283.447.unaryMinus() // -76.201
const frc::Translation2d kFrontLeftPosition =
    frc::Translation2d(units::meter_t{kTrackwidthMeters / 2.0},
                       units::meter_t{kWheelbaseMeters / 2.0});

const int kFrontRightDriveMotorID = 20;
const int kFrontRightTurnMotorID = 21;
const int kFrontRightEncoderID = 22;
const frc::Rotation2d kFrontRightOffset =
    frc::Rotation2d(units::degree_t{318.427}); // 139.658 // 139.658 // module 2
//     public static final Rotation2d kFrontRightOffset =
//     Rotation2d.fromDegrees(318.164); // 139.658 // 139.658
const frc::Translation2d kFrontRightPosition =
    frc::Translation2d(units::meter_t{kTrackwidthMeters / 2.0},
                       units::meter_t{-kWheelbaseMeters / 2.0});

const int kBackLeftDriveMotorID = 30;
const int kBackLeftTurnMotorID = 31;
const int kBackLeftEncoderID = 32;
const frc::Rotation2d kBackLeftOffset =
    frc::Rotation2d(units::degree_t{275.273}); // 95.977 // 96.680 // module 3
//     public static final Rotation2d kBackLeftOffset =
//     Rotation2d.fromDegrees(-84.551);  // 95.977 // 96.680
const frc::Translation2d kBackLeftPosition =
    frc::Translation2d(units::meter_t{-kTrackwidthMeters / 2.0},
                       units::meter_t{kWheelbaseMeters / 2.0});

const int kBackRightDriveMotorID = 40;
const int kBackRightTurnMotorID = 41;
const int kBackRightEncoderID = 42;
const frc::Rotation2d kBackRightOffset =
    frc::Rotation2d(units::degree_t{87.539}); // 265.517 // -93.867 // module 4
//     public static final Rotation2d kBackRightOffset =
//     Rotation2d.fromDegrees(88.770); // 265.517 // -93.867
const frc::Translation2d kBackRightPosition =
    frc::Translation2d(units::meter_t{-kTrackwidthMeters / 2.0},
                       units::meter_t{-kWheelbaseMeters / 2.0});

const frc::SwerveDriveKinematics kSwerveKinematics =
    frc::SwerveDriveKinematics(kFrontLeftPosition, kFrontRightPosition,
                               kBackLeftPosition, kBackRightPosition);

const double kDriveLimit = 0.15; // 0.7 fast
const double kRotationLimit = kDriveLimit;

} // namespace GeneralConstants

namespace ModuleConstants {

const double kMaxSpeed = GeneralConstants::kMaxTranslationalVelocity;
const double kWheelDiameterMeters = GeneralConstants::kSDSModule.wheelDiameter;
const double kWheelCircumference = kWheelDiameterMeters * M_PI;
// ratio is motor rot / wheel rot
const double kDriveGearRatio =
    1.0 / GeneralConstants::kSDSModule.driveReduction;
const double kTurnGearRatio = 1.0 / GeneralConstants::kSDSModule.steerReduction;

const bool kDriveMotorInverted = GeneralConstants::kSDSModule.driveInverted;
const ctre::phoenix::motorcontrol::NeutralMode kDriveMotorNeutral =
    ctre::phoenix::motorcontrol::NeutralMode::Coast;
const double kDriveVoltageComp = 12;

const bool kTurnMotorInverted = true;
const ctre::phoenix::motorcontrol::NeutralMode kTurnMotorNeutral =
    ctre::phoenix::motorcontrol::NeutralMode::Brake; // set back to brake to be
                                                     // amazing
const double kTurnVoltageComp = 12;
const bool kEncoderInverted = false;

const double kOpenLoopRamp = 1.00;
const double kClosedLoopRamp = 0.75;

const bool kTurnEnableCurrentLimit = true;
const int kTurnContinuousCurrentLimit = 25;
const int kTurnPeakCurrentLimit = 40;
const double kTurnPeakCurrentDuration = 0.1;

const bool kDriveEnableCurrentLimit = true;
const int kDriveContinuousCurrentLimit = 35;
const int kDrivePeakCurrentLimit = 60;
const double kDrivePeakCurrentDuration = 0.1;

const auto kDriveS = 0.05558_V;
const auto kDriveV = 0.20333_V / 1_mps;
const auto kDriveA = 0.02250_V / 1_mps_sq;

const double kDriveP = 0.10;
const double kDriveI = 0.0;
const double kDriveD = 0.0;
const double kDriveF = 0; // 0.25 / 0.54 * 0.1;

const double kTurnP = 0.6;
const double kTurnI = 0;
const double kTurnD = 12.0; // 12.0
const double kTurnF = 0.0;

} // namespace ModuleConstants

namespace MathUtilNK {
inline double calculateAxis(double axis, double deadband, double scalar) {
  double res = axis;

  if (std::abs(axis) > deadband) {
    res = (axis - std::copysign(deadband, axis)) / (1.0 - deadband);
  } else {
    res = 0.0;
  }

  return res * scalar;
}

} // namespace MathUtilNK

namespace Conversions {

const double kSecondsPerMinute = 60.0;
const double kFalconTicks = 2048.0;
const double kMaxFalconRPM = 6380.0;
const double kFalconToDegrees = 360.0 / kFalconTicks;
const double kFalconToRotations = 1.0 / kFalconTicks;

// starting units: ticks/100ms
// ticks/100ms * (1000ms/s * 60s/m) * 1 rot/ 2048 ticks = 600 ticks/m * 1
// rot/ticks
const double kFalconVelocityToRPM = 600.0 / kFalconTicks;

inline double falconToDegrees(double counts, double gearRatio) {
  // ratio = motor/wheel
  return counts * kFalconToDegrees / gearRatio;
}

inline double degreesToFalcon(double degrees, double gearRatio) {
  double ticks = degrees * gearRatio / kFalconToDegrees;
  return ticks;
}

/**
 * Converts a falcon motor position into distance traveled
 *
 * @param falconPosition falcon position sensor counts
 * @param circumference  wheel circumference in meters
 * @param gearRatio      motor rotations/wheel rotations
 * @return distance traveled in meters
 */
inline double falconToMeters(double falconPosition, double circumference,
                             double gearRatio) {
  double wheelRotations = falconPosition * kFalconToRotations / gearRatio;
  double distance = wheelRotations * circumference;
  return distance;
}

inline double falconToRPM(double velocityCounts, double gearRatio) {
  double motorRPM = velocityCounts * kFalconVelocityToRPM;
  double mechRPM = motorRPM / gearRatio;
  return mechRPM;
}

inline double RPMToFalcon(double RPM, double gearRatio) {
  double motorRPM = RPM * gearRatio;
  double sensorCounts = motorRPM / kFalconVelocityToRPM;
  return sensorCounts;
}

inline double falconToMPS(double velocitycounts, double circumference,
                          double gearRatio) {
  double wheelRPM = falconToRPM(velocitycounts, gearRatio);
  double wheelMPS = (wheelRPM * circumference) / kSecondsPerMinute;
  return wheelMPS;
}

inline double MPSToFalcon(double velocity, double circumference,
                          double gearRatio) {
  double wheelRPM = ((velocity * kSecondsPerMinute) / circumference);
  double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
  return wheelVelocity;
}
} // namespace Conversions
