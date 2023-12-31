// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveDrive/SwerveModule.h"


SwerveModule::SwerveModule(int driveMotorId, int steerMotorId, int steerEncoderId, frc::Rotation2d angleOffset) :
    driveMotor(driveMotorId),
    steerMotor(steerMotorId),
    steerEncoder(steerEncoderId),
    angleOffset(angleOffset)
    // feedForward()
{
    id = (driveMotorId / 10);
    
    initEncoder(steerEncoderId);
    initDriveMotor(driveMotorId);
    initTurnMotor(steerMotorId);

    resetToAbsolute();

}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {}

void SwerveModule::resetToAbsolute() {
    lastAngle = double {(getCANCoder() - angleOffset).Degrees()};
    double absolutePosition = Conversions::degreesToFalcon(lastAngle, ModuleConstants::kTurnGearRatio);
    steerMotor.SetSelectedSensorPosition(absolutePosition);

}

frc::SwerveModuleState SwerveModule::getCurrentState(){
    double velocity = getVelocityMPS();
    frc::Rotation2d angle = getAngleRotation2d();
    return frc::SwerveModuleState(units::meters_per_second_t {velocity}, angle);
}

frc::SwerveModulePosition SwerveModule::getPosition() {
    double distance = getDistanceMeters();
    frc::Rotation2d angle = getAngleRotation2d();
    return frc::SwerveModulePosition(units::meter_t {distance}, angle);
}

void SwerveModule::setDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop) {
    if (steerMotor.HasResetOccurred()) {
            resetToAbsolute();
        }

    frc::Rotation2d currentAngleRotation2d = getAngleRotation2d();

    //TODO: May throw error
    desiredState = frc::SwerveModuleState::Optimize(desiredState, currentAngleRotation2d);

    desiredState = checkForWrapAround(desiredState, currentAngleRotation2d);

    // sets Angle and velocity of the wheels
    setAngle(desiredState, currentAngleRotation2d);
    setVelocity(desiredState, GeneralConstants::kIsOpenLoop);
}

void SwerveModule::resetDriveEncoders() {
    driveMotor.SetSelectedSensorPosition(0, 0, 0);
}

void SwerveModule::setAngle(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngleRotation2d) {
    double angle = double {desiredState.angle.Degrees()};
    if (abs(double {desiredState.speed}) >= (ModuleConstants::kMaxSpeed * 0.01)) {
        angle = double {desiredState.angle.Degrees()};
    }
    else {
        angle = lastAngle;
    }

    if (std::abs(double {((currentAngleRotation2d - desiredState.angle).Degrees())}) >
            double {ModuleConstants::kAllowableAngleTolerance.Degrees()}) {
        steerMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, Conversions::degreesToFalcon(angle, ModuleConstants::kTurnGearRatio));
    }
    else {
        steerMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    }
    lastAngle = angle;
}
    
void SwerveModule::setVelocity(frc::SwerveModuleState desiredState, bool isOpenLoop){
    if (isOpenLoop) {
        double percentOutput = double {desiredState.speed} / ModuleConstants::kMaxSpeed;
        driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, percentOutput);
    }
    else {
        double velocity = Conversions::MPSToFalcon(double {desiredState.speed},
            ModuleConstants::kWheelCircumference, ModuleConstants::kDriveGearRatio);
        double ffCalculate = ModuleConstants::kDriveS * double {wpi::sgn(double {desiredState.speed})} +
                            ModuleConstants::kDriveV * double {desiredState.speed};

        driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, velocity, 
            ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, 
            ffCalculate);
    }

}

void SwerveModule::initDriveMotor(int driveMotorID){
    driveMotor.ConfigFactoryDefault();
    
    kDriveMotorConfig.slot0.kP = ModuleConstants::kDriveP;
    kDriveMotorConfig.slot0.kI = ModuleConstants::kDriveI;
    kDriveMotorConfig.slot0.kD = ModuleConstants::kDriveD;
    kDriveMotorConfig.slot0.kF = ModuleConstants::kDriveF;
    kDriveMotorConfig.supplyCurrLimit = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(
        ModuleConstants::kDriveEnableCurrentLimit,
        ModuleConstants::kDriveContinuousCurrentLimit,
        ModuleConstants::kDrivePeakCurrentLimit,
        ModuleConstants::kDrivePeakCurrentDuration);
    kDriveMotorConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;
    kDriveMotorConfig.openloopRamp = ModuleConstants::kOpenLoopRamp;
    kDriveMotorConfig.closedloopRamp = ModuleConstants::kClosedLoopRamp;
    kDriveMotorConfig.voltageCompSaturation = ModuleConstants::kDriveVoltageComp;
    kDriveMotorConfig.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_10Ms;

    driveMotor.ConfigAllSettings(kDriveMotorConfig);
    driveMotor.SetInverted(ModuleConstants::kDriveMotorInverted);
    driveMotor.SetNeutralMode(ModuleConstants::kDriveMotorNeutral);
    driveMotor.SetSelectedSensorPosition(0);
}

void SwerveModule::initTurnMotor(int turnMotorID){
    steerMotor.ConfigFactoryDefault();

    kTurnMotorConfig.primaryPID.selectedFeedbackSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
    kTurnMotorConfig.primaryPID.selectedFeedbackCoefficient = 1;
    kTurnMotorConfig.slot0.kP = ModuleConstants::kTurnP;
    kTurnMotorConfig.slot0.kI = ModuleConstants::kTurnI;
    kTurnMotorConfig.slot0.kD = ModuleConstants::kTurnD;
    kTurnMotorConfig.slot0.kF = ModuleConstants::kTurnF;
    kTurnMotorConfig.supplyCurrLimit = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(
            ModuleConstants::kTurnEnableCurrentLimit,
            ModuleConstants::kTurnContinuousCurrentLimit,
            ModuleConstants::kTurnPeakCurrentLimit,
            ModuleConstants::kTurnPeakCurrentDuration);

    kTurnMotorConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;;
    kTurnMotorConfig.voltageCompSaturation = ModuleConstants::kTurnVoltageComp;


    steerMotor.ConfigAllSettings(kTurnMotorConfig);
    steerMotor.SetInverted(ModuleConstants::kTurnMotorInverted);
    steerMotor.SetNeutralMode(ModuleConstants::kTurnMotorNeutral);
    steerMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);
    
    
    resetToAbsolute();

}

void SwerveModule::initEncoder(int encoderID) {
    steerEncoder.ConfigFactoryDefault();

    kEncoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    kEncoderConfig.sensorDirection = ModuleConstants::kEncoderInverted;
    kEncoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
    kEncoderConfig.sensorTimeBase = ctre::phoenix::sensors::SensorTimeBase::PerSecond;
    
    steerEncoder.ConfigAllSettings(kEncoderConfig);

}

void SwerveModule::updateSmartDash() {
    frc::SmartDashboard::PutNumber(std::to_string(id) + "Last Angle", lastAngle);
    frc::SmartDashboard::PutNumber(std::to_string(id) + "PureRaw Angle", double {getAbsoluteAngle().Degrees()});

    frc::SmartDashboard::PutNumber(std::to_string(id) + "Magnet offset", double {angleOffset.Degrees()});

}

frc::SwerveModuleState SwerveModule::checkForWrapAround(frc::SwerveModuleState desiredState, frc::Rotation2d currentState){
    double desiredDegrees = double {desiredState.angle.Degrees()};
    double speedMulti;
    double remainder = std::remainder(desiredDegrees - double {currentState.Degrees()}, 180);
    double newPos = remainder + double {currentState.Degrees()};
    double minDist = std::abs(std::remainder(newPos - desiredDegrees, 180));

    if (minDist < 0.001){
        speedMulti = 1;
    }
    else {
        speedMulti = -1;
    }

    desiredState.angle = frc::Rotation2d(units::degree_t {newPos});
    desiredState.speed = desiredState.speed * speedMulti;
    return desiredState;
}

frc::Rotation2d SwerveModule::getCANCoder() {
    return frc::Rotation2d(units::degree_t {steerEncoder.GetAbsolutePosition()});
}

frc::Rotation2d SwerveModule::getAngleRotation2d(){
    return  frc::Rotation2d(units::degree_t {
                Conversions::falconToDegrees(steerMotor.GetSelectedSensorPosition(), ModuleConstants::kTurnGearRatio)});

}

frc::Rotation2d SwerveModule::getAbsoluteAngle(){
    return frc::Rotation2d(units::degree_t {steerEncoder.GetAbsolutePosition()});

}

double SwerveModule::getDistanceMeters(){
    double distance = Conversions::falconToMeters(driveMotor.GetSelectedSensorPosition(),
                ModuleConstants::kWheelCircumference, ModuleConstants::kDriveGearRatio);
    return distance;

}

double SwerveModule::getVelocityMPS(){
    return Conversions::falconToMPS(driveMotor.GetSelectedSensorVelocity(), ModuleConstants::kWheelCircumference,
                ModuleConstants::kDriveGearRatio);
}

void SwerveModule::setTurnCoast(){
    steerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void SwerveModule::setTurnBrake(){
    steerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void SwerveModule::setOffset(double offset){
    angleOffset = frc::Rotation2d(units::degree_t{offset});
    resetToAbsolute();

}

