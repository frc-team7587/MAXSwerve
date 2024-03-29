// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
    private final SwerveModule frontLeftModule = new SwerveModule(
        DriveConstants.kFrontLeftDrivingMotorID,
        DriveConstants.kFrontLeftTurningMotorID,
        DriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule rearLeftModule = new SwerveModule(
        DriveConstants.kRearLeftDrivingMotorID,
        DriveConstants.kRearLeftTurningMotorID,
        DriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule frontRightModule = new SwerveModule(
        DriveConstants.kFrontRightDrivingMotorID,
        DriveConstants.kFrontRightTurningMotorID,
        DriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule rearRightModule = new SwerveModule(
        DriveConstants.kRearRightDrivingMotorID,
        DriveConstants.kRearRightTurningMotorID,
        DriveConstants.kBackRightChassisAngularOffset);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions()
    );

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
        resetHeading();
    }

    /**
     * Drives the robot.
     * @param xSpeed                Speed of the robot in the x direction (forward).
     * @param ySpeed                Speed of the robot in the y direction (sideways).
     * @param rotation              Angular rate of the robot.
     * @param isFieldRelative       Whether the provided x and y speeds are relative to the field.
     * @param isRateLimitEnabled    Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean isFieldRelative, boolean isRateLimitEnabled) {
        double xSpeedCommand;
        double ySpeedCommand;

        if (isRateLimitEnabled) {
            // Convert XY to polar for rate limiting
            double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;
            double angleDifference = SwerveUtils.angleDifference(inputTranslationDirection, currentTranslationDirection);
            if (angleDifference < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(
                    currentTranslationDirection,
                    inputTranslationDirection,
                    directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if (angleDifference > 0.85 * Math.PI) {
                if (currentTranslationMagnitude > 1e-4) { // some small number to avoid floating-point errors with equality checking
                    // currentTranslationDirection is unchanged
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection = SwerveUtils.wrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(
                    currentTranslationDirection,
                    inputTranslationDirection,
                    directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
            }
            previousTime = currentTime;

            xSpeedCommand = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommand = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(rotation);

        } else {
            xSpeedCommand = xSpeed;
            ySpeedCommand = ySpeed;
            currentRotation = rotation;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommand * DriveConstants.kMaxSpeed;
        double ySpeedDelivered = ySpeedCommand * DriveConstants.kMaxSpeed;
        double rotationDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotationDelivered,
                    getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));
        
        setModuleStates(desiredStates);
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());
    }

    /**
     * Returns the angle of the robot reported by the gyroscope.
     * @return The angle of the robot.
     */
    public double getAngle() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /**
     * Returns the Rotation2d of the robot reported by the gyroscope.
     * @return The Rotation2d of the robot.
     */
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * Returns the pose of the robot.
     * @return The pose of the robot.
     */
    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the turn rate of the robot.
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the positions of the modules.
     * @return The positions of the modules.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        };
    }
    /**
     * Resets the odometry to the specified pose.
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public void setX() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeed);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        rearLeftModule.setDesiredState(desiredStates[2]);
        rearRightModule.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeftModule.resetEncoder();
        rearLeftModule.resetEncoder();
        frontRightModule.resetEncoder();
        rearRightModule.resetEncoder();
    }

    /** Resets the heading of the robot to zero. */
    public void resetHeading() {
        gyro.reset();
    }
}
