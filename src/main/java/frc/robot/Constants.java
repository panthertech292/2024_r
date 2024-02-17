// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;

import edu.wpi.first.math.util.Units;

import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kVoltageComp = 12.0;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveControllerDeadZone = 0.10;
  }

  public static class IntakeConstants {
    public static final int kLowIntakeMotorID = 21;
    public static final int kUpIntakeMotorID = 22;
  }
  public static class ShooterConstants {
    public static final int kBeltsLowMotorID = 31;
    public static final int kBeltsUpMotorID = 32;
    public static final int kRotateMotorID = 33;
    public static final int kShooterLowMotorID = 34;
    public static final int kShooterUpMotorID = 35;
  }

  //TODO: Fix this garbage
  public static final class SwerveConstants{
    //Hardware Setup
    public static final int kPigeon2ID = 10;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)
    //Front Left Module 0
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(342.3);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    //Front Right Module 1
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(358.76); //Might be drifting?
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    //Back Left Module 2
    public static final class Mod2 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 15;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(24.8);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    //Back Right Module 3
    public static final class Mod3 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 17;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(96.4);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    //Swerve non-module specific constants
    //Drivetrain constants
    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    public static final double trackWidth = Units.inchesToMeters(24.50); 
    public static final double wheelBase = Units.inchesToMeters(24.50); 
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    //Swerve Kinematics - No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    //Gear ratios
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    //Speeds - Swerve Profiling Values
    //Meters per Second */
    public static final double maxSpeed = 4.1; //TODO: This must be tuned to specific robot
    public static final double maxAccel = 4.1; //TODO: This must be tuned to specific robot

    //Radians per Second
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

    //Inverts
    //Angle CANEncoder Invert
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;
    //Motor Inverts
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    //Neutral Modes
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    //Current Limits
    //Angle
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;
    //Drive
    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    //PID Values
    //Angle
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;
    //Drive
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    //Drive Motor System Identification/Characterization
    //TODO: Update these values https://docs.wpilib.org/en/2022/docs/software/pathplanning/system-identification/introduction.html
    //TODO: Check if dividing by 12 is right. The template code says "Divide SYSID values by 12 to convert from volts to percent output for CTRE"
    public static final double driveKS = (0.16861 / 12); //TODO: This must be tuned to specific robot. 
    public static final double driveKV = (2.6686 / 12); 
    public static final double driveKA = (0.34757 / 12);

    //Motor conversion factors
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
  }
}
