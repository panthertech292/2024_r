// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive RobotSwerve;

  //Initialize {@link SwerveDrive} with the directory provided. @param directory Directory of swerve drive config files.
  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.

    //Create RobotSwerve
    try{
      RobotSwerve = new SwerveParser(directory).createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (Exception e){
      throw new RuntimeException(e);
    }
    RobotSwerve.setHeadingCorrection(false); //TODO: Try turning this on!
    RobotSwerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    setupPathPlanner();
  }

  //Setup AutoBuilder for PathPlanner.
  public void setupPathPlanner(){
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        AutoConstants.TRANSLATION_PID,
                                         // Translation PID constants
                                         AutoConstants.ANGLE_PID,
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         RobotSwerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                  );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
