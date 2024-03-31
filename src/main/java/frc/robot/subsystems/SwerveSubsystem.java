// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utilities.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive RobotSwerve;

  //Initialize {@link SwerveDrive} with the directory provided. @param directory Directory of swerve drive config files.
  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW; // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.

    //Create RobotSwerve
    try{
      RobotSwerve = new SwerveParser(directory).createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (Exception e){
      throw new RuntimeException(e);
    }
    RobotSwerve.setHeadingCorrection(false);
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
        AutoConstants.kTranslationPID,
                                         // Translation PID constants
                                         AutoConstants.kAnglePID,
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

  public Command getAutonomousCommand(String pathName){
    return new PathPlannerAuto(pathName); // Create a path following command using AutoBuilder. This will also trigger event markers.
  }

  /** Use PathPlanner Path finding to go to a point on the field.
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command*/
  public Command driveToPose(Pose2d pose){
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(RobotSwerve.getMaximumVelocity(), 4.0,RobotSwerve.getMaximumAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(pose, constraints,0.0, 0.0);
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY){
    RobotSwerve.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.

    return run(() -> {
      double xInput = translationX.getAsDouble();
      double yInput = translationY.getAsDouble();
      if(isAllianceBlue()){
        driveFieldOriented(RobotSwerve.swerveController.getTargetSpeeds(xInput, yInput, -headingX.getAsDouble(), -headingY.getAsDouble(), RobotSwerve.getOdometryHeading().getRadians(), RobotSwerve.getMaximumVelocity()));
      }else{
        driveFieldOriented(RobotSwerve.swerveController.getTargetSpeeds(-xInput, -yInput, headingX.getAsDouble(), headingY.getAsDouble(), RobotSwerve.getOdometryHeading().getRadians(), RobotSwerve.getMaximumVelocity()));
      }
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Angular velocity of the robot to set. CUBED TODO: Test if we want this cubed!
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    RobotSwerve.setHeadingCorrection(false);
    return run(() -> {
      if(isAllianceBlue()){
        RobotSwerve.drive(new Translation2d(translationX.getAsDouble() * RobotSwerve.getMaximumVelocity(), translationY.getAsDouble() * RobotSwerve.getMaximumVelocity()), Math.pow(angularRotationX.getAsDouble(), 3) * RobotSwerve.getMaximumAngularVelocity(),true,false);
      }else{
        RobotSwerve.drive(new Translation2d(-translationX.getAsDouble() * RobotSwerve.getMaximumVelocity(), -translationY.getAsDouble() * RobotSwerve.getMaximumVelocity()), Math.pow(angularRotationX.getAsDouble(), 3) * RobotSwerve.getMaximumAngularVelocity(),true,false);
      }
      // Make the robot move
      
    });
  }

  /**
   * The primary method for controlling the drivebase. //TODO: Use this in alignment commands
   * 
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in m/s. In robot-relative mode, positive x is torwards front and positive y is torwards left.  In field-relative mode, positive x is away from the alliance wall (field North) and positive y is torwards the left wall when looking through the driver station glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot, relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative){
    RobotSwerve.drive(translation, rotation, fieldRelative,false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void headingDrive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY){
    double xInput = translationX.getAsDouble();
    double yInput = translationY.getAsDouble();

    if(isAllianceBlue()){
      driveFieldOriented(RobotSwerve.swerveController.getTargetSpeeds(xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble(), RobotSwerve.getOdometryHeading().getRadians(), RobotSwerve.getMaximumVelocity()));
    }else{
      driveFieldOriented(RobotSwerve.swerveController.getTargetSpeeds(-xInput, -yInput, headingX.getAsDouble(), headingY.getAsDouble(), RobotSwerve.getOdometryHeading().getRadians(), RobotSwerve.getMaximumVelocity()));
    }
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity){
    RobotSwerve.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity){
    RobotSwerve.drive(velocity);
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose){
    RobotSwerve.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose(){
    return RobotSwerve.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    RobotSwerve.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro(){
    RobotSwerve.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake){
    RobotSwerve.setMotorIdleMode(brake);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity(){
    return RobotSwerve.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity(){
    return RobotSwerve.getRobotVelocity();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   * @return The yaw angle
   */
  public Rotation2d getHeading(){
    return getPose().getRotation();
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock(){
    RobotSwerve.lockPose();
  }
  public double getMaxVelocity(){
    return RobotSwerve.getMaximumVelocity();
  }

  public double getMaxAngularVelocity(){
    return RobotSwerve.getMaximumAngularVelocity();
  }

  public void updateVisionOdometry(){
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(limelightMeasurement.tagCount >= 2){
      RobotSwerve.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
      //RobotSwerve.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds); //TODO: THIS MIGHT BE WORSE!
    }
  }

  public double getDistanceFromSpeaker(){
    Translation2d target;
    target = getTargetSpeaker();
    return target.getDistance(RobotSwerve.getPose().getTranslation());
  }

  public boolean isAllianceBlue(){ // TODO: This might be system intensive? I don't know. Need to test.
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Blue){
        return true;
      }else{
        return false;
      }
    }else{
      //System.out.println("Warning: Swerve Subsystem: Cannot get alliance from FMS/Driverstation! Defaulting to blue!");
      return true;
    }
  }

  public Translation2d getTargetSpeaker(){ 
    if(isAllianceBlue()){
      return FieldConstants.kSpeakerPositionBLUE;
    }else{
      return FieldConstants.kSpeakerPositionRED;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateVisionOdometry();
    SmartDashboard.putNumber("DISTANCE TO SPEAKER (METERS)" , getDistanceFromSpeaker());
    SmartDashboard.putNumber("Raw Encoder (Back Left): " , RobotSwerve.getModuleMap().get("backleft").getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Raw Encoder (Back Right): " , RobotSwerve.getModuleMap().get("backright").getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Raw Encoder (Front Left): " , RobotSwerve.getModuleMap().get("frontleft").getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Raw Encoder (Front Right): " , RobotSwerve.getModuleMap().get("frontright").getAbsoluteEncoder().getAbsolutePosition());
  }
}
