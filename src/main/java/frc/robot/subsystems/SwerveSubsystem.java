// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    gyro = new Pigeon2(Constants.SwerveConstants.kPigeon2ID);
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    Timer.delay(1.0); //By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
    resetModulesToAbsolute(); //See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.

    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
  }
  
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleID], isOpenLoop);
    }
  }
  //Used by SwerveControllerCommand in Auto - This is for auto drive
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
    
    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(desiredStates[mod.moduleID], true);
    }
  }
  //   - TODO: Figure out what these are for/needed (probably auto)
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(new Rotation2d(gyro.getYaw(),gyro.getYaw()), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : mSwerveMods){
        states[mod.moduleID] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleID] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro(){
    gyro.setYaw(0);
  }
  public Rotation2d getYaw() {
    return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(180 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
}
  public void resetModulesToAbsolute(){
    for(SwerveModule mod : mSwerveMods){
        mod.resetToAbsolute();
    }
  }
  public void rotateToDegree(double target){
    PIDController rotController = new PIDController(.1,0.0008,0.001);
    rotController.enableContinuousInput(-180, 180);

    double rotate = rotController.calculate(gyro.getYaw(), target);

    drive(new Translation2d(0, 0), -.25*rotate, false, true);        
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdometry.update(getYaw(), getModulePositions()); 
    for(SwerveModule mod : mSwerveMods){
      SmartDashboard.putNumber(mod.moduleID + "Angle:", mod.getCanCoder().getDegrees());
      //SmartDashboard.putNumber(mod.moduleID + "Distance to Offset", mod.getOffset().getDegrees() - mod.getCanCoder().getDegrees());
    }
  }
}
