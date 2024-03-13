// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.LimelightHelpers;

public class driveAimAtSpeakerPose extends Command {
  private final SwerveSubsystem SwerveSub;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private Translation2d targetSpeaker;
  /** Creates a new driveAimmAtTarget. */
  public driveAimAtSpeakerPose(SwerveSubsystem s_SwerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
    this.SwerveSub = s_SwerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    targetSpeaker = new Translation2d();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      targetSpeaker = FieldConstants.kSpeakerPositionBLUE; //Set to aim at blue speaker
    }else{
      targetSpeaker = FieldConstants.kSpeakerPositionRED; //Set to aim at red speaker
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SwerveSub.getPose().
    //targetSpeaker.
    double distanceX = SwerveSub.getPose().getX() - targetSpeaker.getX();
    double distanceY = SwerveSub.getPose().getY() - targetSpeaker.getY();

    Math.atan2(distanceY, distanceX);
    //add 90 degrees?
    //SwerveSub.driveCommand(translationX, translationY);
    //SwerveSub.drive(new Translation2d(translationX.getAsDouble() * SwerveSub.getMaxVelocity(), translationY.getAsDouble() * SwerveSub.getMaxVelocity()), headingVelocity * SwerveSub.getMaxAngularVelocity(), true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
