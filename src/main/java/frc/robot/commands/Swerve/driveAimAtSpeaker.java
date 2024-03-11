// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.LimelightHelpers;

public class driveAimAtSpeaker extends Command {
  private final SwerveSubsystem SwerveSub;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private double headingVelocity;
  /** Creates a new driveAimmAtTarget. */
  public driveAimAtSpeaker(SwerveSubsystem s_SwerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
    this.SwerveSub = s_SwerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      LimelightHelpers.setPriorityID("limelight", 7); //Set to aim at blue speaker
    }else{
      LimelightHelpers.setPriorityID("limelight", 4); //Set to aim at red speaker
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get the heading velocity to turn to
    if(LimelightHelpers.getTV("limelight")){
      headingVelocity = -LimelightHelpers.getTX("limelight")/70;
      RobotContainer.setRightRumbleDriver(0);
    }else{ //We don't have a target
      System.out.println("Warning: Swerve Aim: Lost Target!");
      RobotContainer.setRightRumbleDriver(1);
      headingVelocity = 0;
    }
    SwerveSub.drive(new Translation2d(translationX.getAsDouble() * SwerveSub.getMaxVelocity(), translationY.getAsDouble() * SwerveSub.getMaxVelocity()), headingVelocity * SwerveSub.getMaxAngularVelocity(), true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.setRightRumbleDriver(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
