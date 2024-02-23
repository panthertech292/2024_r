// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RotateShooter extends Command {
  private final ShooterSubsystem ShooterSub;
  private double speed;
  /** Creates a new RotateShooter. */
  public RotateShooter(ShooterSubsystem s_ShooterSubsystem, double speed) {
    ShooterSub = s_ShooterSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ShooterSub.getShooterAngle() < 0.009 && speed < 0){
        ShooterSub.rotateShooter(this.speed/4);
      }else{
        ShooterSub.rotateShooter(this.speed);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.rotateShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
