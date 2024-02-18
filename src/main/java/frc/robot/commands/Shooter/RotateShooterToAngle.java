// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RotateShooterToAngle extends Command {
  private final ShooterSubsystem ShooterSub;
  private double target;
  private double error;
  private double p;
  private double minSpeed;
  /** Creates a new RotateShooterToAngle. */
  public RotateShooterToAngle(ShooterSubsystem s_ShooterSubsystem, double target, double p, double minSpeed) {
    ShooterSub = s_ShooterSubsystem;
    this.target = target;
    this.p = p;
    this.minSpeed = minSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = (target - ShooterSub.getShooterAngle())*p;
    if (minSpeed > Math.abs(error)){//If we are running too slow, go at a min speed
      if (error > 0){//Checks sign of error, sets minspeed to either + or -
        error = minSpeed;
      }else{
        error = -minSpeed;
      }
    }
    ShooterSub.rotateShooter(error);
    System.out.println("Error: " + error);
    System.out.println("Distance to Target: " + (target - ShooterSub.getShooterAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.rotateShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(target - ShooterSub.getShooterAngle()) < 0.005);
  }
}
