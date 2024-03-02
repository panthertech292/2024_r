// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRPMBelts extends Command {
  private final ShooterSubsystem ShooterSub;
  private double targetRPM;
  private double p;
  private double beltSpeed;
  /** Creates a new ShooterRPMBelts. */
  public ShooterRPMBelts(ShooterSubsystem s_ShooterSubsystem, double targetRPM, double p, double beltSpeed) {
    ShooterSub = s_ShooterSubsystem;
    this.targetRPM = targetRPM;
    this.p = p;
    this.beltSpeed = beltSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSub.setShooterPID(0.0004, 0.0001, 2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.setBelts(0);
    ShooterSub.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
