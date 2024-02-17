// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooter extends Command {
  private final ShooterSubsystem ShooterSub;
  private DoubleSupplier speed;
  /** Creates a new RevShooter. */
  public RevShooter(ShooterSubsystem s_ShooterSubsystem, DoubleSupplier speed) {
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
    ShooterSub.setShooter(this.speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
