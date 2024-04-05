// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbRotateServo extends Command {
  private final ClimbSubsystem ClimbSub;
  private double angle;
  /** Creates a new ClimberRun. */
  public ClimbRotateServo(ClimbSubsystem s_ClimbSubsystem, double angle) {
    this.ClimbSub = s_ClimbSubsystem;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ClimbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimbSub.setServoAngle(angle);;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
