// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbRun extends Command {
  private final ClimbSubsystem ClimbSub;
  private double speed;
  /** Creates a new ClimberRun. */
  public ClimbRun(ClimbSubsystem s_ClimbSubsystem, double speed) {
    this.ClimbSub = s_ClimbSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ClimbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(speed < 0){
      ClimbSub.setClimbMotorCurrentLimit(75); //TODO: Move this to constants
      ClimbSub.setClimbMotor(speed); //Going up
    }else{
      if(!ClimbSub.getServoLocked()){ // Only run if not locked
        ClimbSub.setClimbMotorCurrentLimit(6);
        ClimbSub.setClimbMotor(speed);
      }else{
        ClimbSub.setClimbMotor(0);
        System.out.println("Error: Can't move arm. Arm is locked!");
      }
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimbSub.setClimbMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
