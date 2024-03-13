// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateToAngle extends Command {
  private final ArmSubsystem ArmSub;
  private double target;
  private double error;
  private double p;
  private double minSpeed;
  /** Creates a new ArmRotateToAngle. */
  public ArmRotateToAngle(ArmSubsystem s_ArmSubsystem, double target, double p, double minSpeed) {
    ArmSub = s_ArmSubsystem;
    this.target = target;
    this.p = p;
    this.minSpeed = minSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = (target - ArmSub.getRotationAngle())*p;
    if (minSpeed > Math.abs(error)){//If we are running too slow, go at a min speed
      if (error > 0){//Checks sign of error, sets minspeed to either + or -
        error = minSpeed;
      }else{
        error = -minSpeed;
      }
    }
    ArmSub.setArmRotate(error);
    //System.out.println("Error: " + error);
    //System.out.println("Distance to Target: " + (target - ArmSub.getRotationAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSub.setArmRotate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if ((target <= ArmConstants.kRotationIntakeAngle) && (ArmSub.isArmDown())){
      return true;
    }else{
      return false;
    }
  }
}
