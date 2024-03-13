// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotate extends Command {
  private final ArmSubsystem ArmSub;
  private double speed;
  /** Creates a new ArmRotate. */
  public ArmRotate(ArmSubsystem s_ArmSubsystem, double speed) {
    ArmSub = s_ArmSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //ArmSub.setArmRotate(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ArmSub.getRotationAngle() < ArmConstants.kRotationQuarterSpeedAngle && speed < 0){
      ArmSub.setArmRotate(this.speed/4, true);
    }else{
      ArmSub.setArmRotate(this.speed, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSub.setArmRotate(0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
