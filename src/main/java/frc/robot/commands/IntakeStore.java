// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeStore extends Command {
  private final ShooterSubsystem ShooterSub;
  private final IntakeSubsystem IntakeSub;
  private final ArmSubsystem ArmSub;
  /** Creates a new IntakeStore. */
  public IntakeStore(ShooterSubsystem s_ShooterSubsystem, IntakeSubsystem s_IntakeSubsystem, ArmSubsystem s_ArmSubsystem) {
    ShooterSub = s_ShooterSubsystem;
    IntakeSub = s_IntakeSubsystem;
    ArmSub = s_ArmSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem, s_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //If the arm is not down, run it down, else stop
    if(!ArmSub.isArmDown()){
      ArmSub.setArmRotate(-ArmConstants.kRotationSpeed);
    }else{
      ArmSub.setArmRotate(0);
    }
    //The arm is down enough to intake into the shooter, run
    if(ArmSub.isArmReadyToIntake()){
      IntakeSub.setIntake(IntakeConstants.kIntakeSpeed);
      ShooterSub.setFeedBelts(ShooterConstants.kIntakeBeltSpeed);
    }else{
      IntakeSub.setIntake(0);
      ShooterSub.setFeedBelts(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSub.setIntake(0);
    ShooterSub.setFeedBelts(0);
    ArmSub.setArmRotate(0);
  }

  // Returns true when the command should end. Stops when a note is detected in the shooter.
  @Override
  public boolean isFinished() {
    return ShooterSub.getFeedBeltSwitch();
  }
}
