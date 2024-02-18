// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeStore extends Command {
  private final ShooterSubsystem ShooterSub;
  private final IntakeSubsystem IntakeSub;
  /** Creates a new IntakeStore. */
  public IntakeStore(ShooterSubsystem s_ShooterSubsystem, IntakeSubsystem s_IntakeSubsystem) {
    ShooterSub = s_ShooterSubsystem;
    IntakeSub = s_IntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem, s_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Run the shooter down if not in position
    if (!ShooterSub.getRotateSwitch()){
      ShooterSub.rotateShooter(-ShooterConstants.kRotateSpeed);
    }else{
      ShooterSub.rotateShooter(0);
      IntakeSub.setIntake(IntakeConstants.kIntakeSpeed);
      ShooterSub.setBelts(ShooterConstants.kIntakeBeltSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSub.setIntake(0);
    ShooterSub.setBelts(0);
    ShooterSub.rotateShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSub.getBeltSwitch();
  }
}
