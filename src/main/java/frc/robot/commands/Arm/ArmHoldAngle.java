// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ArmHoldAngle extends Command {
  private final ArmSubsystem ArmSub;
  private final SwerveSubsystem SwerveSub;
  private double error;
  private double p;
  private double minSpeed;
  /** Creates a new ArmHoldAngle. */
  public ArmHoldAngle(ArmSubsystem s_ArmSubsystem, SwerveSubsystem s_SwerveSubsystem, double p, double minSpeed) {
    ArmSub = s_ArmSubsystem;
    SwerveSub = s_SwerveSubsystem;
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
    //Check in case arm is below or at down
    if (!((ArmSub.getRotationAngle() < ArmConstants.kRotationIntakeAngle) || ArmSub.isArmDown())){
      //Check in case the robot is moving, if so, stop!
      if((Math.abs(SwerveSub.getRobotVelocity().vxMetersPerSecond) < 1) && (Math.abs(SwerveSub.getRobotVelocity().vyMetersPerSecond) < 1)){
        error = (ArmSub.getLastCommandedLocation() - ArmSub.getRotationAngle())*p;
        if (minSpeed > Math.abs(error)){//If we are running too slow, go at a min speed
          if (error > 0){//Checks sign of error, sets minspeed to either + or -
            error = minSpeed;
          }else{
            error = -minSpeed;
          }
        }
      }else{
        error = 0; //robot is moving
      }
      
    }else{ //The arm is down, so we don't need to hold it down
      error = 0;
    }
    ArmSub.setArmRotateWithoutSavingLocation(error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSub.setArmRotateWithoutSavingLocation(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
