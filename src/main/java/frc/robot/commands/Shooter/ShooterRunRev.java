// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterRunRev extends Command {
  private final ShooterSubsystem ShooterSub;
  private final LEDSubsystem LEDSub;
  private final SwerveSubsystem SwerveSub; //having this in here is absoultely awful!
  private DoubleSupplier shooterSpeed;
  private DoubleSupplier beltSpeed;
  private DoubleSupplier manualBeltSpeed;
  /** Creates a new RunShooterRev. */
  public ShooterRunRev(ShooterSubsystem s_ShooterSubsystem, LEDSubsystem s_LEDSubsystem, SwerveSubsystem s_SwerveSubsystem, DoubleSupplier shooterSpeed, DoubleSupplier beltSpeed, DoubleSupplier manualBeltSpeed) {
    ShooterSub = s_ShooterSubsystem;
    LEDSub = s_LEDSubsystem;
    SwerveSub = s_SwerveSubsystem;
    this.shooterSpeed = shooterSpeed;
    this.beltSpeed = beltSpeed;
    this.manualBeltSpeed = manualBeltSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem, s_LEDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //If there is a note loaded and our rev speed is greater than the input speed, run at rev speed
    if(ShooterSub.getFeedBeltSwitch() && (this.shooterSpeed.getAsDouble() < ShooterConstants.kRevSpeed)){
      ShooterSub.setShooter(ShooterConstants.kRevSpeed);
      RobotContainer.setRightRumbleDriver(0.25);
      LEDSub.blinkSolidColor(0, 255, 0);
    }else{
      ShooterSub.setShooter(this.shooterSpeed.getAsDouble());
      RobotContainer.setRightRumbleDriver(0.0);
      if(SwerveSub.isAllianceBlue()){
        LEDSub.setColorChase(120, 60, SwerveSub.getVelocity()*5);
      }else{
        LEDSub.setColorChase(60, 120, SwerveSub.getVelocity()*5);
      }
      
    }
    
    //If the override for manual speed operation is greater than the toggle speed
    if(Math.abs(manualBeltSpeed.getAsDouble()) > beltSpeed.getAsDouble()){
      ShooterSub.setFeedBelts(manualBeltSpeed.getAsDouble());
    }else{
      //If the trigger is held down enough, full send belts
      if (beltSpeed.getAsDouble() > 0.25){
        ShooterSub.setFeedBelts(1);
      }else{
        ShooterSub.setFeedBelts(0);
      }
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSub.setShooter(0);
    ShooterSub.setFeedBelts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
