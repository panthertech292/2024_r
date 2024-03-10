// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRPMBelts extends Command {
  private final ShooterSubsystem ShooterSub;
  private double shooterSpeed;
  private double beltSpeed;
  private double [] last10Values;
  private int index;
  private boolean readyToFire;
  private boolean setInitalMovingAt; //debug
  /** Creates a new ShooterRPMBelts. */
  public ShooterRPMBelts(ShooterSubsystem s_ShooterSubsystem, double shooterSpeed, double beltSpeed) {
    ShooterSub = s_ShooterSubsystem;
    this.shooterSpeed = shooterSpeed;
    this.beltSpeed = beltSpeed;
    last10Values = new double[10];
    index = 0;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSub.setShooter(shooterSpeed);
    setInitalMovingAt = false;
    last10Values[0] = 0;
    last10Values[9] = 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    last10Values[index] = ShooterSub.getShooterLowEncoderSpeed();
    readyToFire = true;
    for (int i = 0; i < 10; i++){
      if(Math.abs(ShooterSub.getShooterLowEncoderSpeed() - last10Values[i]) > 25){
        readyToFire = false;
      }
    }
    //dumbass check here
    if(ShooterSub.getShooterLowEncoderSpeed() < 1000 & shooterSpeed > 0.49){
      readyToFire = false;
    }

    if(readyToFire){
      ShooterSub.setBelts(beltSpeed);
      if(!setInitalMovingAt){//debug
        setInitalMovingAt = true;
        System.out.println("Started advancing belts forward @: " + ShooterSub.getShooterLowEncoderSpeed());
      }
      
    }
    index++;
    index = index % 10;
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
