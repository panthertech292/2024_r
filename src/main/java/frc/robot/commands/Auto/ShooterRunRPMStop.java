// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRunRPMStop extends Command {
  private final ShooterSubsystem ShooterSub;
  private double shooterSpeed;
  private double beltSpeed;
  private double shooterRPM;
  private double [] last10Values;
  private int index;
  private boolean readyToFire;
  private boolean setInitalMovingAt; //Used for debug, can be removed if needed
  /** Creates a new RunShooterRPM. */
  public ShooterRunRPMStop(ShooterSubsystem s_ShooterSubsystem, double shooterSpeed, double beltSpeed) {
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
    /* Basically this creates an array of the last 10 recorded shooter RPMS
       It will only shoot the note once RPMs have stabilized.
       Stabilized means the 10 recorded values are within 25 RPMs of the current shooter RPM
       This really could be acomplished with a PID, but I'm lazy.
    */
    shooterRPM = ShooterSub.getShooterLowEncoderVelocity();
    last10Values[index] = shooterRPM;
    readyToFire = true;
    for (int i = 0; i < 10; i++){
      if(Math.abs(shooterRPM - last10Values[i]) > 25){
        readyToFire = false;
      }
    }
    //Make sure we don't fire early
    if(shooterRPM < 1000 & shooterSpeed > 0.49){
      readyToFire = false;
    }

    if(readyToFire){
      ShooterSub.setFeedBelts(beltSpeed);
      if(!setInitalMovingAt){//debug
        setInitalMovingAt = true;
        System.out.println("RunShooterRPM: Started advancing belts forward @: " + shooterRPM);
      }
      
    }
    index++;
    index = index % 10;
    if(ShooterSub.getShooterSwitch()){
      System.out.println("ShooterRunRPMRotateDistance: Finished due to detecing out note");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ShooterRunRPMStop: On the way!");
    ShooterSub.setFeedBelts(0);
    ShooterSub.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSub.getShooterSwitch();
  }
}
