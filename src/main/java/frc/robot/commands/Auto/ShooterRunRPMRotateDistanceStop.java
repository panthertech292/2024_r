// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShooterRunRPMRotateDistanceStop extends Command {
  //Shooting
  private final ShooterSubsystem ShooterSub;
  private double shooterSpeed;
  private double beltSpeed;
  private double shooterRPM;
  private double [] last10Values;
  private int index;
  private boolean readyToFire;
  private boolean setInitalMovingAt; //Used for debug, can be removed if needed

  //Rotation
  private final ArmSubsystem ArmSub;
  private double error;
  private double p;
  private double minSpeed;

  //Distance
  private SwerveSubsystem SwerveSub;
  private double distance;
  private double angle;

  private boolean finished;
  //private Timer time;
  private double noteOutTime;
  /** Creates a new RunShooterRPM. */
  public ShooterRunRPMRotateDistanceStop(ShooterSubsystem s_ShooterSubsystem, ArmSubsystem s_ArmSubsystem, SwerveSubsystem s_SwerveSubsystem, double shooterSpeed, double beltSpeed, double p, double minSpeed) {
    ShooterSub = s_ShooterSubsystem;
    ArmSub = s_ArmSubsystem;
    SwerveSub = s_SwerveSubsystem;
    this.shooterSpeed = shooterSpeed;
    this.beltSpeed = beltSpeed;
    last10Values = new double[10];
    index = 0;
    distance = SwerveSub.getDistanceFromSpeaker();
    angle = InterpolationConstants.angleMap.get(distance);
    
    //Rotation
    this.p = p;
    this.minSpeed = minSpeed;

    
    //time.reset();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSub.setShooter(shooterSpeed);
    setInitalMovingAt = false;
    finished = false;
    //noteOutTime = 999999999;
    last10Values[0] = 0;
    last10Values[9] = 1000;
    System.out.println("ShooterRunRPMRotateDistanceStop: Start Auto Shoot");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("SHOOTER RPM", angle);
    distance = SwerveSub.getDistanceFromSpeaker();
    angle = InterpolationConstants.angleMap.get(distance);

    error = (angle - ArmSub.getRotationAngle())*p;
    if (minSpeed > Math.abs(error)){//If we are running too slow, go at a min speed
      if (error > 0){//Checks sign of error, sets minspeed to either + or -
        error = minSpeed;
      }else{
        error = -minSpeed;
      }
    }
    ArmSub.setArmRotate(error);
    readyToFire = true;
    //Only run below if arm is in position
    if(Math.abs(error) > 0.03){
      readyToFire = false;
    }
      /* Basically this creates an array of the last 10 recorded shooter RPMS
        It will only shoot the note once RPMs have stabilized.
        Stabilized means the 10 recorded values are within 25 RPMs of the current shooter RPM
        This really could be acomplished with a PID, but I'm lazy.
      */

      shooterRPM = ShooterSub.getShooterLowEncoderVelocity();
      last10Values[index] = shooterRPM;
      
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
          noteOutTime = Timer.getFPGATimestamp();
          System.out.println("ShooterRunRPMRotateDistanceStop: Started advancing belts forward @: " + shooterRPM + " With: Interpolated Angle: " + angle + " at time: " + noteOutTime + " at a distance of " + distance);
        }
        if(ShooterSub.getShooterSwitch()){
          System.out.println("ShooterRunRPMRotateDistanceStop: Finished due to detecing out note");
          finished = true;
        }
        
        if(!ShooterSub.getFeedBeltSwitch()){
          if(Timer.getFPGATimestamp() > noteOutTime + 0.15){
            System.out.println("ShooterRunRPMRotateDistanceStop: Finished at time: " + Timer.getFPGATimestamp());
            finished = true;
          }
          
        }
        
      }
      index++;
      index = index % 10;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ShooterDistance: On the way!");
    ShooterSub.setFeedBelts(0);
    ShooterSub.setShooter(0);
    ArmSub.setArmRotate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
