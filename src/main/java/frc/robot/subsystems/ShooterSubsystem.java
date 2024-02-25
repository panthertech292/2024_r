// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax RotateMotor;
  private final CANSparkMax ShooterLowMotor;
  private final CANSparkMax ShooterUpMotor;
  private final CANSparkMax BeltsLowMotor;
  private final CANSparkMax BeltsUpMotor;

  private final DutyCycleEncoder ShooterAngleEncoder;

  private final DigitalInput RotateSwitch;
  private final DigitalInput BeltSwitch;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    RotateMotor = configSparkMax(ShooterConstants.kRotateMotorID, false, true);
    ShooterLowMotor = configSparkMax(ShooterConstants.kShooterLowMotorID, false, false);
    ShooterUpMotor = configSparkMax(ShooterConstants.kShooterUpMotorID, false, false);
    BeltsLowMotor = configSparkMax(ShooterConstants.kBeltsLowMotorID, false, true);
    BeltsUpMotor = configSparkMax(ShooterConstants.kBeltsUpMotorID, false, true);

    ShooterAngleEncoder = new DutyCycleEncoder(ShooterConstants.kShooterAngleEncoderID);
    //if(getRotateSwitch()){
      //ShooterAngleEncoder.setPositionOffset(ShooterAngleEncoder.getAbsolutePosition());
    //}else{
      ShooterAngleEncoder.setPositionOffset(ShooterConstants.kShooterAngleOffset);
    //}
    RotateSwitch = new DigitalInput(ShooterConstants.kRotateSwitchID);
    BeltSwitch = new DigitalInput(ShooterConstants.kBeltSwitchID);
  }

  private CANSparkMax configSparkMax(int ID, boolean invert, boolean brakeMode){
    final CANSparkMax newSpark = new CANSparkMax(ID, MotorType.kBrushless);
    newSpark.restoreFactoryDefaults();
    if(brakeMode){
      newSpark.setIdleMode(IdleMode.kBrake);
    }else{
      newSpark.setIdleMode(IdleMode.kCoast);
    }
    
    newSpark.setSmartCurrentLimit(60);
    newSpark.setInverted(invert);
    newSpark.enableVoltageCompensation(12);
    newSpark.burnFlash();
    return newSpark;
  }

  public void setShooter(double speed){
    ShooterLowMotor.set(speed);
    ShooterUpMotor.set(speed);
  }
  public boolean getRotateSwitch(){
    return !RotateSwitch.get();
  }
  public boolean getBeltSwitch(){
    return !BeltSwitch.get();
  }
  public double getShooterAngle(){
    return ShooterAngleEncoder.get();
  }
  public boolean isShooterDown(){
    return getRotateSwitch() || (getShooterAngle() < 0.002);
  }
  public void rotateShooter(double speed){
    double rotateSpeed = speed;
    //Saftey checks to try and not destroy the shooter. These checks only need to be performed if going down
    if(speed < 0){
      //1. We are on the limit switch, stop!
      if(isShooterDown()){
        rotateSpeed = 0;
        System.out.println("Warning: TRYING TO ROTATE SHOOTER DOWN WHILE ON SWTICH");
      }
      //2. We are close to the swtich, slow down
      if(getShooterAngle() < 0.009){
        rotateSpeed = rotateSpeed/4;
      }
      if(getShooterAngle() < 0.003){
        rotateSpeed = rotateSpeed/2;
      }
    }
    //Saftey check for going up
    if(speed > 0){
      if(getShooterAngle() > 0.25){
        rotateSpeed = 0;
        System.out.println("Warning: TRYING TO ROTATE SHOOTER UP PAST SAFE LIMIT");
      }
    }
    RotateMotor.set(rotateSpeed);
  }
  public void setBelts(double speed){
    BeltsLowMotor.set(speed);
    BeltsUpMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ROTATE LIMIT", getRotateSwitch());
    SmartDashboard.putBoolean("BELT LIMIT", getBeltSwitch());
    SmartDashboard.putBoolean("SHOOTER DOWN", isShooterDown());
    SmartDashboard.putNumber("Shooter Angle DISTANCE", getShooterAngle());

    
    
  }
}
