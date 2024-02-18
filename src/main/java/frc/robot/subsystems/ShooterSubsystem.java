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
    BeltsLowMotor = configSparkMax(ShooterConstants.kBeltsLowMotorID, false, false);
    BeltsUpMotor = configSparkMax(ShooterConstants.kBeltsUpMotorID, false, false);

    ShooterAngleEncoder = new DutyCycleEncoder(ShooterConstants.kShooterAngleEncoderID);

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
    return RotateSwitch.get();
  }
  public boolean getBeltSwitch(){
    return BeltSwitch.get();
  }
  public double getShooterAngle(){
    return ShooterAngleEncoder.get();
  }
  public void rotateShooter(double speed){
    //Check to make sure we aren't at limit and trying to go down. Let's try not to break the robot.
    if(!getRotateSwitch() || speed > 0){ //If the switch is not active or we are going in the positive direction (away)
      RotateMotor.set(speed);
    }else{
      RotateMotor.set(0);
    }
    
  }
  public void setBelts(double speed){
    BeltsLowMotor.set(speed);
    BeltsUpMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("ROTATE LIMIT", RotateSwitch.get());
    if(getRotateSwitch()){
      ShooterAngleEncoder.setPositionOffset(ShooterAngleEncoder.getAbsolutePosition());
    }
    SmartDashboard.putNumber("Shooter Angle DISTANCe", getShooterAngle());
  }
}
