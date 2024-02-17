// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax RotateMotor;
  private final CANSparkMax ShooterLowMotor;
  private final CANSparkMax ShooterUpMotor;
  private final CANSparkMax BeltsLowMotor;
  private final CANSparkMax BeltsUpMotor;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    RotateMotor = configSparkMax(ShooterConstants.kRotateMotorID, false, true);
    ShooterLowMotor = configSparkMax(ShooterConstants.kShooterLowMotorID, false, false);
    ShooterUpMotor = configSparkMax(ShooterConstants.kShooterUpMotorID, false, false);
    BeltsLowMotor = configSparkMax(ShooterConstants.kBeltsLowMotorID, false, false);
    BeltsUpMotor = configSparkMax(ShooterConstants.kBeltsUpMotorID, false, false);
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
  public void rotateShooter(double speed){
    RotateMotor.set(speed);
  }
  public void setBelts(double speed){
    BeltsLowMotor.set(speed);
    BeltsUpMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
