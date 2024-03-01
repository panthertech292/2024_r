// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax LeftClimbMotor;
  private final CANSparkMax RightClimbMotor;
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    LeftClimbMotor = configSparkMax(ClimbConstants.kLeftClimbMotorID, true, true);
    RightClimbMotor = configSparkMax(ClimbConstants.kRightClimbMotorID, false, true);
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
  public void setClimbMotors(double speed){
    LeftClimbMotor.set(speed);
    RightClimbMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
