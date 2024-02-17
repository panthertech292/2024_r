// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax LowIntakeMotor;
  private final CANSparkMax UpIntakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    LowIntakeMotor = configSparkMax(IntakeConstants.kLowIntakeMotorID, true);
    UpIntakeMotor = configSparkMax(IntakeConstants.kUpIntakeMotorID, true);
  }

  private CANSparkMax configSparkMax(int ID, boolean invert){
    final CANSparkMax newSpark = new CANSparkMax(ID, MotorType.kBrushless);
    newSpark.restoreFactoryDefaults();
    newSpark.setIdleMode(IdleMode.kCoast);
    newSpark.setSmartCurrentLimit(60);
    newSpark.setInverted(invert);
    newSpark.enableVoltageCompensation(12);
    newSpark.burnFlash();
    return newSpark;
  }

  public void setIntake(double speed){
    LowIntakeMotor.set(speed);
    UpIntakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
