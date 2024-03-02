// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utilities.MotorUtil;

public class IntakeSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax LowIntakeMotor;
  private final CANSparkMax UpIntakeMotor;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    LowIntakeMotor = MotorUtil.initSparkMax(IntakeConstants.kLowIntakeMotorID, true, false, false);
    UpIntakeMotor = MotorUtil.initSparkMax(IntakeConstants.kUpIntakeMotorID, true, false, false);
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
