// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utilities.MotorUtil;

public class ClimbSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax ClimbMotor;
  
  public ClimbSubsystem() {
    ClimbMotor = MotorUtil.initSparkMax(ClimbConstants.kClimbMotorID, true, true);
  }

  public void setClimbMotor(double speed){
    ClimbMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
