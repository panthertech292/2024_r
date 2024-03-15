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
  private final CANSparkMax LeftClimbMotor;
  private final CANSparkMax RightClimbMotor;
  
  public ClimbSubsystem() {
    LeftClimbMotor = MotorUtil.initSparkMax(ClimbConstants.kLeftClimbMotorID, true, true);
    RightClimbMotor = MotorUtil.initSparkMax(ClimbConstants.kRightClimbMotorID, false, true);
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
