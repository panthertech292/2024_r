// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utilities.MotorUtil;

public class ClimbSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax ClimbMotor;
  private final Servo ClimbServo;
  private boolean servoLocked;
  
  public ClimbSubsystem() {
    ClimbMotor = MotorUtil.initSparkMax(ClimbConstants.kClimbMotorID, true, true);
    //ClimbMotor.setSmartCurrentLimit(0, 40, 10);
    ClimbServo = new Servo(ClimbConstants.kClimbServoID);
    setServoAngle(.25); // Release the servo on enable
  }

  public void setClimbMotor(double speed){
    ClimbMotor.set(speed);
  }

  public double getClimbMotorCurrent(){
    return ClimbMotor.getOutputCurrent();
  }

  public void setClimbMotorCurrentLimit(int current){
    ClimbMotor.setSmartCurrentLimit(current);
  }
  public void setServoAngle(double angle){
    if (angle >= 0 && angle <= 1){
      ClimbServo.set(angle);
    }else{
      System.out.println("Error: Trying to set climb servo out of bounds");
    }
    if (angle == 0){
      servoLocked = true;
    }else{
      servoLocked = false;
    }
  }
  public boolean getServoLocked(){
    return servoLocked;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Current (AMPS)", getClimbMotorCurrent());
  }
}
