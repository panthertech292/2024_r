// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.utilities.MotorUtil;

public class ArmSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax RotationMotor;
  //Encoders
  private final DutyCycleEncoder RotationAngleEncoder;
  //Limit Switches
  private final DigitalInput RotationDownSwitch;
  //Distance
  private double LastCommandedLocation; //Last position we were at
  //Interpolation Map
  InterpolatingDoubleTreeMap angleMap;

  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Motors
    RotationMotor = MotorUtil.initSparkMax(ArmConstants.kRotationMotorID, false, true);
    //Encoders
    RotationAngleEncoder = new DutyCycleEncoder(ArmConstants.kRotationAngleEncoder);
    RotationAngleEncoder.setPositionOffset(ArmConstants.kRotationZeroAngleOffset);
    //Limit Switches
    RotationDownSwitch = new DigitalInput(ArmConstants.kRotationDownSwitchID);
    //Distances
    LastCommandedLocation = getRotationAngle();

    
    //Interpolation Map
    angleMap = new InterpolatingDoubleTreeMap();
    //Input constants map into interpolation map.
    for (Map.Entry<Double, Double> entry : InterpolationConstants.angleMap.entrySet()){
      angleMap.put(entry.getKey(), entry.getValue()); //TODO: Is this too much voodoo for what we are doing?
    }
  }

  /** @return True: The down limit switch is activated. NOTE: This value is inverted, due to current robot wiring. */
  public boolean getRotationDownSwitch(){
    return !RotationDownSwitch.get();
  }

  /** @return The rotation value of the arm/shooter. NOTE: This value is offset by {@link ArmConstants#kRotationZeroAngleOffset ArmConstants.kRotationZeroAngleOffset} */
  public double getRotationAngle(){
    return RotationAngleEncoder.get();
  }
  /** @return True: The arm is down at its MIN SAFE position.*/
  public boolean isArmDown(){
    return getRotationDownSwitch() || (getRotationAngle() < ArmConstants.kRotationMinAngle);
  }

  /** @return Truue: The arm is up at its MAX SAFE position*/
  public boolean isArmUp(){
    return getRotationAngle() > ArmConstants.kRotationMaxAngle;
  }

  /** @return True: The arm is down enough to intake.*/
  public boolean isArmReadyToIntake(){
    return getRotationDownSwitch() || (getRotationAngle() < ArmConstants.kRotationIntakeAngle);
  }

  /** Rotates the arm WITH saftey checks to stop the arm from destroying itself.
   *  @param speed The speed to set the arm to rotate at */
  public void setArmRotate(double speed){
    double rotationSpeed = speed;
    //Saftey checks for going down
    if(speed < 0){
      if(isArmDown()){ //Arm is down, stop.
        rotationSpeed = 0;
        System.out.println("Warning: Trying to rotate arm while arm is down!");
      }
      if(getRotationAngle() < ArmConstants.kRotationQuarterSpeedAngle){ //Arm is close to down, slow down
        rotationSpeed = rotationSpeed/4;
      }
      if(getRotationAngle() < ArmConstants.kRotationEighthSpeedAngle){ //Slow down even more, to 1/8 the given speed
        rotationSpeed = rotationSpeed/2;
      }
    }
    //Saftey check for going up
    if(speed > 0){
      if(getRotationAngle() > ArmConstants.kRotationMaxAngle){
        rotationSpeed = 0;
        System.out.println("Warning: Trying to rotate arm past safe UP limit!");
      }
    }
    LastCommandedLocation = getRotationAngle();
    RotationMotor.set(rotationSpeed);
  }

  /** Rotates the arm WITH saftey checks to stop the arm from destroying itself. Use this method to run the arm, but not log what the last commanded input was.
   *  @param speed The speed to set the arm to rotate at */
  public void setArmRotateWithoutSavingLocation(double speed){
    double rotationSpeed = speed;
    //Saftey checks for going down
    if(speed < 0){
      if(isArmDown()){ //Arm is down, stop.
        rotationSpeed = 0;
        System.out.println("Warning: Trying to rotate arm while arm is down!");
      }
      if(getRotationAngle() < ArmConstants.kRotationQuarterSpeedAngle){ //Arm is close to down, slow down
        rotationSpeed = rotationSpeed/4;
      }
      if(getRotationAngle() < ArmConstants.kRotationEighthSpeedAngle){ //Slow down even more, to 1/8 the given speed
        rotationSpeed = rotationSpeed/2;
      }
    }
    //Saftey check for going up
    if(speed > 0){
      if(getRotationAngle() > ArmConstants.kRotationMaxAngle){
        rotationSpeed = 0;
        System.out.println("Warning: Trying to rotate arm past safe UP limit!");
      }
    }
    
    RotationMotor.set(rotationSpeed);
  }
  public double getLastCommandedLocation(){
    return LastCommandedLocation;
  }

  /**
   * Gets the angle to set the arm to based off the distance of the robot from the speaker.
   * Uses interpolation to get values that are not directly defined in {@link InterpolationConstants}
   * @param distance The distance in INCHES from the Speaker to the Robot
   * @return The angle to set the arm to make the shot
   */
  public double getAngleFromDistance(double distance){
    return angleMap.get(distance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
