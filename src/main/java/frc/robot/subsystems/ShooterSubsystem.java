// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.MotorUtil;

public class ShooterSubsystem extends SubsystemBase {
  //Motors
  private final CANSparkMax ShooterLowMotor;
  private final CANSparkMax ShooterUpMotor;
  private final CANSparkMax FeedBeltLowMotor;
  private final CANSparkMax FeedBeltUpMotor;

  //Encoders (We only use the encoder from the low motor, but we can add the high one if we need to)
  private RelativeEncoder ShooterLowMotorEncoder;
  private RelativeEncoder ShooterUpMotorEncoder;

  //Limit Switches
  private final DigitalInput FeedBeltSwitch;
  private final DigitalInput ShooterSwitch;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //Motors
    ShooterLowMotor = MotorUtil.initSparkMax(ShooterConstants.kShooterLowMotorID, true, false, 11);
    ShooterUpMotor = MotorUtil.initSparkMax(ShooterConstants.kShooterUpMotorID, true, false, 11);
    FeedBeltLowMotor = MotorUtil.initSparkMax(ShooterConstants.kFeedBeltsLowMotorID, true, true); //might want to enable vComp?
    FeedBeltUpMotor = MotorUtil.initSparkMax(ShooterConstants.kFeedBeltsUpMotorID, true, true);

    //Encoders
    ShooterLowMotorEncoder = ShooterLowMotor.getEncoder();
    ShooterUpMotorEncoder = ShooterUpMotor.getEncoder();

    //Limit Switches
    FeedBeltSwitch = new DigitalInput(ShooterConstants.kFeedBeltSwitchID); //This switch activates when a note is just before the shooter
    ShooterSwitch = new DigitalInput(ShooterConstants.kShooterSwitchID);
  }

  /** @return Returns the RPM of the lower shooter motor */
  public double getShooterLowEncoderVelocity(){
    return ShooterLowMotorEncoder.getVelocity();
  }

  public double getShooterUpEncoderVelocity(){
    return ShooterUpMotorEncoder.getVelocity();
  }

  /** @return True: There is a note(or something) in the belts before the shooter. NOTE: Due to robot wiring, this is inverted! */
  public boolean getFeedBeltSwitch(){
    return !FeedBeltSwitch.get();
  }

  public boolean getShooterSwitch(){
    return !ShooterSwitch.get();
  }

  /** @param speed The speed to set the shooter to*/
  public void setShooter(double speed){
    ShooterLowMotor.set(speed);
    ShooterUpMotor.set(speed);
  }

  /** @param speed The speed to set the belts to*/
  public void setFeedBelts(double speed){
    FeedBeltLowMotor.set(speed);
    FeedBeltUpMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Feedbelt sensor active", getFeedBeltSwitch());
    //if(getShooterSwitch()){
    //  System.out.println("######### SAW NOTE");
    //}
    SmartDashboard.putBoolean("Shooter sensor active", getShooterSwitch());
    SmartDashboard.putNumber("Low Shooter RPM", getShooterLowEncoderVelocity());
    SmartDashboard.putNumber("Up Shooter RPM", getShooterUpEncoderVelocity());
  }
}
