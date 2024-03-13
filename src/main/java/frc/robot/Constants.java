// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDeadband = 0.05;
  }

  public static class ArmConstants {
    //CAN Motor IDs
    public static final int kRotationMotorID = 33;
    //DIO Sensor IDs
    public static final int kRotationDownSwitchID = 2;
    public static final int kRotationAngleEncoder = 0; //This is a REV through bore encoder
    //Speeds
    public static final double kRotationSpeed = 0.40;
    //Encoder Positions
    public static final double kRotationZeroAngleOffset = 0.1726;
    public static final double kRotationMinAngle = 0.001;
    public static final double kRotationIntakeAngle = 0.002;
    public static final double kRotationMaxAngle = 0.233;
    public static final double kRotationQuarterSpeedAngle = 0.009;
    public static final double kRotationEighthSpeedAngle = 0.003;
  }

  public static class ClimbConstants {
    //CAN Motor IDs
    public static final int kLeftClimbMotorID = 41; //might need to lower these
    public static final int kRightClimbMotorID = 42;
    //Speeds
    public static final double kClimbSpeed = 0.80;
  }

  public static class IntakeConstants {
    //CAN Motor IDs
    public static final int kLowIntakeMotorID = 21;
    public static final int kUpIntakeMotorID = 22;
    //Speeds
    public static final double kIntakeSpeed = 0.40;
  }

  public static class ShooterConstants {
    //CAN Motor IDs
    public static final int kFeedBeltsLowMotorID = 31;
    public static final int kFeedBeltsUpMotorID = 32;
    public static final int kShooterLowMotorID = 34;
    public static final int kShooterUpMotorID = 35;
    //Limit Switches
    public static final int kFeedBeltSwitchID = 1;
    //Speeds
    public static final double kIntakeBeltSpeed = 0.05;
    public static final double kRevSpeed = .30;
  }

  public static class SwerveConstants {
    //Speeds
    public static final double kMaxSpeed = Units.feetToMeters(17.2); // Maximum speed of the robot in meters per second, used to limit acceleration.
  }

  public static class AutoConstants {
    //PIDs
    public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants kAnglePID   = new PIDConstants(0.4, 0, 0.01);
  } 
  
  public static class FieldConstants {
    public static final Translation2d kSpeakerPosition = new Translation2d(0, 5.547868);
  }

  public static class InterpolationConstants {
    public static final Map<Double, Double> angleMap = new HashMap<Double, Double>();
    //Values for Shooter Distance. Key is distance in inches, value is shooter angle
    static {
      angleMap.put(0.0, 0.0); //Example!
    }
  }

}
