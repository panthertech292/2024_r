// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    public static final double kRotationZeroAngleOffset = 0.3335;
    public static final double kRotationMinAngle = 0.001;
    public static final double kRotationIntakeAngle = 0.002;
    public static final double kRotationMaxAngle = 0.233;
    public static final double kRotationQuarterSpeedAngle = 0.009;
    public static final double kRotationEighthSpeedAngle = 0.003;
    public static final double kRotationClimbAngle = 0.1665;
    //
    public static final double kShotAnglePodium = 0.068; // might need to double check?
  }

  public static class ClimbConstants {
    //CAN Motor IDs
    public static final int kClimbMotorID = 36;
    // PWM Servo IDs
    public static final int kClimbServoID = 1;
    //Speeds
    public static final double kClimbSpeed = 0.80;
    //Positions
    public static final double kLockPosition = 0.30;
    public static final double kReleasePosition = 0.70;
    //Currents
    public static final int kUpCurrentLimit = 75;
    public static final int kDownCurrentLimit = 9;
  }

  public static class IntakeConstants {
    //CAN Motor IDs
    public static final int kLowIntakeMotorID = 21;
    public static final int kUpIntakeMotorID = 22;
    //Speeds
    public static final double kIntakeSpeed = 0.50;
  }

  public static class LEDConstants {
    public static final int kLEDPort = 0; //PWM
  }

  public static class ShooterConstants {
    //CAN Motor IDs
    public static final int kFeedBeltsLowMotorID = 34;
    public static final int kFeedBeltsUpMotorID = 35;
    public static final int kShooterLowMotorID = 31;
    public static final int kShooterUpMotorID = 32;
    //Limit Switches
    public static final int kFeedBeltSwitchID = 1;
    public static final int kShooterSwitchID = 9;
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
    public static final PIDConstants kTranslationPID = new PIDConstants(1.25, 0, 0); // was 0.7 before!
    public static final PIDConstants kAnglePID   = new PIDConstants(1, 0, 0.01); /// was 0.4 before!
  } 
  
  public static class FieldConstants {
    public static final Translation2d kSpeakerPositionBLUE = new Translation2d(0.0, 5.547868); //TODO: Confirm these
    public static final Translation2d kSpeakerPositionRED = new Translation2d(16.5410642, 5.547868);
  }

  public static class InterpolationConstants {
    //public static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap angleMapRED = new InterpolatingDoubleTreeMap();
    //Values for Shooter Distance. Key is distance in meters, value is shooter angle
    static {       //Distance //Angle //RED SIDE
      angleMapRED.put(1.44, 0.0);
      angleMapRED.put(2.27, 0.0438);
      angleMapRED.put(3.02, 0.07); //spot for shooting from top close note
      angleMapRED.put(3.12, 0.072);
      angleMapRED.put(4.05, 0.085);
      angleMapRED.put(4.25, 0.086);
      angleMapRED.put(4.45, 0.090);
      angleMapRED.put(4.85, 0.092);
      angleMapRED.put(5.00, 0.096);
      angleMapRED.put(5.58, 0.097);
      angleMapRED.put(5.79, 0.098 );
      angleMapRED.put(6.16, 0.102);
    }
    /*static {       //Distance //Angle //BLUE SIDE
      angleMap.put(1.44, 0.0);
      angleMap.put(2.27, 0.0438);
      angleMap.put(3.02, 0.07); //spot for shooting from top close note
      angleMap.put(3.12, 0.072);
      angleMap.put(4.05, 0.085);
      angleMap.put(4.25, 0.086);
      angleMap.put(4.45, 0.092);
      angleMap.put(4.85, 0.093);
      angleMap.put(5.00, 0.097);
      angleMap.put(5.58, 0.098);
      angleMap.put(5.79, 0.099 );
      angleMap.put(6.16, 0.103);
    }*/
  }

}
