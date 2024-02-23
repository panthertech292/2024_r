// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = 46.2664; //kg, not lbs
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;
  }

  public static class IntakeConstants {
    //CAN Motors
    public static final int kLowIntakeMotorID = 21;
    public static final int kUpIntakeMotorID = 22;
    //Speeds
    public static final double kIntakeSpeed = 0.60;
  }
  public static class ShooterConstants {
    //CAN Motors
    public static final int kBeltsLowMotorID = 31;
    public static final int kBeltsUpMotorID = 32;
    public static final int kRotateMotorID = 33;
    public static final int kShooterLowMotorID = 34;
    public static final int kShooterUpMotorID = 35;
    //Limit Switches
    public static final int kRotateSwitchID = 2;
    public static final int kBeltSwitchID = 1;
    //Encoders
    public static final int kShooterAngleEncoderID = 0;
    public static final double kShooterAngleOffset = 0.1706;
    //Speeds
    public static final double kRotateSpeed = 0.30;
    public static final double kIntakeBeltSpeed = 0.05;
  }
  public static final class AutoConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }
}
