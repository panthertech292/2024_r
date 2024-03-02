// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDeadband = 0.05;
  }

  public static class ArmConstants {

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

  }
  public static class AutoConstants {

  } 

}
