// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.function.DoubleSupplier;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Swerve.driveAimAtSpeakerPose;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShootStop extends ParallelRaceGroup {
  
  /** Creates a new AutoShoot. */
  public AutoShootStop(SwerveSubsystem s_SwerveSubsystem, ShooterSubsystem s_ShooterSubsystem, ArmSubsystem s_ArmSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShooterRunRPMRotateDistanceStop(s_ShooterSubsystem, s_ArmSubsystem, s_SwerveSubsystem, 1, 1, 17, 0.01),
    new driveAimAtSpeakerPose(s_SwerveSubsystem, translationX, translationY)
    );
  }
}