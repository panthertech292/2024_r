// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.Shooter.IntakeStore;
import frc.robot.commands.Shooter.RevShooter;
import frc.robot.commands.Shooter.RotateShooter;
import frc.robot.commands.Shooter.RotateShooterToAngle;
import frc.robot.commands.Shooter.RunShooterBelsAndRev;
import frc.robot.commands.Shooter.RunShooterBelts;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Controllers
  private final CommandXboxController io_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Subsystems
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem s_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //Commands

  //Intake Commands
  //private final Command z_IntakeRunForward = new IntakeRun(s_IntakeSubsystem, 0.45);
  //private final Command z_IntakeRunBackward = new IntakeRun(s_IntakeSubsystem, -0.45);

  //Shooter Commands
  private final Command z_RevShooter = new RevShooter(s_ShooterSubsystem, () -> (0.50)); //Use this command if shooter needs set speed
  private final Command z_RotateShooterUp = new RotateShooter(s_ShooterSubsystem, 0.20);
  private final Command z_RotateShooterDown = new RotateShooter(s_ShooterSubsystem, -0.20);
  private final Command z_RunShooterBeltsForward = new RunShooterBelts(s_ShooterSubsystem, 0.30);
  private final Command z_RunShooterBeltsBackward = new RunShooterBelts(s_ShooterSubsystem, -0.30);
  private final Command z_RotateShooterToAngle = new RotateShooterToAngle(s_ShooterSubsystem, 0.244, 5, 0.09);

  //Shooter & Intake Commands
  private final Command z_IntakeStore = new IntakeStore(s_ShooterSubsystem, s_IntakeSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();

    s_ShooterSubsystem.setDefaultCommand(new RunShooterBelsAndRev(s_ShooterSubsystem, () -> io_DriverController.getRightTriggerAxis(), () -> io_DriverController.getLeftTriggerAxis()));
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> -io_DriverController.getRightX(),
        () -> -io_DriverController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> io_DriverController.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = s_SwerveSubsystem.simDriveCommand(
        () -> MathUtil.applyDeadband(io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> io_DriverController.getRawAxis(2));

    s_SwerveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

  }

  private void configureBindings() {
    //Intake Buttons
    io_DriverController.a().whileTrue(z_IntakeStore);
    io_DriverController.b().whileTrue(z_RotateShooterToAngle);
    //Shooter Buttons
    io_DriverController.x().whileTrue(z_RunShooterBeltsBackward);
    io_DriverController.y().whileTrue(z_RunShooterBeltsForward);
    //Rotate Shooter
    io_DriverController.rightBumper().whileTrue(z_RotateShooterUp);
    io_DriverController.leftBumper().whileTrue(z_RotateShooterDown);
    //Rev Shooter (at set speed)
    io_DriverController.start().whileTrue(z_RevShooter);
    //Reset gyro
    io_DriverController.back().whileTrue(new InstantCommand(s_SwerveSubsystem::zeroGyro));

    //Turn using AXYB
    /*
    io_DriverController.y().whileTrue(s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> 0,
        () -> 1));
    io_DriverController.x().whileTrue(s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> 1,
        () -> 0));
    io_DriverController.b().whileTrue(s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> -1,
        () -> 0));
    io_DriverController.a().whileTrue(s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> 0,
        () -> -1));
    //Triggers
    io_DriverController.rightTrigger(0.05).whileTrue(s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> -io_DriverController.getRightTriggerAxis()));

    io_DriverController.leftTrigger(0.05).whileTrue(s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> io_DriverController.getLeftTriggerAxis())); */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
