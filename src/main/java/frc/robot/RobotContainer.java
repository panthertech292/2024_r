// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Shooter.*;
import frc.robot.subsystems.*;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RobotContainer {
  //Controllers
  private final static CommandXboxController io_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final static CommandXboxController io_OperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Subsystems
  private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();
  private final ClimbSubsystem s_ClimbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem  s_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //Commands

  //Intake Commands
  private final Command z_IntakeStore = new IntakeStore(s_ShooterSubsystem, s_IntakeSubsystem, s_ArmSubsystem);

  //Shooter Commands
  private final Command z_ShootFullPower = new ShooterRunRPM(s_ShooterSubsystem, 1, 1);
  private final Command z_Shoot75Power = new ShooterRunRPM(s_ShooterSubsystem, 0.75, 1);
  
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    registerCommands();
    setDefaultCommands();
    configureCamera();
    configureBindings();
  }

  private void registerCommands(){
    NamedCommands.registerCommand("IntakeStore", z_IntakeStore);
    NamedCommands.registerCommand("RevFullPower", new ShooterRev(s_ShooterSubsystem, 1));
    NamedCommands.registerCommand("Rev75Power", new ShooterRev(s_ShooterSubsystem, 0.75));
    NamedCommands.registerCommand("ShootFullPower", z_ShootFullPower);
    NamedCommands.registerCommand("Shoot75Power", z_Shoot75Power);
  }

  private void setDefaultCommands(){
    s_ArmSubsystem.setDefaultCommand(new ArmHoldAngle(s_ArmSubsystem, 17, 0.01));
    //Run intake on operator's right stick
    s_IntakeSubsystem.setDefaultCommand(new IntakeRun(s_IntakeSubsystem, ()-> io_OperatorController.getRightY()));
    s_ShooterSubsystem.setDefaultCommand(new ShooterRunRev(s_ShooterSubsystem, ()-> io_OperatorController.getLeftTriggerAxis(), ()-> io_OperatorController.getRightTriggerAxis(), ()-> io_OperatorController.getLeftY()));

    Command driveFieldOrientedAnglularVelocity = s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> -MathUtil.applyDeadband(io_DriverController.getRightX(), OperatorConstants.kDeadband));
    s_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureCamera(){
    //Configure the USB camera here
    //UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    //intakeCam.getActualDataRate(); <--Test this to see bandwidth usage
  }

  public static void setRightRumbleDriver(double rumble){
    io_DriverController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }

  private void configureBindings() {
    //Driver Controller
    io_DriverController.a().toggleOnTrue(z_IntakeStore);
    for(int povAngle = 0; povAngle < 360; povAngle = povAngle + 45){ //TODO: God please test this, no clue if this will work, at all.
      double povAngleRadians = Math.toRadians(povAngle+90);
      io_DriverController.pov(povAngle).whileTrue(s_SwerveSubsystem.driveCommand(() -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband), () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
      () -> -Math.cos(povAngleRadians), () -> Math.sin(povAngleRadians))); //TODO: This feels super sketchy
    }
    

    ///Operator Controller
    io_OperatorController.a().whileTrue(new ArmRotate(s_ArmSubsystem, -ArmConstants.kRotationSpeed)); //Rotate arm down
    io_OperatorController.x().whileTrue(new ArmRotate(s_ArmSubsystem, ArmConstants.kRotationSpeed)); //Rotate arm up
    io_OperatorController.b().whileTrue(new ClimbRun(s_ClimbSubsystem, -ClimbConstants.kClimbSpeed)); //Have robot climb down
    io_OperatorController.y().whileTrue(new ClimbRun(s_ClimbSubsystem, ClimbConstants.kClimbSpeed)); //Have robot climb up

    io_OperatorController.leftBumper().whileTrue(new ArmRotateToAngle(s_ArmSubsystem, ArmConstants.kRotationMaxAngle, 17, 0.01)); //Bring arm to max
    io_OperatorController.rightBumper().whileTrue(new ArmRotateToAngle(s_ArmSubsystem, ArmConstants.kRotationMinAngle, 17, 0.01)); //Bring arm home
    
    io_OperatorController.start().whileTrue(z_ShootFullPower);
    io_OperatorController.start().whileTrue(z_Shoot75Power);
  }

  public Command getAutonomousCommand() {
    return null; //Autos.exampleAuto(m_exampleSubsystem);
  }
}
