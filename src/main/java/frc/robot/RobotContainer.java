// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Swerve.driveAimAtSpeakerPose;
import frc.robot.subsystems.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  //Controllers
  private final static CommandXboxController io_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final static CommandXboxController io_OperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Subsystems
  private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();
  private final ClimbSubsystem s_ClimbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
  private final LEDSubsystem s_LEDSubsystem = new LEDSubsystem();
  private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem  s_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  //Commands

  //Intake Commands
  private final Command z_IntakeStore = new IntakeStore(s_ShooterSubsystem, s_IntakeSubsystem, s_ArmSubsystem);

  //Shooter Commands
  private final Command z_ShootFullPower = new ShooterRunRPM(s_ShooterSubsystem, 1, 1);
  private final Command z_Shoot75Power = new ShooterRunRPM(s_ShooterSubsystem, 0.75, 1);

  //Auto
  private final SendableChooser<Command> autoChooser;
  
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    registerCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
    
    setDefaultCommands();
    configureCamera();
    configureBindings();
  }

  private void registerCommands(){
    NamedCommands.registerCommand("IntakeStore", z_IntakeStore);
    NamedCommands.registerCommand("IntakeStoreRevFull",  new IntakeStoreRev(s_ShooterSubsystem, s_IntakeSubsystem, s_ArmSubsystem, 1));
    NamedCommands.registerCommand("RevFullPower", new ShooterRev(s_ShooterSubsystem, 1));
    NamedCommands.registerCommand("Rev75Power", new ShooterRev(s_ShooterSubsystem, 0.75));
    NamedCommands.registerCommand("ShootFullPower", z_ShootFullPower);
    NamedCommands.registerCommand("Shoot75Power", z_Shoot75Power);
    NamedCommands.registerCommand("AutoShootStop", new AutoShootStop(s_SwerveSubsystem, s_ShooterSubsystem, s_ArmSubsystem, () -> 0, () -> 0));
    NamedCommands.registerCommand("ShootFullPowerStop", new ShooterRunRPMStop(s_ShooterSubsystem, 1, 1));
    NamedCommands.registerCommand("Shoot75PowerStop", new ShooterRunRPMStop(s_ShooterSubsystem, 0.75, 1));
    NamedCommands.registerCommand("Shoot25Power", new ShooterRunRPM(s_ShooterSubsystem, 0.25, 1));
  }

  private void setDefaultCommands(){
    //s_ArmSubsystem.setDefaultCommand(new ArmHoldAngle(s_ArmSubsystem, s_SwerveSubsystem, 17, 0.01)); TODO: Figure out if we want this
    //Run intake on operator's right stick
    s_IntakeSubsystem.setDefaultCommand(new IntakeRun(s_IntakeSubsystem, ()-> MathUtil.applyDeadband(-io_OperatorController.getRightY(), OperatorConstants.kDeadband)));
    s_ShooterSubsystem.setDefaultCommand(new ShooterRunRev(s_ShooterSubsystem, s_LEDSubsystem, s_SwerveSubsystem,
    ()-> getGreaterAxis(io_OperatorController.getLeftTriggerAxis(), io_DriverController.getLeftTriggerAxis()),
    ()-> getGreaterAxis(io_OperatorController.getRightTriggerAxis(), io_DriverController.getRightTriggerAxis()), 
    ()-> MathUtil.applyDeadband(-io_OperatorController.getLeftY(), OperatorConstants.kDeadband)));

    Command driveFieldOrientedAnglularVelocity = s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
        () -> -MathUtil.applyDeadband(io_DriverController.getRightX(), OperatorConstants.kDeadband+0.05));
    s_SwerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureCamera(){
    //Configure the USB camera here
    CameraServer.startAutomaticCapture();
    //UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    //intakeCam.getActualDataRate(); <--Test this to see bandwidth usage
  }

  public static void setRightRumbleDriver(double rumble){
    io_DriverController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }
  public static void setRightRumbleOperator(double rumble){
    io_OperatorController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }

  private void configureBindings() {

    //Driver Controller
    io_DriverController.a().toggleOnTrue(z_IntakeStore); //Intake

    io_DriverController.b().whileTrue( //Drive Slow button
    s_SwerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftY()*.5, OperatorConstants.kDeadband),
        () -> MathUtil.applyDeadband(-io_DriverController.getLeftX()*.5, OperatorConstants.kDeadband),
        () -> -MathUtil.applyDeadband(io_DriverController.getRightX(), OperatorConstants.kDeadband+0.05))
    );
    //Aim at speaker button
    io_DriverController.y().whileTrue(new driveAimAtSpeakerPose(s_SwerveSubsystem, () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband), () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband)));
    //Aim at speaker and shoot button (auto shoot)
    io_DriverController.x().whileTrue(new AutoShoot(s_SwerveSubsystem, s_ShooterSubsystem, s_ArmSubsystem, 
    () -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband),
    () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband)));

    //Align robot based off dpad
    for(int povAngle = 0; povAngle < 360; povAngle = povAngle + 45){
      double povAngleRadians = Math.toRadians(povAngle+90);
      io_DriverController.pov(povAngle).whileTrue(s_SwerveSubsystem.driveCommand(() -> MathUtil.applyDeadband(-io_DriverController.getLeftY(), OperatorConstants.kDeadband), () -> MathUtil.applyDeadband(-io_DriverController.getLeftX(), OperatorConstants.kDeadband),
      () -> -Math.cos(povAngleRadians), () -> -Math.sin(povAngleRadians)));
    }

    io_DriverController.rightBumper().whileTrue(z_ShootFullPower);
    io_DriverController.leftBumper().whileTrue(z_Shoot75Power);

    io_DriverController.start().whileTrue(new ArmRotateToAngle(s_ArmSubsystem, ArmConstants.kShotAnglePodium, 17, 0.01)); //Podium Shot
    io_DriverController.back().whileTrue(new InstantCommand(s_SwerveSubsystem::zeroGyro)); //Reset gyro
    

    ///Operator Controller
    io_OperatorController.a().whileTrue(new ArmRotate(s_ArmSubsystem, -ArmConstants.kRotationSpeed)); //Rotate arm down
    io_OperatorController.x().whileTrue(new ArmRotate(s_ArmSubsystem, ArmConstants.kRotationSpeed)); //Rotate arm up
    io_OperatorController.b().whileTrue(new ClimbRun(s_ClimbSubsystem, -ClimbConstants.kClimbSpeed)); //Have aro go down down
    io_OperatorController.y().whileTrue(new ClimbRun(s_ClimbSubsystem, ClimbConstants.kClimbSpeed)); //Have arm go up

    io_OperatorController.leftBumper().whileTrue(new ArmRotateToAngle(s_ArmSubsystem, ArmConstants.kRotationMaxAngle, 17, 0.01)); //Bring arm to max
    io_OperatorController.rightBumper().whileTrue(new ArmRotateToAngle(s_ArmSubsystem, ArmConstants.kRotationMinAngle, 17, 0.01)); //Bring arm home
    //io_OperatorController.rightBumper().whileTrue(new ArmRotateDashboard(s_ArmSubsystem, 17, 0.01)); //for testing

    //io_OperatorController.start().whileTrue(z_ShootFullPower);
    //io_OperatorController.back().whileTrue(z_Shoot75Power);
    io_OperatorController.back().onTrue(new ClimbRotateServo(s_ClimbSubsystem, 0.25)); //release
    io_OperatorController.start().onTrue(new ClimbRotateServo(s_ClimbSubsystem, 0)); //lock

    io_OperatorController.povUp().whileTrue(new ShooterRunRPM(s_ShooterSubsystem, 1.00, 1));
    io_OperatorController.povRight().whileTrue(new ShooterRunRPM(s_ShooterSubsystem, 0.75, 1));
    io_OperatorController.povDown().whileTrue(new ShooterRunRPM(s_ShooterSubsystem, 0.50, 1));
    io_OperatorController.povLeft().whileTrue(new ShooterRunRPM(s_ShooterSubsystem, 0.25, 1));
    
  }
  private double getGreaterAxis(double axisOne, double axisTwo){
    if(Math.abs(axisOne) > Math.abs(axisTwo)){
      return axisOne;
    }else{
      return axisTwo;
    }
  }

  public void setDisabledLED() {
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Blue){
        s_LEDSubsystem.setSolidColor(0, 0, 128);
      }else{
        s_LEDSubsystem.setSolidColor(128, 0, 0);
      }
    }
  }

  public Command getAutonomousCommand() {
    //return s_SwerveSubsystem.getAutonomousCommand("scoreClose3");
    return autoChooser.getSelected();
  }
}
