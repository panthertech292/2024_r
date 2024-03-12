// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmHoldAngle;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;

import java.io.File;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RobotContainer {
  //Controllers
  private final static CommandXboxController io_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Subsystems
  private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();
  private final ClimbSubsystem s_ClimbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem  s_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_ArmSubsystem.setDefaultCommand(new ArmHoldAngle(s_ArmSubsystem, 17, 0.01));
    configureCamera();
    configureBindings();
  }
  private void configureCamera(){
    //Configure the USB camera here
    UsbCamera intakeCam = CameraServer.startAutomaticCapture();
    //intakeCam.getActualDataRate(); <--Test this to see bandwidth usage
  }

  public static void setRightRumbleDriver(double rumble){
    io_DriverController.getHID().setRumble(RumbleType.kRightRumble, rumble);
  }

  private void configureBindings() {
    //Driver controller
  }

  public Command getAutonomousCommand() {
    return null; //Autos.exampleAuto(m_exampleSubsystem);
  }
}
