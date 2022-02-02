// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OrchestraCommand;
import frc.robot.commands.ShootCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    new Thread(() -> {
      UsbCamera one = CameraServer.startAutomaticCapture(0);
      UsbCamera two = CameraServer.startAutomaticCapture(1);
    }).start();

    RobotContainer.controlChooser.addOption("None", "none");
    RobotContainer.controlChooser.addOption("Airplane tank", "airplane-tank");
    RobotContainer.controlChooser.addOption("Airplane arcade", "airplane-arcade");
    RobotContainer.controlChooser.addOption("Joystick tank", "joystick-tank");
    RobotContainer.controlChooser.addOption("Joystick arcade", "joystick-arcade");
    RobotContainer.controlChooser.addOption("Joystick car", "joystick-car");
    SmartDashboard.putData("Controls", RobotContainer.controlChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.driveCommand.schedule();
    // RobotContainer.shootCommand.schedule();

    // new OrchestraCommand("gourmeg.chrp", Arrays.asList(
    //   RobotContainer.talonSubsystem.one
    // )).schedule();
    // new OrchestraCommand("n.chrp", Arrays.asList(RobotContainer.talonSubsystem.one)).schedule();
    // new OrchestraCommand("n.chrp", Arrays.asList(RobotContainer.talonSubsystem.two)).schedule();
    // new OrchestraCommand("n.chrp", Arrays.asList(RobotContainer.talonSubsystem.three)).schedule();
    // new OrchestraCommand("n.chrp", Arrays.asList(RobotContainer.talonSubsystem.four)).schedule();
    // new OrchestraCommand("n.chrp", Arrays.asList(RobotContainer.talonSubsystem.five)).schedule();
    // RobotContainer.talonSubsystem.one.set(ControlMode.PercentOutput, 1);
    // RobotContainer.talonSubsystem.two.set(ControlMode.PercentOutput, 1);
    // RobotContainer.talonSubsystem.three.set(ControlMode.PercentOutput, 1);
    // RobotContainer.talonSubsystem.four.set(ControlMode.PercentOutput, 1);
    // RobotContainer.talonSubsystem.five.set(ControlMode.PercentOutput, 1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.println("asdfasdfasdfsadfa");
    // double[] info = RobotContainer.m_gyroscopeSubsystem.getYawPitchRoll();
    // System.out.println("a" + info[0]);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
