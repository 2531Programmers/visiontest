package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  public DriveCommand() {
    driveSubsystem = RobotContainer.m_driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean turbo = RobotContainer.gamepad.getRawButton(5) ||
      RobotContainer.gamepad.getRawButton(6) ||
      RobotContainer.rightJoy.getTrigger() ||
      RobotContainer.leftJoy.getTrigger();
    SmartDashboard.putBoolean("Turbo", turbo);

    double m = turbo ? 1.0 : 0.6;
    String controls = RobotContainer.controlChooser.getSelected();
    if (controls == null) controls = "airplane-tank";
    if (controls.equals("airplane-tank")) {
      driveSubsystem.tankDrive(
        RobotContainer.leftJoy.getY() * m,
        RobotContainer.rightJoy.getY() * m
      );
    } else if (controls.equals("airplane-arcade")) {
      driveSubsystem.arcadeDrive(
        RobotContainer.rightJoy.getY() * m,
        -RobotContainer.rightJoy.getX() * m
      );
    } else if (controls.equals("joystick-tank")) {
      driveSubsystem.tankDrive(
        RobotContainer.gamepad.getRawAxis(1) * m,
        RobotContainer.gamepad.getRawAxis(5) * m
      );
    } else if (controls.equals("joystick-arcade")) {
      driveSubsystem.arcadeDrive(
        RobotContainer.gamepad.getY() * m,
        -RobotContainer.gamepad.getX() * m
      );
    } else if (controls.equals("joystick-car")) {
      driveSubsystem.arcadeDrive(
        (-RobotContainer.gamepad.getRawAxis(3) + RobotContainer.gamepad.getRawAxis(2)) * m,
        -RobotContainer.gamepad.getX() * m
      );
    } else {
      System.out.println("Unknown mode " + controls);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
