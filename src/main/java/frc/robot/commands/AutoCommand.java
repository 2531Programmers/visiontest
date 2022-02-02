// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightDistance;

public class AutoCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private LimelightDistance limelightDistance;
  private PIDController verticalPid = new PIDController(0.02, 0, 0);
  private PIDController horizontalPid = new PIDController(0.01, 0, 0);

  public AutoCommand(DriveSubsystem subsystem, LimelightDistance limelightDistance) {
    this.driveSubsystem = subsystem;
    this.limelightDistance = limelightDistance;
    addRequirements(driveSubsystem, limelightDistance);
  }

  @Override
  public void initialize() {
    verticalPid.setSetpoint(70);
    verticalPid.reset();
    horizontalPid.setSetpoint(0);
    horizontalPid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vertical = verticalPid.calculate(limelightDistance.getDistance());
    double horizontal = horizontalPid.calculate(limelightDistance.getX());
    if (vertical > 0.5) vertical = 0.5;
    if (vertical < -0.5) vertical = -0.5;
    driveSubsystem.arcadeDrive(vertical, horizontal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return pid.atSetpoint();
    return false;
  }
}
