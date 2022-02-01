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
  private PIDController pid = new PIDController(0.02, 0.01, 0);

  public AutoCommand(DriveSubsystem subsystem, LimelightDistance limelightDistance) {
    this.driveSubsystem = subsystem;
    this.limelightDistance = limelightDistance;
    addRequirements(driveSubsystem, limelightDistance);
  }

  @Override
  public void initialize() {
    pid.reset();
    pid.setSetpoint(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = limelightDistance.getDistance();
    double calculated = pid.calculate(distance);
    System.out.println(calculated);
    driveSubsystem.arcadeDrive(calculated, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
