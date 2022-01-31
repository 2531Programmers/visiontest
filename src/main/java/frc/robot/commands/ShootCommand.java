// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends CommandBase {
  private ShootSubsystem shootSubsystem;

  public ShootCommand(ShootSubsystem shoot) {
    this.shootSubsystem = shoot;
    addRequirements(shoot);
    SmartDashboard.putNumber("Power", 0.0);
  }

  @Override
  public void execute() {
    shootSubsystem.shoot(SmartDashboard.getNumber("Power", 0.0));
  }

  @Override
  public void end(boolean interrupted) {
    shootSubsystem.stop();
  }
}
