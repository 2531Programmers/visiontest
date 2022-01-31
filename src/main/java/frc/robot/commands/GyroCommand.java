package frc.robot.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GyroscopeSubsystem;

public class GyroCommand extends CommandBase {
  public GyroCommand() {
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // short[] rawMag = RobotContainer.m_gyroscopeSubsystem.getMegnetometer();
    // double[] newArray = new double[3];
    // for (int i = 0; i < 3; i++) {
    //   newArray[i] = rawMag[i];
    // }
    // SmartDashboard.putNumberArray("Mag", newArray);
    // SmartDashboard.putNumber("Yaw", RobotContainer.m_gyroscopeSubsystem.getYawPitchRoll()[0]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
