// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private WPI_TalonFX left = new WPI_TalonFX(0);
  private WPI_TalonFX right = new WPI_TalonFX(1);

  private static final double kS = 0.86975;
  private static final double kV = 0.39549;
  private static final double kA = 0.45718;
  private static final double kP = 0.059991;
  private static final double trackWidth = 0.68; // m
  private static final double kMaxSpeed = 3; // m/s
  private static final double kMaxAcceleration = 3; // m/s/s
  private static final double kRamseteB = 2;
  private static final double kRamseteZeta = 0.7;
  private static final double kPerTurn = 2048.0;
  private static final double kWheelCircumference = Math.PI * 6;
  private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);

  private DifferentialDriveOdometry odometry;

  private DifferentialDrive drive = new DifferentialDrive(left, right);

  public DriveSubsystem() {
    right.setInverted(true);
    resetEncoders();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    odometry.update(
      gyro.getRotation2d(),
      getLeftEncoderPosition(),
      getRightEncoderPositon()
    );
  }

  public void tankDrive(double leftPower, double rightPower) {
    // drive.tankDrive(leftPower, rightPower);
    right.set(rightPower);
    left.set(leftPower);
  }

  public void arcadeDrive(double power, double steering) {
    // drive.arcadeDrive(power, steering);
    double leftPower = power + steering;
    double rightPower = power - steering;
    tankDrive(leftPower, rightPower);
  }

  public void stop() {
    drive.stopMotor();
  }

  public double getLeftEncoderPosition() {
    return left.getSelectedSensorPosition() / kPerTurn * kWheelCircumference;
  }

  public double getRightEncoderPositon() {
    return right.getSelectedSensorPosition() / kPerTurn * kWheelCircumference;
  }

  public double getLeftEncoderVelocity() {
    return left.getSelectedSensorVelocity() / kPerTurn * kWheelCircumference;
  }

  public double getRightEncoderVelocity() {
    return right.getSelectedSensorVelocity() / kPerTurn * kWheelCircumference;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    drive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPositon()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void resetEncoders() {
    // TODO
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
