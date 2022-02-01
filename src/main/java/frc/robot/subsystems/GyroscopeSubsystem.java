// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroscopeSubsystem extends SubsystemBase {
  // private PigeonIMU gyro = new PigeonIMU(new TalonSRX(5));

  public double[] getYawPitchRoll() {
    // double[] yawPitchRoll = new double[3];
    // gyro.getYawPitchRoll(yawPitchRoll);
    // return yawPitchRoll;
    return new double[0];
  }

  public short[] getMegnetometer() {
    // short[] magnetometer = new short[3];
    // gyro.getRawMagnetometer(magnetometer);
    // return magnetometer;
    return new short[0];
  }

  public double getFusedHeading() {
    // return gyro.getFusedHeading();
    return 0;
  }

  public void calibrate() {
    // gyro.enterCalibrationMode(CalibrationMode.Magnetometer12Pt);
  }
}
