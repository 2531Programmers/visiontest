package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;


public class LimelightDistance extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance to target", getDistance());
    }

    //vertical offset from crosshairs
    public double getY() {
        return ty.getDouble(0);
    }

    public double getDistance() {
        // d = (h2 - h1) / tan(a1+a2)

        double a2 = getY();
        return (Constants.Limelight.TargetHeight - Constants.Limelight.LimelightMountingHeight) / Math.tan((Constants.Limelight.LimelightMountingAngle + a2) * (Math.PI / 180.0));
    }

    public double getX() {
        return tx.getDouble(0);
    }
}
