package frc.lib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class ADISGyro {
    private ADIS16470_IMU gyro = new ADIS16470_IMU();

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ));
    }

    public void reset() {
        gyro.reset();
    }
}
