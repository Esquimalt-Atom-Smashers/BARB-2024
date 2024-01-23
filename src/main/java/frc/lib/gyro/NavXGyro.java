package frc.lib.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;

public class NavXGyro {
    private AHRS navX = new AHRS(SerialPort.Port.kUSB1);
    public NavXGyro() {}

    public Rotation2d getRotation2d() {
        return navX.getRotation2d();
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(Units.degreesToRadians(navX.getRoll()), Units.degreesToRadians(navX.getPitch()), 0);
    }

    public Rotation3d getRotationRates3d() {
        return new Rotation3d(navX.getRawGyroX(), navX.getRawGyroY(), navX.getRawGyroZ());
    }
}
