package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftShooterMotor;
    private final WPI_TalonSRX rightShooterMotor;
    
    public ShooterSubsystem() {
        leftShooterMotor = new WPI_TalonSRX(Constants.ShooterConstants.LEFT_SHOOTER_PORT);
        rightShooterMotor = new WPI_TalonSRX(Constants.ShooterConstants.RIGHT_SHOOTER_PORT);

        leftShooterMotor.configFactoryDefault();
        rightShooterMotor.configFactoryDefault();
    }

    public Command shoot() {
        return runOnce(() -> {
            leftShooterMotor.set(0.3);
            rightShooterMotor.set(0.3);
        });
    }

    public Command stopShooting() {
        return runOnce(() -> {
            leftShooterMotor.set(0);
            rightShooterMotor.set(0);
        });
    }
}
