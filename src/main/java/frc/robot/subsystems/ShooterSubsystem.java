package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;

    private final SparkPIDController leftPIDController;
    private final SparkPIDController rightPIDController;
    
    public ShooterSubsystem() {
        leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.LEFT_SHOOTER_PORT, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_PORT, MotorType.kBrushless);

        leftPIDController = leftShooterMotor.getPIDController();
        rightPIDController = rightShooterMotor.getPIDController();
        
        motorInit();
    }

    private void motorInit() {
        leftShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.VOLTAGE_COMPENSATION);
        rightShooterMotor.enableVoltageCompensation(Constants.ShooterConstants.VOLTAGE_COMPENSATION);

        leftPIDController.setP(Constants.ShooterConstants.LEFT_SHOOTER_PID_K_P);
        leftPIDController.setI(Constants.ShooterConstants.LEFT_SHOOTER_PID_K_I);
        leftPIDController.setD(Constants.ShooterConstants.LEFT_SHOOTER_PID_K_D);

        rightPIDController.setP(Constants.ShooterConstants.RIGHT_SHOOTER_PID_K_P);
        rightPIDController.setI(Constants.ShooterConstants.RIGHT_SHOOTER_PID_K_I);
        rightPIDController.setD(Constants.ShooterConstants.RIGHT_SHOOTER_PID_K_D);

        leftPIDController.setOutputRange(-1, 1);
        rightPIDController.setOutputRange(-1, 1);
    }

    private void setMotors(double rpm) {
        rpm = Math.min(rpm, Constants.ShooterConstants.MAX_SHOOTER_RPM);
        leftPIDController.setReference(rpm, ControlType.kVelocity);
        rightPIDController.setReference(rpm, ControlType.kVelocity);
    }
    
    public Command shoot() {
        return runOnce(() -> {
            setMotors(Constants.ShooterConstants.SHOOT_RPM);
        });
    }
    
    public Command shootWithTimeout() {
        Timer timer = new Timer();
        return run(() -> {
            setMotors(Constants.ShooterConstants.SHOOT_RPM);
        }).onlyWhile(() -> !timer.hasElapsed(3)).andThen(stopShooting());
    }
    public Command shoot(double velocity) {
        return runOnce(() -> {
            setMotors(velocity);
        });
    }
    
    public Command stopShooting() {
        return runOnce(() -> {
            leftShooterMotor.set(0);
            rightShooterMotor.set(0);
        });
    }
}