package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem representing the motors that control the wheels that shoot the notes. 
 */
public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.LEFT_SHOOTER_PORT, MotorType.kBrushless);
    private final CANSparkMax rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_PORT, MotorType.kBrushless);

    private final SparkPIDController leftPIDController = leftShooterMotor.getPIDController();
    private final SparkPIDController rightPIDController = rightShooterMotor.getPIDController();
    
    public double appliedVoltage = 0.70;

    /**
     * Constructs a ShooterSubsystem object.
     * Initializes the two motors and their PID controllers, which spin to shoot the notes.
     */
    public ShooterSubsystem() {
        configureMotors();
    }
    
    /**
     * Configure the motors and their PID controllers. 
     */
    private void configureMotors() {
        rightShooterMotor.setInverted(true);
        
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
    
    /**
     * Creates and returns a command that shoots at a specified voltage.
     * 
     * @param voltage The voltage to shoot at
     * @return A command that shoots at that voltage
     */
    public Command shootAtVoltageCommand(double voltage) {
        return runOnce(() -> setVoltage(voltage));
    }
    
    /** @return A command that shoots at the default voltage */
    public Command shootAtVoltageCommand() {
        return shootAtVoltageCommand(ShooterConstants.SHOOTING_VOLTAGE);
    }
    
    /**
     * Creates and returns a command that shoots at a specified velocity.
     * 
     * @param velocity The velocity to shoot at
     * @return A command that shoots at that velocity
     */
    public Command shootAtVelocityCommand(double velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    /** @return A command that shoots at the default rpm */
    public Command shootAtVelocityCommand() {
        return shootAtVelocityCommand(ShooterConstants.SHOOT_RPM);
    }
    
    /** @return Command that stops the motors */
    public Command stopShootingCommand() {
        return runOnce(this::stopMotors);
    }
    
    /**
     * Sets the motors to the specified voltage.
     * 
     * @param voltage Voltage to set the motors to
     */
    private void setVoltage(double voltage) {
        leftShooterMotor.set(voltage);
        rightShooterMotor.set(voltage * 1.10);
    }

    /** Stops the motors. */
    private void stopMotors() {
        setVoltage(0);
    }
    
    /**
     * Sets the motors to the specified rpm.
     * 
     * @param rpm The rpm to set the motors to
     */
    private void setVelocity(double rpm) {
        rpm = Math.min(rpm, Constants.ShooterConstants.MAX_SHOOTER_RPM);
        leftPIDController.setReference(rpm, ControlType.kVelocity);
        rightPIDController.setReference(rpm, ControlType.kVelocity);
    }
}