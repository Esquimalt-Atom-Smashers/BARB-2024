package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

/**
 * Subsystem representing the motors that control the wheels 
 * that shoot the notes. 
 */
public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;

    private final SparkPIDController leftPIDController;
    private final SparkPIDController rightPIDController;
    
    /**
     * Constructs a ShooterSubsystem object.
     * Initializes the two motors and their PID controllers, which
     * spin to shoot the notes.
     */
    public ShooterSubsystem() {
        leftShooterMotor = new CANSparkMax(Constants.ShooterConstants.LEFT_SHOOTER_PORT, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(Constants.ShooterConstants.RIGHT_SHOOTER_PORT, MotorType.kBrushless);

        leftPIDController = leftShooterMotor.getPIDController();
        rightPIDController = rightShooterMotor.getPIDController();
        
        configureMotors();
    }

    /**
     * Configure the motors and their PID controllers. 
     */
    private void configureMotors() {
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
    
    /** @return Command that starts shooting at the default velocity */
    public Command shoot() {
        return shootAtVelocity(Constants.ShooterConstants.SHOOT_RPM);
    }
    
    /**
     * Creates and returns a command that shoots at a specified velocity.
     * 
     * @param velocity The velocity to shoot at
     * @return The command that shoots at that velocity
     */
    public Command shootAtVelocity(double velocity) {
        return runOnce(() -> setVelocity(velocity));
    }
    
    /**
     * Creates and returns a command that shoots at a specified voltage.
     * 
     * @param voltage The voltage to shoot at
     * @return The command that shoots at that voltage
     */
    public Command shootAtVoltage(double voltage) {
        return runOnce(() -> setVoltage(voltage));
    } 
    
    /**
     * Creates and returns a command that shoots at a specified voltage for three seconds, then stops. 
     * 
     * @param voltage The voltage to shoot at
     * @return The command that shoots at that voltage, then stops after three seconds
     */
    public Command shootManuallyWithTimeout(double voltage) {
        Timer timer = new Timer();
        // I (Elliot) changed this from the commented version below because the timer needs to be reset at the start of the command. 
        // I did the same to shootWithTimeout
        return runOnce(timer::reset)
        .andThen(shootAtVoltage(voltage), new WaitUntilCommand(() -> timer.hasElapsed(3)), stopShooting());
        // return run(() -> {
            //     leftShooterMotor.setVoltage(voltage);
            //     rightShooterMotor.setVoltage(voltage);
            // }).until(() -> timer.hasElapsed(3)).andThen(stopShooting());
    }
        
    /** @return Command that shoots for three seconds then stops */
    public Command shootWithTimeout() {
        Timer timer = new Timer();
        // See shootManuallyWithTimeout for reason why
        return runOnce(timer::reset)
        .andThen(shoot(), new WaitUntilCommand(() -> timer.hasElapsed(3)), stopShooting());
        // return run(() -> {
            //     setMotors(Constants.ShooterConstants.SHOOT_RPM);
            // }).until(() -> timer.hasElapsed(3)).andThen(stopShooting());
    }
    
    /** @return Command that stops the motors */
    public Command stopShooting() {
        return runOnce(this::stopMotors);
    }
    
    /**
     * Sets the motors to the specified voltage.
     * 
     * @param voltage Voltage to set the motors to
     */
    private void setVoltage(double voltage) {
        leftShooterMotor.set(voltage);
        rightShooterMotor.set(voltage);
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

    /**
     * Stops the motors.
     */
    private void stopMotors() {
        setVoltage(0);
    }
}