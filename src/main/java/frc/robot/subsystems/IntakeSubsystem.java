package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * A subsystem that represents the intake motor, rotation motor, their PID controllers, and the two limit switches on the intake.
 */
public final class IntakeSubsystem extends SubsystemBase {
    /** Motor for rotating the intake into intake and index position. */
    private final CANSparkMax rotationMotor;
    private final CANSparkMax intakeMotor;

    private final SparkPIDController intakeController;
    private final SparkPIDController rotationController;

    private final DigitalInput intakePositionLimit;
    private final DigitalInput indexPositionLimit;

    private boolean isUp = true;

    /**
     * Constructs an IntakeSubsystem object. 
     * Initializes the two motors, and the limit switches. 
     */
    public IntakeSubsystem() {
        rotationMotor = new CANSparkMax(IntakeConstants.ROTATION_MOTOR_PORT, MotorType.kBrushless);
        rotationController = rotationMotor.getPIDController();
        configureRotationMotor();

        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
        intakeController = intakeMotor.getPIDController();
        configureIntakeMotor();

        intakePositionLimit = new DigitalInput(IntakeConstants.INTAKE_LIMIT_SWITCH_PORT);
        indexPositionLimit = new DigitalInput(IntakeConstants.INDEX_LIMIT_SWITCH_PORT);
    }

    /**
     * Configures the intake motor and its PID controller. 
     */
    private void configureIntakeMotor() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(IntakeConstants.INTAKE_MAX_VOLTAGE);
        intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_INVERTED);

        intakeController.setP(IntakeConstants.INTAKE_CONTROLLER_KP);
        intakeController.setI(IntakeConstants.INTAKE_CONTROLLER_KI);
        intakeController.setD(IntakeConstants.INTAKE_CONTROLLER_KD);
        intakeController.setIZone(IntakeConstants.INTAKE_CONTROLLER_IZ);
        intakeController.setFF(IntakeConstants.INTAKE_CONTROLLER_FF);
        intakeController.setOutputRange(-1, 1);
    }

    /**
     * Configures the rotation motor and its PID controller. 
     */
    private void configureRotationMotor() {
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.enableVoltageCompensation(Constants.IntakeConstants.ROTATION_MAX_VOLTAGE);
        rotationMotor.setInverted(IntakeConstants.ROTATION_MOTOR_INVERTED);

        rotationController.setP(IntakeConstants.ROTATION_CONTROLLER_KP);
        rotationController.setI(IntakeConstants.ROTATION_CONTROLLER_KI);
        rotationController.setD(IntakeConstants.ROTATION_CONTROLLER_KD);
        rotationController.setIZone(IntakeConstants.ROTATION_CONTROLLER_IZ);
        rotationController.setFF(IntakeConstants.ROTATION_CONTROLLER_FF);
        rotationController.setOutputRange(-1, 1);
    }

    /** @return Command that starts intaking */
    public Command intakeCommand() {
        return runOnce(() -> intake(IntakeConstants.INTAKE_RPM));
    }

    /** @return Command that starts outtaking */
    public Command outtakeCommand() {
        return runOnce(() -> outtake(IntakeConstants.OUTTAKE_RPM));
    }

    /** @reutrn Command that stops the intake motor */
    public Command stopMotorCommand() {
        return runOnce(this::stopIntaking);
    }

    /** @return Command that moves the intake up or down, depending on where the intake is */
    public Command toggleUpAndDownCommand() {
        return new ConditionalCommand(
            lowerIntakeCommand(),
            raiseIntakeCommand(),
            this::isUp
        );
    }

    /** Rotates the intake until it is in index position */
    public Command raiseIntakeCommand() {
        return run(() -> {
            isUp = true;
            rotateIntake(IntakeConstants.INTAKE_TO_INDEX_RPM);
        }).until(this::atIndexPosition).andThen(this::stopRotation);
    }

    /** Rotates the intake until it is in intake position */
    public Command lowerIntakeCommand() {
        return run(() -> {
            isUp = false;
            rotateIntake(IntakeConstants.INTAKE_TO_INTAKE_RPM);
        }).until(this::atIntakePosition).andThen(this::stopRotation);
    }    

    /**
     * Rotate the intake at a specified velocity. 
     * 
     * @param rpm The velocity to rotate at (in rpm)
     */
    private void rotateIntake(double rpm) {
        rpm = Math.min(rpm, IntakeConstants.ROTATION_MAX_RPM);
        rotationController.setReference(rpm, ControlType.kVelocity);
    }

    /**
     * Intake at a specified velocity. 
     * 
     * @param rpm The velocity to intake at (in rpm)
     */
    private void intake(double rpm) {
        intakeController.setReference(MathUtil.clamp(rpm, rpm, IntakeConstants.INTAKE_MAX_RPM), ControlType.kVelocity);
    }

    /**
     * Outtake at a specified velocity. 
     * 
     * @param rpm The velocity to outtake at (in rpm)
     */
    private void outtake(double rpm) {
        intakeController.setReference(-MathUtil.clamp(rpm, rpm, IntakeConstants.OUTTAKE_MAX_RPM) , ControlType.kVelocity);
    }

    /** Stop the rotation motor */
    private void stopRotation() {
        rotationMotor.set(0);
    }

    /** Stop the intake motor */
    private void stopIntaking() {
        intakeMotor.set(0);
    }
    
    /** @reutrn True if the lower limit switch is being pressed */
    private boolean atIntakePosition() {
        return intakePositionLimit.get();
    }

    /** @reutrn True if the upper limit switch is being pressed */
    private boolean atIndexPosition() {
        return indexPositionLimit.get();
    }

    /** @return True is the intake is up or moving to up, false otherwise */
    public boolean isUp() {
        return isUp;
    }
}
