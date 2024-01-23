package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public final class IntakeSubsystem extends SubsystemBase {
    /** Motor for rotating the intake into intake and index position. */
    private final CANSparkMax rotationMotor;
    private final CANSparkMax intakeMotor;

    private final SparkPIDController intakeController;

    private final DigitalInput intakePositionLimit;
    private final DigitalInput indexPositionLimit;

    public IntakeSubsystem() {
        rotationMotor = new CANSparkMax(-1, MotorType.kBrushless);
        configureRotationMotor();

        intakeMotor = new CANSparkMax(-1, MotorType.kBrushless);
        intakeController = intakeMotor.getPIDController();
        configureIntakeMotor();

        intakePositionLimit = new DigitalInput(-1);
        indexPositionLimit = new DigitalInput(-1);
    }

    private void configureIntakeMotor() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(12.2);
        intakeMotor.setInverted(false);

        intakeController.setP(IntakeConstants.INTAKE_CONTROLLER_KP);
        intakeController.setI(IntakeConstants.INTAKE_CONTROLLER_KI);
        intakeController.setD(IntakeConstants.INTAKE_CONTROLLER_KD);
        intakeController.setIZone(IntakeConstants.INTAKE_CONTROLLER_IZ);
        intakeController.setFF(IntakeConstants.INTAKE_CONTROLLER_FF);
        intakeController.setOutputRange(-1, 1);
    }

    private void configureRotationMotor() {

    }

    public Command intakeNote() {
        return run(() -> intake(IntakeConstants.INTAKE_RPM));
    }

    public Command outtakeNote() {
        return run(() -> outtake(IntakeConstants.OUTTAKE_RPM));
    }


    /** Rotates the intake until it is in index position. */
    public Command intakeToIndexPosition() {
        return run(() -> rotateIntake(-1)).until(this::atIndexPosition).andThen(this::stopRotation);
    }

    /** Rotates the intake until it is in intake position */
    public Command intakeToIntakePosition() {
        return run(() -> rotateIntake(1)).until(this::atIntakePosition).andThen(this::stopRotation);
    }

    private void rotateIntake(double rpm) {

    }

    private void intake(double rpm) {
        intakeController.setReference(MathUtil.clamp(rpm, rpm, IntakeConstants.INTAKE_MAX_RPM)
        , ControlType.kVelocity);
    }

    private void outtake(double rpm) {
        intakeController.setReference(-MathUtil.clamp(rpm, rpm, IntakeConstants.OUTTAKE_MAX_RPM)
        , ControlType.kVelocity);
    }

    private void stopRotation() { rotationMotor.set(0); }
    private void stopIntaking() {
        intakeMotor.set(0);
    }
    

    private boolean atIntakePosition() { return intakePositionLimit.get(); }
    private boolean atIndexPosition() { return indexPositionLimit.get(); }
}
