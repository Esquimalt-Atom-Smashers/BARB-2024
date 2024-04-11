package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.ABSOLUTE_ENCODER_PORT;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * A subsystem that represents the intake motor, rotation motor, their PID controllers, and the two limit switches on the intake.
 */
public final class IntakeSubsystem extends SubsystemBase {
    /** Whether we currently have a note. */
    private boolean hasNote = false;

    /** Motor for rotating the intake into intake and index position. */
    private final CANSparkMax rotationMotor;
    /** Motor that rotates the wheels to intake and outtake the notes. */
    private final CANSparkMax intakeMotor;

    /** The absolute encoder on the rotationMotor. */
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ABSOLUTE_ENCODER_PORT);

    /** The PID controller for the rotationMotor. */
    private final SparkPIDController rotationController;
    /** The PID controller for the intakeMotor. */
    private final SparkPIDController intakeController;

    private final DigitalInput lowerPositionLimit;
    private final DigitalInput upperPositionLimit;

    private boolean isUp = true;
    private int currentTimer = 0;

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

        lowerPositionLimit = new DigitalInput(IntakeConstants.LOWER_LIMIT_SWITCH_PORT);
        upperPositionLimit = new DigitalInput(IntakeConstants.UPPER_LIMIT_SWITCH_PORT);
    }

    @Override
    public void periodic() {
        if (intakeMotor.getOutputCurrent() > 20) {
            currentTimer++;
            if (currentTimer > 10) { this.hasNote = true; }
        } else { currentTimer = 0; }
        
        SmartDashboard.putBoolean(" Note in Intake", this.hasNote);
        SmartDashboard.putNumber("Intake Position", this.rotationMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Upper pressed?", isAtUpperPosition());
        SmartDashboard.putBoolean("Lower pressed?", isAtLowerPosition());
//        System.out.println("Is lower pressed?: " + isAtLowerPosition());
//        System.out.println("Is upper pressed?: " + isAtUpperPosition());
    }

    /**
     * Configures the intake motor and its PID controller. 
     */
    private void configureIntakeMotor() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(IntakeConstants.INTAKE_MAX_VOLTAGE);
        intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotor.burnFlash();
        // intakeMotor.setOpenLoopRampRate(2);

        // intakeController.setP(IntakeConstants.INTAKE_CONTROLLER_KP);
        // intakeController.setI(IntakeConstants.INTAKE_CONTROLLER_KI);
        // intakeController.setD(IntakeConstants.INTAKE_CONTROLLER_KD);
        // intakeController.setIZone(IntakeConstants.INTAKE_CONTROLLER_IZ);
        // intakeController.setFF(IntakeConstants.INTAKE_CONTROLLER_FF);
        // intakeController.setOutputRange(-1, 1);
    }

    /**
     * Configures the rotation motor and its PID controller. 
     */
    private void configureRotationMotor() {
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.enableVoltageCompensation(Constants.IntakeConstants.ROTATION_MAX_VOLTAGE);
        rotationMotor.setInverted(IntakeConstants.ROTATION_MOTOR_INVERTED);
        rotationMotor.burnFlash();

        rotationController.setP(0.04);
        rotationController.setI(IntakeConstants.ROTATION_CONTROLLER_KI);
        rotationController.setD(IntakeConstants.ROTATION_CONTROLLER_KD);
        rotationController.setIZone(IntakeConstants.ROTATION_CONTROLLER_IZ);
        rotationController.setFF(IntakeConstants.ROTATION_CONTROLLER_FF);
        rotationController.setOutputRange(-0.75, 0.75);
    }

    /** @return Command that starts intaking */
    public Command intakeCommand() {
        return new ParallelDeadlineGroup(new WaitUntilCommand(() -> hasNote), new InstantCommand(() -> intakeMotor.set(0.70))).andThen(stopMotorCommand());
        // return runOnce(() -> intakeMotor.set(0.7));
        // return runOnce(() -> intakeMotor.set(hasNote ? 0 : 0.5));
    }

    /**Command used by impatient Auto
     * @return Command
     */
    public Command autoIntakeCommand() {
        return new ParallelRaceGroup(intakeCommand(), new WaitCommand(2));
    }

    /**Goes to the position used for intaking
     * @return Command
     */
    public Command goToIntakePosition() {
        //return new RotateIntakeCommand(this, -1, encoder); //rotations relative to start is -73
        return runOnce(() -> rotationController.setReference(-73, CANSparkBase.ControlType.kPosition));
    }

    /**Goes to the positions used to throw note into the amp
     * @return Command
     */
    public Command goToAMPPosition() {
        return runOnce(() -> rotationController.setReference(-33, CANSparkBase.ControlType.kPosition)); //rotations relative to start is -33
    }

    /**Goes back to the original starting position
     * @return Command
     */
    public Command goToIntakeHome() {
        return runOnce(() -> rotationController.setReference(0, CANSparkBase.ControlType.kPosition)); //rotations relative to start is 0
    }

    /**Makes the intake spit out its note
     * @return Command
     */
    public Command outtakeCommand() {
        return runOnce(() -> intakeMotor.set(-1));
    }

    /**Shoots the note in the amp
     * @return Command
     */
    public Command shootAmpCommand() {
        return runOnce(() -> {
            intakeMotor.set(-1);
            this.hasNote = false;
        });
    }

    /**Outakes slowly so we don't break things
     * @return Command
     */
    public Command shootSlowCommand() {
        return runOnce(() -> {
            intakeMotor.set(-0.1);
            this.hasNote = false;
        });
    }

    /**Moves the intake motor slowly (intake direction)
     * @return Command
     */
    public Command moveIntakeManualCommand() {
        return runOnce(() -> rotationMotor.set(.1));
    }


    /** @return Command that starts outtaking */
    // public Command outtakeCommand() {
    //     return runOnce(() -> outtake(IntakeConstants.OUTTAKE_RPM));
    // }

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
        return new ConditionalCommand(
                stopMotorCommand(),
                // This is technically not needed right now, because the control is a hold, but this will work for both hold and a press
                runOnce(() -> rotationMotor.set(0.5)).andThen(new WaitUntilCommand(this::isAtUpperPosition), stopRotatingIntake()),
                this::isAtUpperPosition
        );
    }

    /** Rotates the intake until it is in intake position */
    public Command lowerIntakeCommand() {
        return new ConditionalCommand(
                stopMotorCommand(),
                // This is technically not needed right now, because the control is a hold, but this will work for both hold and a press
                runOnce(() -> rotationMotor.set(-0.5)).andThen(new WaitUntilCommand(this::isAtLowerPosition), stopRotatingIntake()),
                this::isAtLowerPosition
        );
    }

    public Command raiseIntakeCommandPID() {
//         return run(() -> {
//             isUp = true;
//             rotateIntake(IntakeConstants.INTAKE_TO_INDEX_RPM);
//         }).until(this::atIndexPosition).andThen(this::stopRotation);
        return run(() -> {});
    }

    /** Rotates the intake until it is in intake position */
    public Command lowerIntakeCommandPID() {
//         return run(() -> {
//             isUp = false;
//             rotateIntake(IntakeConstants.INTAKE_TO_INTAKE_RPM);
//
//         }).until(this::atIntakePosition).andThen(this::stopRotation);
        return run(() -> {});
    }

    /**The name is self explanatory
     * @return Command
     */
    public Command stopRotatingIntake() {
        return runOnce(() -> rotationMotor.set(0));
    }

    /**
     * Rotate the intake to a specified position.
     * 
     * @param position The position to rotate to.
     */
    @SuppressWarnings("unused")
    private void rotateIntake(double position) {
        position = Math.min(position, IntakeConstants.ROTATION_MAX_RPM); //this is a mistake, it's the rotation motor not the intake motor and the input is a position not rpm
        //new RotateIntakeCommand(this, position, encoder).schedule();
    } 

    /**
     * Intake at a specified velocity. 
     * 
     * @param rpm The velocity to rotate at (in rpm)
     */
    @SuppressWarnings("unused")
    private void intake(double rpm) {
        intakeController.setReference(MathUtil.clamp(rpm, rpm, IntakeConstants.INTAKE_MAX_RPM), ControlType.kVelocity);
    }

    /**
     * Outtake at a specified velocity. 
     * 
     * @param rpm The velocity to rotate at (in rpm)
     */
    @SuppressWarnings("unused")
    private void outtake(double rpm) {
        intakeController.setReference(-MathUtil.clamp(rpm, rpm, IntakeConstants.OUTTAKE_MAX_RPM) , ControlType.kVelocity);
    }

    /** Stop the rotation motor */
    @SuppressWarnings("unused")
    private void stopRotation() {
        rotationMotor.set(0);
    }

    /** Stop the intake motor */
    private void stopIntaking() {
        intakeMotor.set(0);
    }
    
    /** @reutrn True if the lower limit switch is being pressed */
    public boolean isAtLowerPosition() {
        return !lowerPositionLimit.get();
    }

     /** @return True if the upper limit switch is being pressed */
    public boolean isAtUpperPosition() {
        // This is temporarily set to true because there isn't the metal bit on the limit switch
        return true;
        // return upperPositionLimit.get();
    }

    /** @return True is the intake is up or moving to up, false otherwise */
    public boolean isUp() {
        return isUp;
    }

    /** @return True is we are currently holding a note, false otherwise */
    public boolean hasNote() {
        return hasNote;
    }

    /** @return The absolute encoder on the rotation motor */
    public DutyCycleEncoder getEncoder() {
        return encoder;
    }

    /** @return The rotation motor */
    public CANSparkMax getRotationMotor() {
        return rotationMotor;
    }
}
