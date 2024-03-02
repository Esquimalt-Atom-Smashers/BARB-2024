package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class RotateIntakeCommand extends Command{

    private IntakeSubsystem intake;
    private double targetPosition;
    private double giveOrTake = 10; //change later
    private DutyCycleEncoder encoder;
    private PIDController pidController = new PIDController(0.04, ROTATION_CONTROLLER_KI, ROTATION_CONTROLLER_KD);

    public RotateIntakeCommand(IntakeSubsystem intake, double targetPosition, DutyCycleEncoder encoder) {

        this.intake = intake;
        this.targetPosition = targetPosition;
        this.encoder = encoder; 

        addRequirements(intake);

    }

    @Override
    public void execute() {
        intake.rotationMotor.set(pidController.calculate(encoder.getAbsolutePosition(), targetPosition));
    }

    @Override
    public boolean isFinished() {
        return (encoder.getAbsolutePosition() > (targetPosition - giveOrTake)) && (encoder.getAbsolutePosition() < (targetPosition + giveOrTake));
    }
}