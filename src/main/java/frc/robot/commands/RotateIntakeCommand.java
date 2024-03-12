package frc.robot.commands;

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

    /**Rotates the intake using a fancy absolute encoder
     * @param intake - the intake subsystem so it can actually move the intake
     * @param targetPosition - however much you want to rotate the intake
     */
    public RotateIntakeCommand(IntakeSubsystem intake, double targetPosition) {

        this.intake = intake;
        this.targetPosition = targetPosition;
        this.encoder = intake.getEncoder(); 

        encoder.setDistancePerRotation(1);

        addRequirements(intake);

    }

    @Override 
    public void execute() {
        intake.getRotationMotor().set(pidController.calculate(encoder.getDistance(), targetPosition)); //could be getAbsoluteValue()
    }

    @Override
    public boolean isFinished() {
        return Math.abs(encoder.getDistance() - targetPosition) <= giveOrTake;
    }
}