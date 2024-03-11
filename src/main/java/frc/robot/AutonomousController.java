package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonomousController {
    private RobotContainer robotContainer;

    SendableChooser<Command> autoChooser;
    public PathPlannerAuto center;

    public AutonomousController(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        center = new PathPlannerAuto("Center");
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Center", center);
        SmartDashboard.putData("AutonomousMode", autoChooser);

        NamedCommands.registerCommand("AutoShoot", getAutoShoot());
        NamedCommands.registerCommand("AutoRaiseIntake", getAutoRaiseIntake());
        NamedCommands.registerCommand("AutoLowerIntake", getAutoLowerIntake());
    }

    private Command getAutoShoot() {
        return sequence(
            robotContainer.shooterSubsystem.shootAtVoltageCommand(),
            new WaitCommand(2),
            robotContainer.intakeSubsystem.outtakeCommand(),
            new WaitCommand(1),
            robotContainer.shooterSubsystem.stopShootingCommand(),
            robotContainer.intakeSubsystem.stopMotorCommand()
        );
    }

    private Command getAutoLowerIntake() {
        return sequence(
            robotContainer.intakeSubsystem.goToIntakePosition(),
            new WaitCommand(2),
            robotContainer.intakeSubsystem.autoIntakeCommand()
        );
    }

    private Command getAutoRaiseIntake() {
        return sequence(
            robotContainer.intakeSubsystem.stopMotorCommand(),
            robotContainer.intakeSubsystem.goToIntakeHome()
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void sendOption() {
        SmartDashboard.putData("Auto Mode", autoChooser);
    }
}