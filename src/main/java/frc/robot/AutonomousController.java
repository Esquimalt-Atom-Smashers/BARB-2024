package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class AutonomousController {

    SendableChooser<Command> autoChooser;

    public AutonomousController() {
        NamedCommands.registerCommand("AutoShoot", getAutoShoot());
        NamedCommands.registerCommand("AutoRaiseIntake", getAutoRaiseIntake());
        NamedCommands.registerCommand("AutoLowerIntake", getAutoLowerIntake());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private Command getAutoShoot() {
        return print("No content");

        // return runOnce(() -> {
            
        // });
    }

    private Command getAutoLowerIntake() {
        return print("No content");

        // return runOnce(() -> {

        // });
    }

    private Command getAutoRaiseIntake() {
        return print("No content");

        // return runOnce(() -> {

        // });
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}