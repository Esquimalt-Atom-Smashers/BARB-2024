// package frc.robot;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.ShooterSubsystem;

// import static edu.wpi.first.wpilibj2.command.Commands.*;

// public class AutonomousController {
//     private RobotContainer robotContainer;

//     SendableChooser<Command> autoChooser;
//     public PathPlannerAuto center;

//     public AutonomousController(RobotContainer robotContainer) {
//         this.robotContainer = robotContainer;
//         NamedCommands.registerCommand("AutoShoot", getAutoShoot());
//         NamedCommands.registerCommand("AutoRaiseIntake", getAutoRaiseIntake());
//         NamedCommands.registerCommand("AutoLowerIntake", getAutoLowerIntake());

//         autoChooser = AutoBuilder.buildAutoChooser();

//         center = new PathPlannerAuto("Center");
//         autoChooser.setDefaultOption("Center", center);
//     }

//     private Command getAutoShoot() {
//         return sequence(
//             robotContainer.shooterSubsystem.shootAtVoltageCommand(),
//             new WaitCommand(5),
//             robotContainer.intakeSubsystem.outtakeCommand(),
//             new WaitCommand(0.5),
//             robotContainer.shooterSubsystem.stopShootingCommand(),
//             robotContainer.intakeSubsystem.stopMotorCommand()
//         );
//     }

//     private Command getAutoLowerIntake() {
//         return sequence(
//             robotContainer.intakeSubsystem.goToIntakePosition(),
//             robotContainer.intakeSubsystem.intakeCommand()
//         );
//     }

//     private Command getAutoRaiseIntake() {
//         return sequence(
//             robotContainer.intakeSubsystem.stopMotorCommand(),
//             robotContainer.intakeSubsystem.goToIntakeHome()
//         );
//     }

//     public Command getAutonomousCommand() {
//         return autoChooser.getSelected();
//     }

//     public void sendOption() {
//         SmartDashboard.putData("Auto Mode", autoChooser);
//     }
// }