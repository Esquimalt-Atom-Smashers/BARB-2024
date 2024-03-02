// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Objects;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.LogitechController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.BlinkinSubsystem.BlinkinValue;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private LogitechController driverLogitechController;
    private LogitechController operatorLogitechController;

    public AutonomousController autonomousController;

    // private final LogitechController operatorControllerLogitech = new LogitechController(1);

    // private final LogitechController operatorController = new
    // LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(40, -40, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(40, -40, 0);

    /** The swerve drive base of the robot */
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    // private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    /** The shooter on the robot */
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final TrapDoorSubsystem trapDoorSubsystem = new TrapDoorSubsystem();
    private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem();
    
    public boolean intakeControls;
    
    public RobotContainer(TimedRobot robot) {
        autonomousController = new AutonomousController(this);
        autonomousController.sendOption();
        
        driverLogitechController = new LogitechController(0);
        operatorLogitechController = new LogitechController(1);
        configureLogitechBindings();
    }
    
    /** Configure the controls for the logitech controller */
    private void configureLogitechBindings() {
        // if (driverLogitechController == null) return;

        // Trapdoor
        driverLogitechController.getDPadUp().onTrue(trapDoorSubsystem.extendCommand());
        driverLogitechController.getDPadDown().onTrue(trapDoorSubsystem.retractCommand());

        driverLogitechController.getDPadRight().onTrue(trapDoorSubsystem.winchCommand()).onFalse(trapDoorSubsystem.stopWinchMotor());
        
        // LJ - Updown = Forward/Reverse (DRIVE)
        // RJ - Left/Right = Strafe (DRIVE)
        driverLogitechController.getRightXAxis().setInverted(true);

        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
            () -> getDriveForwardAxis(), () -> getDriveStrafeAxis(),
            () -> getDriveRotationAxis(), true));
            
        driverLogitechController.getBack().onTrue(swerveDriveSubsystem.resetGyro());
            // DPAD = 90 degree snap
            //        driverLogitechController.getDPadLeft().onTrue(swerveDriveSubsystem.snap90LeftCommand());
            //        driverLogitechController.getDPadRight().onTrue(swerveDriveSubsystem.snap90RightCommand());
            
            // LJ Push - (Toggle) = Slowmode
            //        driverLogitechController.getLeftJoystick().whileTrue(swerveDriveSubsystem.enableSlowMode()).onFalse(swerveDriveSubsystem.disableSlowMode());
            
            // driverLogitechController.getDPadUp().onTrue(blinkinSubsystem.setColor(BlinkinValue.SOLID_PINK));
            // driverLogitechController.getDPadDown().onTrue(blinkinSubsystem.setColor(BlinkinValue.CONFETTI));
            
            // = Intake
            //        driverLogitechController.getX().whileTrue(intakeSubsystem.intakeCommand()).onFalse(intakeSubsystem.stopMotorCommand());
            //        driverLogitechController.getY().whileTrue(intakeSubsystem.outtakeCommand()).onFalse(intakeSubsystem.stopMotorCommand());
            
            // Only be able to control the intake if it starts in the upper position
            // UNCOMMENT
            if (intakeSubsystem.isAtUpperPosition()) {
                bindIntakeCommands();    
            }
            else {
                intakeControls = false;
                operatorLogitechController.getA().onTrue(intakeSubsystem.moveIntakeManualCommand()).onFalse(intakeSubsystem.stopMotorCommand());
                new Trigger(() -> intakeSubsystem.isAtUpperPosition()).onTrue(new InstantCommand(() -> {
                    bindIntakeCommands();

                }));
                SmartDashboard.putString("Intake Controls", "NOT initialized. Press switch and restart code.");
                System.out.println("!!! Intake subsystem didn't start in the correct position !!!");
            }
            
            // Intake Rotation (Manual)
            // driverLogitechController.getLeftBumper().whileTrue(intakeSubsystem.lowerIntakeCommand()).onFalse(intakeSubsystem.stopRotatingIntake());
            // driverLogitechController.getRightBumper().whileTrue(intakeSubsystem.raiseIntakeCommand()).onFalse(intakeSubsystem.stopRotatingIntake());
            
            // Intake Rotation (PID)
            //        driverLogitechController.getLeftBumper().whileTrue(intakeSubsystem.raiseIntakeCommandPID());
            //        driverLogitechController.getRightBumper().whileTrue(intakeSubsystem.lowerIntakeCommandPID());
                        
        // driverLogitechController.getLeftBumper().onTrue(swerveDriveSubsystem.autoDriveForwardCommand());
        // driverLogitechController.getDPadRight().onTrue(shooterSubsystem.setAppliedVoltage(0.05));
        // driverLogitechController.getDPadLeft().onTrue(shooterSubsystem.setAppliedVoltage(-0.05));
                
        // driverControllerLogitech.getA().onTrue(swerveDriveSubsystem.rotateCenterApriltagCommand(() -> 0.2, limelightSubsystem.getAprilTagXOffset()));
        // operatorControllerLogitech.getA().onTrue(shooterSubsystem.shootManuallyWithTimeout(-1));

    //    driverLogitechController.getDPadLeft().whileTrue(trapDoorSubsystem.extendCommand()).onFalse(trapDoorSubsystem.stopCommand());
    //    driverLogitechController.getDPadRight().whileTrue(trapDoorSubsystem.retractCommand()).onFalse(trapDoorSubsystem.stopCommand());
        // driverLogitechController.getDPadUp().whileTrue(trapDoorSubsystem.extendCommand()).onFalse(trapDoorSubsystem.stopCommand());
        // driverLogitechController.getDPadDown().whileTrue(trapDoorSubsystem.retractCommand()).onFalse(trapDoorSubsystem.stopCommand());
        // driverLogitechController.getLeftBumper().whileTrue(shooterSubsystem.shootAtVoltageCommand(ShooterSubsystem.appliedVoltage)).onFalse(shooterSubsystem.shootAtVoltageCommand(0));
        // driverLogitechController.getRightBumper().whileTrue(shooterSubsystem.shootAtVoltageCommand(-ShooterSubsystem.appliedVoltage)).onFalse(shooterSubsystem.shootAtVoltageCommand(0));

        driverLogitechController.getA().whileTrue(shooterSubsystem.shootAtVoltageCommand()).onFalse(shooterSubsystem.stopShootingCommand());
        driverLogitechController.getLeftBumper().whileTrue(swerveDriveSubsystem.enableSlowMode()).onFalse(swerveDriveSubsystem.disableSlowMode());
        // driverLogitechController.getX().whileTrue(shooterSubsystem.shootAtVoltageCommand(0.7)).onFalse(shooterSubsystem.stopShootingCommand());
        // driverLogitechController.getY().whileTrue(shooterSubsystem.shootAtVoltageCommand(-0.7)).onFalse(shooterSubsystem.stopShootingCommand());        
        // driverLogitechController.getDPadLeft().onTrue(swerveDriveSubsystem.autoDriveForwardCommand());
        // driveController.getA().whileTrue(
        // swerveDriveSubsystem.rotateCenterApriltagCommand(() -> 0.05,
        // limelightSubsystem.getAprilTagXOffset()));
        // driveController.getB().onTrue(blinkinSubsystem.updateColour(BlinkinValue.CONFETTI));
        // shooterSubsystem.setDefaultCommand(shooterSubsystem.getDefaultCommand());
    }

    private void bindIntakeCommands() {
        intakeControls = true;
        SmartDashboard.putString("Intake Controls", "Initialized");
        driverLogitechController.getX().onTrue(intakeSubsystem.goToIntakeHome());
        driverLogitechController.getY().onTrue(intakeSubsystem.goToIntakePosition());
        driverLogitechController.getB().onTrue(intakeSubsystem.goToAMPPosition());
        driverLogitechController.getRightTrigger().whileTrue(intakeSubsystem.shootAmpCommand()).onFalse(intakeSubsystem.stopMotorCommand());
        driverLogitechController.getRightBumper().whileTrue(intakeSubsystem.intakeCommand()).onFalse(intakeSubsystem.stopMotorCommand());
        // driverLogitechController.getLeftTrigger().whileTrue(intakeSubsystem.shootAmpCommand()).onFalse(intakeSubsystem.stopMotorCommand());
        // driverLogitechController.getLeftBumper().whileTrue(intakeSubsystem.intakeCommand()).onFalse(intakeSubsystem.stopMotorCommand());

    }

    // Autonomous path that shoots, moves forward while intaking, moves back and shoots
    public Command autoCenterPath() {
        return new SequentialCommandGroup(
            shooterSubsystem.shootAtVoltageCommand(),
            new WaitCommand(2),
            intakeSubsystem.outtakeCommand(),
            new WaitCommand(1),
            intakeSubsystem.intakeCommand(),
            shooterSubsystem.stopShootingCommand(),
            intakeSubsystem.goToIntakePosition(),
            new WaitCommand(1),
            swerveDriveSubsystem.autoDriveCommand(1.5, 0),
            intakeSubsystem.goToIntakeHome(),
            // Start ramping up before we get there
            shooterSubsystem.shootAtVoltageCommand(),
            swerveDriveSubsystem.autoDriveCommand(-1.5, 0),
            intakeSubsystem.outtakeCommand(),
            new WaitCommand(1),
            intakeSubsystem.stopMotorCommand(),
            shooterSubsystem.stopShootingCommand()
        );
    }

    public Command autoTestDrive() {
        return new SequentialCommandGroup(
            swerveDriveSubsystem.autoDriveCommand(2, 0),
            new InstantCommand(() -> System.out.println("Command done")),
            new WaitCommand(2),
            swerveDriveSubsystem.autoDriveCommand(0, 2),
            new WaitCommand(2),
            swerveDriveSubsystem.autoDriveCommand(-2, -2)
        );
    }
    //hello!


    /** @return The axis used to drive forwards, scaled and limited by the slew rate limiter */
    public double getDriveForwardAxis() {
        return -forwardRateLimiter.calculate(
            square(deadband(driverLogitechController.getLeftYAxis().getRaw(), 0.05)) * swerveDriveSubsystem.getMaxSpeedSupplier().getAsDouble());
    }
        
    /** @return The axis used to strafe, scaled and limited by the slew rate limiter */
    public double getDriveStrafeAxis() {
        return -strafeRateLimiter.calculate(
            square(deadband(driverLogitechController.getLeftXAxis().getRaw(), 0.05)) * swerveDriveSubsystem.getMaxSpeedSupplier().getAsDouble());            
    }
                
    /** @return The axis used to turn, scaled and limited by the slew rate limiter */
    public double getDriveRotationAxis() {
        return square(deadband(driverLogitechController.getRightXAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxAngularVelocity;
    }

    /**
     * Takes a value and a tolerance and determines if the absolute value of value is greater that the tolerance. 
     * If it is, returns 0, else returns value.
     * 
     * @param value The value to check
     * @param tolerance The tolerance for the value
     * @return The value, if the magnitude of value if greater than tolerance
     */
    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        // This isn't achieving much
        // return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
        return value;
    }

    /**
     * Takes a value and squares it, keeping its sign the same. Equivalent to |value| * value.
     * 
     * @param value The value to square
     * @return The squared value, with the same sign as the original value
     */
    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public Command getAutoCommand() {
        return autonomousController.getAutonomousCommand();
    }
}
