// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Objects;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.controller.LogitechController;
import frc.robot.Constants.ControllerConstants;
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
    private final LogitechController driverControllerLogitech = new LogitechController(0);
    private final LogitechController operatorControllerLogitech = new LogitechController(1);

    // private final LogitechController operatorController = new
    // LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(40, -40, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(40, -40, 0);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    // private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem();

    public RobotContainer(TimedRobot robot) {
        configureBindings();
    }

    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis,
                this::getDriveRotationAxis, true));

        // driverControllerLogitech.getA().onTrue(swerveDriveSubsystem.rotateCenterApriltagCommand(() -> 0.2, limelightSubsystem.getAprilTagXOffset()));
        // operatorControllerLogitech.getA().onTrue(shooterSubsystem.shootManuallyWithTimeout(-1));
        driverControllerLogitech.getA().onTrue(shooterSubsystem.shootManually(0.5));
        driverControllerLogitech.getB().onTrue(shooterSubsystem.shootManually(-0.5));

        // driveController.getA().whileTrue(
        // swerveDriveSubsystem.rotateCenterApriltagCommand(() -> 0.05,
        // limelightSubsystem.getAprilTagXOffset()));
        // driveController.getB().onTrue(blinkinSubsystem.updateColour(BlinkinValue.CONFETTI));
        // shooterSubsystem.setDefaultCommand(shooterSubsystem.getDefaultCommand());
    }

    public double getDriveForwardAxis() {
        return -forwardRateLimiter.calculate(
            square(deadband(driverControllerLogitech.getLeftYAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
    }

    public double getDriveStrafeAxis() {
            return -forwardRateLimiter.calculate(
                square(deadband(driverControllerLogitech.getLeftXAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
        
    }

    public double getDriveRotationAxis() {
            return -forwardRateLimiter.calculate(
                square(deadband(driverControllerLogitech.getRightXAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxAngularVelocity);
        
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }
}
