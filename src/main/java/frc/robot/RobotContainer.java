// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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
    // private final LogitechController driveController = new LogitechController(ControllerConstants.DRIVE_CONTROLLER);
    private final XboxController driveController = new XboxController(0);
    // private final LogitechController operatorController = new
    // LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(40, -40, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(40, -40, 0);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    // private final LimelightSubsystem limelightSubsystem = new
    // LimelightSubsystem();
    // private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem();

    public RobotContainer(TimedRobot robot) {
        configureBindings();
    }

    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis,
                this::getDriveRotationAxis, true));

        // driveController.getA().whileTrue(
        // swerveDriveSubsystem.rotateCenterApriltagCommand(() -> 0.05,
        // limelightSubsystem.getAprilTagXOffset()));
        // driveController.getB().onTrue(blinkinSubsystem.updateColour(BlinkinValue.CONFETTI));
    }

    public double getDriveForwardAxis() {
        return -forwardRateLimiter.calculate(
                square(deadband(driveController.getLeftY(), 0.05)) * Constants.SwerveConstants.maxSpeed);
    }

    public double getDriveStrafeAxis() {
        return -strafeRateLimiter.calculate(
                square(deadband(driveController.getLeftX(), 0.05)) * Constants.SwerveConstants.maxSpeed * 0.75);
    }

    public double getDriveRotationAxis() {
        return -square(deadband(driveController.getRightX(), 0.05))
                * Constants.SwerveConstants.maxAngularVelocity
                * 0.75;
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
