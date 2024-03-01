// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * This code was adapted from https://github.com/FRC2539/javabot-2023/
 * In order for this project to run, you need to do the following: 
 * 0) Create your project using the WPILib Project Creator or clone the enitre repository
 * 1) Install the Extension Pack for Java https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack
 * 2) Install Java JDK 17 https://code.visualstudio.com/docs/java/java-project#:~:text=If%20you%20want%20to%20download,Code%20with%20new%20Java%20versions
 * 3) Update the gradle file if necessary to use JDK 17
 * 4) Install the following WPILib online 3rd party libraries according to https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html
 * 5) Phoenix (v6): https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-beta-latest.json and v5 https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-beta-latest.json
 * 6) Kauai Labs - Libraries for NavX-MXP, NavX-Micro, and Sensor Fusion https://dev.studica.com/releases/2023/NavX.json
 * 7) REV Robotics REVLib - Library for all REV devices including SPARK MAX and Color Sensor V3 https://software-metadata.revrobotics.com/REVLib-2023.json
 * 8) PhotonVision - Library for PhotonVision CV software https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
 * 9) PathPlanner - Library for PathPlanner https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json - note here that a previous version is used in the vendordeps folder
 */

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.swerve.CTREConfigs;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static CTREConfigs ctreConfigs = new CTREConfigs();
  public SendableChooser<String> controllerType = new SendableChooser<>();
  public SendableChooser<String> autonomousMode = new SendableChooser<>();
  public static String selectedController = "Logitech";
  public static String selectedAutonomous = "Testing";
  private RobotContainer m_robotContainer;

  public SendableChooser<Command> autonomousChooser = new SendableChooser<>();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(this);
    // m_robotContainer.autonomousController.sendOption();
    autonomousChooser.setDefaultOption("Test", m_robotContainer.autoTestDrive());
    autonomousChooser.addOption("Center", m_robotContainer.autoCenterPath());
    SmartDashboard.putData("Autonomous Mode", autonomousChooser);
    controllerType.addOption("Logitech", selectedController);
    controllerType.addOption("XBox", selectedController);
    SmartDashboard.putData("Controller Type", controllerType);

    // autonomousMode.addOption("Testing", selectedAutonomous);
    // autonomousMode.addOption("Amp Side", selectedAutonomous);
    // autonomousMode.addOption("Center", selectedAutonomous);
    // autonomousMode.addOption("Source Side", selectedAutonomous);
    // SmartDashboard.putData("Autonomous Mode", autonomousMode);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  //  TalonFX turnMotor = new TalonFX(0);
  //  DutyCycleOut out = new DutyCycleOut(0.2);

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // System.out.println(m_robotContainer.autonomousController.getAutonomousCommand().getName());
    // turnMotor.setControl(out);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    // This will be used in the future when we want to pick which auto
    // autonomousChooser.getSelected().schedule();
    // Currently, just use the test one
    m_robotContainer.autoTestDrive().schedule();

    // if (m_robotContainer.getAutoCommand() != null) {
    //   m_robotContainer.getAutoCommand().schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
