// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.SecondOrderSwerveKinematics;
import frc.lib.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean competitionMode = false;

  public static final class GlobalConstants {
      //public static final String CANIVORE_NAME = "CANivore";
      public static final int PCM_ID = 19;
      public static final double targetVoltage = 12.0; // Used for voltage compensation

      public static final double batteryVoltageThreshold = 12.3;

      //public static final double minimumPressure = 100; // PSI
      //public static final double maximumPressure = 120; // try 120
  }

  public static final class ControllerConstants {
      public static final int DRIVE_CONTROLLER = 0;
      public static final int OPERATOR_CONTROLLER = 1;
  }


  public static final class SwerveConstants extends CompBotConstants {
      
  }

  public static class CompBotConstants {
      public static final double lengthWithBumpers = Units.inchesToMeters(32 + 3.25 * 2);
      public static final double widthWithBumpers = Units.inchesToMeters(27.5 + 3.25 * 2);

      public static final double trackWidth = Units.inchesToMeters(21.5);
      public static final double wheelBase = Units.inchesToMeters(26.75);
      public static final double wheelDiameter = Units.inchesToMeters(4.0);
      public static final double wheelCircumference = wheelDiameter * Math.PI;

      public static final double robotMass = Units.lbsToKilograms(45);

      public static final double openLoopRamp = 0.25; // 0.25
      public static final double closedLoopRamp = 0.0;

      public static final double driveGearRatio = (6.75 / 1.0); // 5.14:1
      public static final double angleGearRatio = (150/7 / 1.0); // 12.8:1

      public static final Translation2d[] moduleTranslations = new Translation2d[] {
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

      public static final SecondOrderSwerveKinematics swerveKinematics =
              new SecondOrderSwerveKinematics(moduleTranslations);

      /* Swerve Current Limiting */
      public static final int angleContinuousCurrentLimit = 25;
      public static final int anglePeakCurrentLimit = 40;
      public static final double anglePeakCurrentDuration = 0.1;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveContinuousCurrentLimit = 35;
      public static final int drivePeakCurrentLimit = 60;
      public static final double drivePeakCurrentDuration = 0.1;
      public static final boolean driveEnableCurrentLimit = true;

      /* Angle Motor PID Values */
      public static final double angleKP = 43;
      public static final double angleKI = 0.0;
      public static final double angleKD = 0.0;

      /* Drive Motor PID Values */
      public static final double driveKP = 0.1;
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0;

      /* Motor Information */
      public static final double driveMotorFreeSpeed = 6380; // RPM of Falcon 500
      public static final double angleMotorFreeSpeed = 6380; // RPM of Falcon 500
      public static final double stallTorque = 4.69;

      /* Drive Motor Characterization Values */
      public static final double driveKS =
              (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
      public static final double driveKV = (2.44 / 12);
      public static final double driveKA = (0.27 / 12);

      /* Angle Motor Characterization Values */
      public static final double angleKS = 0;
      // (0.368 / 12); // divide by 12 to convert from volts to percent output for CTRE
      public static final double angleKV = (0.234 / 12);
      public static final double angleKA = (0.003 / 12);

      /* Swerve Profiling Values */
      public static final double maxSpeed = 5.00; // (6.52) meters per second
      public static final double maxAcceleration =
              (stallTorque * driveGearRatio * 4) / (wheelDiameter * robotMass); // 16.52; // meters per second^2
      public static final double maxAngularVelocity = 5; // rad/s
        //       / Arrays.stream(moduleTranslations)
        //               .map(translation -> translation.getNorm())
        //               .max(Double::compare)
        //               .get();

      /* Calculated Characterization Values */
      public static final double calculatedDriveKS = 0;
      public static final double calculatedDriveKV = (12 / maxSpeed) / GlobalConstants.targetVoltage;
      public static final double calculatedDriveKA = (12 / maxAcceleration) / GlobalConstants.targetVoltage;
      public static final double calculatedAngleKV =
              (12 * 60) / (angleMotorFreeSpeed * Math.toRadians(360 / angleGearRatio));

      /* Precise Driving Mode Values */
      public static final double preciseDrivingModeSpeedMultiplier = 0.2;

      /* Neutral Modes */
      public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

      /* Drive Motor Inverts */
      public static final boolean driveMotorInvert = false;

      /* Drive Encoder Inverts */
      public static final boolean driveEncoderInvert = false;

      /* Angle Motor Inverts */
      public static final boolean angleMotorInvert = true;

      /* Angle Encoder Invert */
      public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

      /* PathPlanner Configuration */
      public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      maxSpeed, 
      moduleTranslations[0].getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );

      /* Module Specific Constants */
      // Note, bevel gears should face left (relative to back-to-front)

      /* Front Left Module - Module 0 */
      public static final class Mod0 {
          public static final int driveMotorID = 1;
          public static final int angleMotorID = 0;
          public static final int canCoderID = 0;
          public static final double angleOffset = (.1582 * 360) - 45 + 180; 
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 {

          public static final int driveMotorID = 5;
          public static final int angleMotorID = 4;
          public static final int canCoderID = 2;
          public static final double angleOffset = (.9797 * 360) - 90 + 180;
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Left Module - Module 2 */
      public static final class Mod2 {
          public static final int driveMotorID = 3;
          public static final int angleMotorID = 2;
          public static final int canCoderID = 1;
          public static final double angleOffset = (.6382 * 360) - 90;
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 {
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 6;
          public static final int canCoderID = 3;
          public static final double angleOffset = (.667 * 360) - 45;
          //public static final String canivoreName = "CANivore";
          public static final SwerveModuleConstants constants =
                  new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  }

    public static final class IntakeConstants {
        /** Hardware ports */
        public static final int INTAKE_MOTOR_PORT = -1;
        public static final int ROTATION_MOTOR_PORT = -1;

        public static final double INTAKE_LIMIT_SWITCH_PORT = -1;
        public static final double ROTATION_LIMIT_SWITCH_PORT = -1;

        /** Hardware behaviour constants */
        public static final IdleMode INTAKE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ROTATION_IDLE_MODE = IdleMode.kBrake;

        public static final double INTAKE_VOLTAGE_COMPENSATION = 12.2;
        public static final double ROTATION_VOLTAGE_COMPENSATION = 12.2;

        public static final boolean INTAKE_MOTOR_INVERTED = false;
        public static final boolean ROTATION_MOTOR_INVERTED = false;

        /** PID Controller constants */ //TODO: determine values
        public static final double INTAKE_CONTROLLER_KP = 6e-5; 
        public static final double INTAKE_CONTROLLER_KI = 0; 
        public static final double INTAKE_CONTROLLER_KD = 0; 
        public static final double INTAKE_CONTROLLER_IZ = 0; 
        public static final double INTAKE_CONTROLLER_FF = 1.5e-5; 

        public static final double ROTATION_CONTROLLER_KP = -1;
        public static final double ROTATION_CONTROLLER_KI = -1;
        public static final double ROTATION_CONTROLLER_KD = -1;
        public static final double ROTATION_CONTROLLER_IZ = -1;
        public static final double ROTATION_CONTROLLER_FF = -1;

        // HI -JAKE

        /** Velocity Constants */
        public static final double INTAKE_MAX_RPM = 5000;
        public static final double OUTTAKE_MAX_RPM = -2000;
        public static final double ROTATION_MAX_RPM = -1;

        public static final double ROTATION_MAX_VOLTAGE = -1;
        public static final double INTAKE_MAX_VOLTAGE = 12.2;

        public static final double INTAKE_RPM = 4000;
        public static final double OUTTAKE_RPM = 3000;
        public static final double INTAKE_TO_INDEX_RPM = -1;
        public static final double INTAKE_TO_INTAKE_RPM = 1;
    }

    public static class ShooterConstants {
        public static final int LEFT_SHOOTER_PORT = 2;
        public static final int RIGHT_SHOOTER_PORT = 1;

        public static final double VOLTAGE_COMPENSATION = 12.2;

        public static final double LEFT_SHOOTER_PID_K_P = -1; 
        public static final double LEFT_SHOOTER_PID_K_I = -1; 
        public static final double LEFT_SHOOTER_PID_K_D = -1; 

        public static final double RIGHT_SHOOTER_PID_K_P = -1; 
        public static final double RIGHT_SHOOTER_PID_K_I = -1; 
        public static final double RIGHT_SHOOTER_PID_K_D = -1; 

        public static final double MAX_SHOOTER_RPM = -1;
        
        public static final double SHOOT_RPM = -1;     
    }
    
    public static class TrapDoorConstants {
        public static final int FORWARD_PORT = -1;
        public static final int REVERSE_PORT = -1;
    }
}
