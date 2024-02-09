package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.gyro.ADISGyro;
import frc.lib.swerve.SwerveDriveSignal;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.lib.swerve.SwerveModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
    /** The current pose of the robot. */
    // private Pose2d pose = new Pose2d();

    /** The current velocity and previous velocity of the robot. */
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private ChassisSpeeds previousVelocity = new ChassisSpeeds();

    private SwerveDriveSignal driveSignal = new SwerveDriveSignal();

    /** Array of the swerve modules on the robot */
    private SwerveModule[] modules;

    private ADISGyro gyro;

    /** Max speed supplier. */
    private DoubleSupplier maxSpeedSupplier = () -> Constants.SwerveConstants.maxSpeed;

    private SwerveDriveOdometry odometry;
    
    /**
     * Constructs a SwerveDriveSubsystem object.
     * Initializes the gyro, modules, and any autonmous variables.
     */
    public SwerveDriveSubsystem() {
        gyro = new ADISGyro();

        modules = new SwerveModule[] {
                new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
                new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
                new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
                new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        odometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getGyroRotation(), getModulePositions());

        // Configures pathplanner
        // AutoBuilder.configureHolonomic(
        //     this::getPose, 
        //     this::resetPose, 
        //     this::getVelocity, 
        //     this::setVelocity, 
        //     Constants.SwerveConstants.pathFollowerConfig, 
        //     () -> false, //Change this later 
        //     this);
    }

    /**
     * Command used for driving during tele-operated mode.
     * 
     * @param forward         The forwards velocity in meters/second.
     * @param strafe          The strafe velocity in meters/second.
     * @param rotation        The rotation velocity in radians/second.
     * @param isFieldOriented The driving orientation of the robot.
     * 
     * @return A command object.
     */
    public Command driveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, boolean isFieldOriented) {
        return run(() -> {
            setVelocity(
                    new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble()),
                    isFieldOriented);
        });
    }

    /**
     * Command which 'should' make the robot try and rotate to center with an
     * apriltag.
     */
    public Command rotateCenterApriltagCommand(DoubleSupplier min_rotation, double tx) {
        double Kp = 0.1;
        return run(() -> {
            double headingError = -tx;
            double steeringAdjust = 0;

            if (Math.abs(headingError) > 1) {
                if (headingError < 0)
                    steeringAdjust = Kp * headingError + min_rotation.getAsDouble();
                else
                    steeringAdjust = Kp * headingError - min_rotation.getAsDouble();
            }
            this.setVelocity(new ChassisSpeeds(0, 0, steeringAdjust), true);
        });
    }

    /** Command which gives the robot a little 'boost'. Use caution when calling. */
    public Command nitrosDriveCommand() {
        return runOnce(() -> {
            setCustomMaxSpeedSupplier(() -> 6.0);
        });
    }

    /** Sets the custom max speed(m/s) of the drivetrain. */
    public void setCustomMaxSpeedSupplier(DoubleSupplier maxSpeedSupplier) {
        this.maxSpeedSupplier = maxSpeedSupplier;
    }

    /** @return The pose of the robot. (used in pathplanner)*/
    public Pose2d getPose() {
        //return pose; isn't pose always 0?
        return odometry.getPoseMeters();
    }

    /** Resets pose of the robot. (used in pathplanner) */
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    // public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
    //     SwerveModuleState[] targetStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    //     setModuleStates(targetStates, false);
    //   }    

    /** @return The robot relative velocity of the drivetrain. */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    /** @return The relative acceleration of the drivetrain. */
    public ChassisSpeeds getAcceleration() {
        return new ChassisSpeeds(
                velocity.vxMetersPerSecond - previousVelocity.vxMetersPerSecond,
                velocity.vyMetersPerSecond - previousVelocity.vyMetersPerSecond,
                velocity.omegaRadiansPerSecond - previousVelocity.omegaRadiansPerSecond);
    }

    /** @return The potentially field relative desired velocity of the drivetrain */
    public ChassisSpeeds getDesiredVelocity() {
        return (ChassisSpeeds) driveSignal;
    }

    /** @return The magnitude of the velocity on the robot. */
    public double getVelocityMagnitude() {
        return Math.sqrt(Math.pow(velocity.vxMetersPerSecond, 2) + Math.pow(velocity.vyMetersPerSecond, 2));
    }

    /** @return The rotation velocity of the robot. */
    public Rotation2d getVelocityRotation() {
        return (new Translation2d(velocity.vxMetersPerSecond, velocity.vxMetersPerSecond)).getAngle();
    }

    /** @return The Rotation2d of the gyro. */
    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    /** @return the Rotation2d of the robots pose. */
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    /** @return The Rotation3d of the gyro. */
    // public Rotation3d getGyroRotation3d() {
    //     return gyro.getRotation3d();
    // }

    // public Translation3d getNormalVector3d() {
    //     return new Translation3d(0, 0, 1).rotateBy(getGyroRotation3d());
    // }

    /**
     * Instantiates the driveSignal based on the given parameters.
     *
     * @param velocity        The velocity of the robot in ChassisSpeeds.
     * @param isFieldOriented true if it is field oriented, otherwise false.
     * @param isOpenLoop      true if it is field oriented, otherwise false.
     */
    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented, boolean isOpenLoop) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented, isOpenLoop);
    }

    /**
     * Instantiates the driveSignal based on the given parameters.
     * 
     * @param velocity        The velocity of the robot in ChassisSpeeds.
     * @param isFieldOriented true if it is field oriented, otherwise false.
     */
    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented) {
        setVelocity(velocity, isFieldOriented, true);
    }

    /**
     * Instantiates the driveSignal based on the given parameters.
     * 
     * @param velocity The velocity of the robot in ChassisSpeeds.
     */
    public void setVelocity(ChassisSpeeds velocity) {
        setVelocity(velocity, false);
    }

    /** Stops all drive movement of the robot. */
    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    /**
     * Called every frame (approx, 30ms), this is where the odometry and the modules
     * are updated.
     */
    public void update() {
        updateOdometry();

        updateModules(driveSignal);
        odometry.update(getGyroRotation(), getModulePositions());

    }

    /** Updates the current robot odometry. */
    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();

        previousVelocity = velocity;
        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);
    }

    /** Updates the modules based on the given driveSignal. */
    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;

        if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getRotation());
        } else {
            chassisVelocity = (ChassisSpeeds) driveSignal;
        }

        SwerveModuleState[] moduleStates = Constants.SwerveConstants.swerveKinematics
                .toSwerveModuleStates(chassisVelocity);

        setModuleStates(moduleStates, isDriveSignalStopped(driveSignal) ? true : driveSignal.isOpenLoop());
    }

    /** Sets all the states of the swerve modules. */
    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedSupplier.getAsDouble());

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, SwerveModuleConstants.isSecondOrder);
        }
    }

    /** @return true if the driveSignal is stopped, false otherwise. */
    private boolean isDriveSignalStopped(SwerveDriveSignal driveSignal) {
        return driveSignal.vxMetersPerSecond == 0
                && driveSignal.vyMetersPerSecond == 0
                && driveSignal.omegaRadiansPerSecond == 0;
    }

    @Override
    public void periodic() {
        update();
        System.out.println("Gyro Rotation: " + gyro.getRotation2d());
    }

    /** @return the current states of all the swerve modules. */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    /** @return the current positions of all the swerve modules. */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }
}
