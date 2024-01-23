package frc.lib.swerve;

import java.util.Optional;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;
    public Optional<String> canivoreName = Optional.empty();
    public final static boolean isSecondOrder = true;
    public final static double[] pidValueReciever = {0.8 / 15, 0, .01, 8, 0.9}; // P I D stopAngle leveingMaxSpeed;
    public final static double angleRateThresholdReceiver = 18.0;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }

    public SwerveModuleConstants(
            int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, String canivoreName) {
        this(driveMotorID, angleMotorID, canCoderID, angleOffset);
        this.canivoreName = Optional.of(canivoreName);
    }
}