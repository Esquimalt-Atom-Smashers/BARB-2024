package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimelightEntry;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;

    private boolean validTarget;
    private double aprilTagXOffset;
    private double aprilTagYOffset;
    private double[] aprilTagId;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Command for updating a specifc limelight entry. Note, implicitly assumes the
     * number is an integer.
     */
    public Command updateConfig(Number value, LimelightEntry... entries) {
        return runOnce(() -> {
            for (LimelightEntry entry : entries) {
                table.getEntry(entry.attrName).setNumber(MathUtil.clamp((int) value, entry.min, entry.max));
            }
        });
    }

    /** @return the horizontal offset of the detected april-tag. */
    public double getAprilTagXOffset() {
        return aprilTagXOffset;
    }

    /** @return the vertical offset of the detected april-tag. */
    public double getAprilTagYOffset() {
        return aprilTagYOffset;
    }

    /** @return the primary id of the detected april-tag */
    public double getPrimaryAprilTagId() {
        return aprilTagId[0];
    }

    /** @return true if the limelight sees a target, false otherwise. */
    public boolean hasTarget() {
        return validTarget;
    }

    /** Retrieves the new entry data every frame iteration. */
    @Override
    public void periodic() {
        validTarget = (table.getEntry("tv").getDouble(0) == 1) ? true : false;
        aprilTagXOffset = table.getEntry("tx").getDouble(0);
        aprilTagYOffset = table.getEntry("ty").getDouble(0);
        aprilTagId = table.getEntry("tid").getDoubleArray(new double[6]);
    }
}
