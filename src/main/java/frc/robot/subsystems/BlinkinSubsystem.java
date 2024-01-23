package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinSubsystem extends SubsystemBase {
    /**
     * The port the blinkin is plugged into. We use the Spark class because the
     * range of pwm values are the same.
     */
    private final Spark blinkin = new Spark(-1);

    public BlinkinSubsystem() {
        this.updateAllianceColour();
    }

    /** Command which updates the LEDs to corrospond to the alliance we on. */
    public Command updateAllianceColour() {
        return runOnce(() -> {
            boolean isBlueAlliance = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsBlueAlliance")
                    .getBoolean(true);

            blinkin.set(isBlueAlliance ? BlinkinValue.SOLID_BLUE.pwm : BlinkinValue.SOLID_RED.pwm);
        });
    }

    public Command updateColour(BlinkinValue value) {
        return runOnce(() -> {
            blinkin.set(value.pwm);
        });
    }

    /**
     * Enum for all the colour values. Retrieved from:
     * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
     */
    public enum BlinkinValue {
        /** Solid colours, no special effects. */
        SOLID_RED(0.61),
        SOLID_BLUE(0.87),
        SOLID_PINK(0.57),
        SOLID_PURPLE(0.91),

        /** Patterned colours, w/ special effects. */
        RAINBOW_WITH_GLITTER(-0.89),
        CONFETTI(-0.87);

        double pwm;

        BlinkinValue(double pwm) {
            this.pwm = MathUtil.clamp(pwm, -1.0, 1.0);
        }
    }

}