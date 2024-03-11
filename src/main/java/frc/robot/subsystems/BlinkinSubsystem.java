package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.BlinkinConstants.BlinkinPattern;

/** A subsystem that represents the blinkin lights on the robot. */
public class BlinkinSubsystem extends SubsystemBase {

    /** The blinkin lights on the robot. */
    private final PWM blinkin = new PWM(BlinkinConstants.BLINKIN_PORT);

    /**
     * Constructs a BlinkinSubsystem. 
     */
    public BlinkinSubsystem() {
        setColor(BlinkinPattern.SOLID_PINK);
    }
    
    /**
     * @param pattern The pattern to set the lights to
     * @return A command that sets the light to a specific pattern
     */
    public Command updatePattern(BlinkinPattern pattern) {
        return runOnce(() -> {
            setColor(pattern);
        });
    }

    /**
     * Sets the lights to a specific pattern.
     * 
     * @param pattern The light pattern to set to
     */
    public void setColor(BlinkinConstants.BlinkinPattern pattern) {
        blinkin.setSpeed(pattern.pwm);
    }
}