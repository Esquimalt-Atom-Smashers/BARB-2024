package frc.robot.subsystems;

import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;;

// https://github.com/FRC2539/javabot-2023/blob/main/src/main/java/frc/robot/subsystems/LightsSubsystem.java

public class CANdleSubsystem extends SubsystemBase {

    CANdle candle = new CANdle(CANdleConstants.CANDLE_ID);
    private CANdleConfiguration config;

    public CANdleSubsystem() {
        config = new CANdleConfiguration();
        config.brightnessScalar = 1.0;
        config.disableWhenLOS = false;
        config.statusLedOffWhenActive = true;
        config.stripType = CANdle.LEDStripType.RGB;

        setColor(CANdleConstants.DEFAULT_PINK); //Default colour setting
    }

    private void setColor(Colour colour) {
        candle.setLEDs(colour.r, colour.g, colour.b);
    }

    private void setStrobe(Colour colour) {
        candle.animate(new StrobeAnimation(colour.r, colour.g, colour.b));
    }

    public Command setColorCommand(Colour colour) {
        return runOnce(() -> setColor(colour));
    }

    public Command setStrobeCommand(Colour colour) {
        return runOnce(() -> setStrobe(colour));
    }

    public Command clearCANdleCommand() {
        return setColorCommand(new Colour(0, 0, 0));
    }

    public static class Colour {

        public int r;
        public int g;
        public int b;

        public Colour(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
}
