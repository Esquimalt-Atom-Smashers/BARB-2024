package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;;

// https://github.com/FRC2539/javabot-2023/blob/main/src/main/java/frc/robot/subsystems/LightsSubsystem.java

public class CANdleSubsystem extends SubsystemBase {
    CANdle candle = new CANdle(CANdleConstants.CANDLE_PORT);

    public Command setColorCommand(Colour colour) {
        return runOnce(() -> candle.setLEDs(colour.r, colour.g, colour.b));
    }

    public Command clearCANdleCommand() {
        // Set the lights to black
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
