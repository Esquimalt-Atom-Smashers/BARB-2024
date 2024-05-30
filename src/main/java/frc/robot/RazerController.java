package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class RazerController extends GenericHID {
    public enum Button {
        kLeftBumper(5),
        kRightBumper(6),
        kLeftStick(9),
        kRightStick(10),
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        kShare(7),
        kMenu(8);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Axis {
        kLeftX(0),
        kRightX(4),
        kLeftY(1),
        kRightY(5),
        kLeftTrigger(2),
        kRightTrigger(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public RazerController(int port) {
        super(port);
    }

    //Get axis values

    public double getLeftX() {
        return getRawAxis(Axis.kLeftX.value);
    }

    public double getRightX() {
        return getRawAxis(Axis.kRightX.value);
    }

    public double getLeftY() {
        return getRawAxis(Axis.kLeftY.value);
    }

    public double getRightY() {
        return getRawAxis(Axis.kRightY.value);
    }

    public double getLeftTrigger() {
        return getRawAxis(Axis.kLeftTrigger.value);
    }

    public double getRightTrigger() {
        return getRawAxis(Axis.kRightTrigger.value);
    }

    //Get button states

    public boolean getLeftBumper() {
        return getRawButton(Button.kLeftBumper.value);
    }

    public boolean getRightBumper() {
        return getRawButton(Button.kRightBumper.value);
    }

    public boolean getleftStickButton() {
        return getRawButton(Button.kLeftStick.value);
    }

    public boolean getRightStickButton() {
        return getRawButton(Button.kRightStick.value);
    }

    public boolean getA() {
        return getRawButton(Button.kA.value);
    }

    public boolean getB() {
        return getRawButton(Button.kB.value);
    }

    public boolean getX() {
        return getRawButton(Button.kX.value);
    }

    public boolean getY() {
        return getRawButton(Button.kY.value);
    }

    public boolean getShare() {
        return getRawButton(Button.kShare.value);
    }

    public boolean getMenu() {
        return getRawButton(Button.kMenu.value);
    }

    //Get button pressed
    
    public boolean getLeftBumperPressed() {
        return getRawButtonPressed(Button.kLeftBumper.value);
    }

    public boolean getRightBumperPressed() {
        return getRawButtonPressed(Button.kRightBumper.value);
    }

    public boolean getLeftStickButtonPressed() {
        return getRawButtonPressed(Button.kLeftStick.value);
    }

    public boolean getRightStickButtonPressed() {
        return getRawButtonPressed(Button.kRightStick.value);
    }

    public boolean getAPressed() {
        return getRawButtonPressed(Button.kA.value);
    }

    public boolean getBPressed() {
        return getRawButtonPressed(Button.kB.value);
    }

    public boolean getXPressed() {
        return getRawButtonPressed(Button.kX.value);
    }

    public boolean getYPressed() {
        return getRawButtonPressed(Button.kY.value);
    }

    public boolean getSharePressed() {
        return getRawButtonPressed(Button.kShare.value);
    }

    public boolean getMenuPressed() {
        return getRawButtonPressed(Button.kMenu.value);
    }

    //Get button released

    public boolean getLeftBumperReleased() {
        return getRawButtonReleased(Button.kLeftBumper.value);
    }

    public boolean getRightBumperReleased() {
        return getRawButtonReleased(Button.kRightBumper.value);
    }

    public boolean getLeftStickButtonReleased() {
        return getRawButtonReleased(Button.kLeftStick.value);
    }

    public boolean getRightStickButtonReleased() {
        return getRawButtonReleased(Button.kRightStick.value);
    }

    public boolean getAReleased() {
        return getRawButtonReleased(Button.kA.value);
    }

    public boolean getBReleased() {
        return getRawButtonReleased(Button.kB.value);
    }

    public boolean getXReleased() {
        return getRawButtonReleased(Button.kX.value);
    }

    public boolean getYReleased() {
        return getRawButtonReleased(Button.kY.value);
    }

    public boolean getShareReleased() {
        return getRawButtonReleased(Button.kShare.value);
    }

    public boolean getMenuReleased() {
        return getRawButtonReleased(Button.kMenu.value);
    }

    //Rumble

    public void rumble() {
        setRumble(RumbleType.kBothRumble, 1);
    }

    public void rumble(double amount) {
        setRumble(RumbleType.kBothRumble, amount);
    }

    public void rumbleLeft() {
        setRumble(RumbleType.kLeftRumble, 1);
    }

    public void rumbleLeft(int amount) {
        setRumble(RumbleType.kLeftRumble, amount);
    }

    public void rumbleRight() {
        setRumble(RumbleType.kRightRumble, 1);
    }

    public void rumbleRight(int amount) {
        setRumble(RumbleType.kRightRumble, amount);
    }
}
