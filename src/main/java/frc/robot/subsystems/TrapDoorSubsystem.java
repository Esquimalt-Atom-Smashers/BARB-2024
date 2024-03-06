package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapDoorConstants;

/*
 * Subsystem representing the pneumatics on the shooter and the winch .
 */
public class TrapDoorSubsystem extends SubsystemBase {
    private DoubleSolenoid leftSolenoid = new DoubleSolenoid(TrapDoorConstants.LEFT_PORT, PneumaticsModuleType.CTREPCM, TrapDoorConstants.LEFT_FORWARD_CHANNEL, TrapDoorConstants.LEFT_REVERSE_CHANNEL);
    private DoubleSolenoid rightSolenoid = new DoubleSolenoid(TrapDoorConstants.RIGHT_PORT, PneumaticsModuleType.CTREPCM, TrapDoorConstants.RIGHT_FORWARD_CHANNEL, TrapDoorConstants.RIGHT_REVERSE_CHANNEL);
    private CANSparkMax winch = new CANSparkMax(TrapDoorConstants.WINCH_PORT, CANSparkLowLevel.MotorType.kBrushless);

    /**
     * Constructs a TrapDoorSubsystem object. 
     * Also immediately retracts the pnematics
     */
    public TrapDoorSubsystem(){
        retractPneumatics();
    }

    /** @return Command that starts winching in */
    public Command winchCommand() {
        return runOnce(() -> {
            winch.set(-1);
        });
    }

    /** @return Command that starts winching out */
    public Command unwinch() {
        return runOnce(() -> {
            winch.set(1);
        });
    }

    /** @return Command that stops the winch motor */
    public Command stopWinchMotor() {
        return runOnce(() -> winch.set(0));
    }
    
    /** @return Command that extends the pneumatics */
    public Command extendCommand() {
        return runOnce(this::extendPneumatics);
    }

    /** @return Command that retracts the pneumatics */
    public Command retractCommand() {
        return runOnce(this::retractPneumatics);
    }

    /** @return Command that stops the pneumatics */
    public Command stopCommand() {
        return runOnce(this::stopPnematics);
    }

    /** Extend the pneumatics */
    private void extendPneumatics() {
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kReverse);
    }

    /** Retract the pneumatics */
    private void retractPneumatics() {
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);
    }

    /** Stop the pneumatics */
    private void stopPnematics() {
        leftSolenoid.set(Value.kOff);
        rightSolenoid.set(Value.kOff);
    }
}