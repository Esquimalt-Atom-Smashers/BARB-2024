package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * Subsystem representing the pneumatics on the shooter.
 */
public class TrapDoorSubsystem extends SubsystemBase {
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;
  private CANSparkMax winch;

  /**
   * Constructs a TrapDoorSubsystem object. 
   * Initializes the double solenoid that opens the trap door and lets us score into the amp.
   */
  public TrapDoorSubsystem(){
    leftSolenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, Constants.TrapDoorConstants.FORWARD_PORT, Constants.TrapDoorConstants.REVERSE_PORT);
    rightSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 1, 0);
    winch = new CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushless);
    retract();
  }


  public Command winchCommand() {
    return runOnce(() -> {
        winch.set(-20);
    });
  }

  public Command unwinch() {
    return runOnce(() -> {
      winch.set(-0.80);
    });
  }

  public Command stopWinchMotor() {
    return runOnce(() -> winch.set(0));
  }

  
  /** @return Command that extends the pneumatics */
  public Command extendCommand() {
    return runOnce(this::extend);
  }

  /** @return Command that retracts the pneumatics */
  public Command retractCommand() {
    return runOnce(this::retract);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  /** Extend the pneumatics */
  private void extend() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  /** Retract the pnematics */
  private void retract() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  private void stop() {
    leftSolenoid.set(Value.kOff);
    rightSolenoid.set(Value.kOff);
  }
}