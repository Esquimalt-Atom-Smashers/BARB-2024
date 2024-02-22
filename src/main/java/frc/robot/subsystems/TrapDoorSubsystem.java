package frc.robot.subsystems;

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

  /**
   * Constructs a TrapDoorSubsystem object. 
   * Initializes the double solenoid that opens the trap door and lets us score into the amp.
   */
  public TrapDoorSubsystem(){
    leftSolenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, Constants.TrapDoorConstants.FORWARD_PORT, Constants.TrapDoorConstants.REVERSE_PORT);
    rightSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);
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
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
  }

  /** Retract the pnematics */
  private void retract() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  private void stop() {
    leftSolenoid.set(Value.kOff);
    rightSolenoid.set(Value.kOff);
  }
}