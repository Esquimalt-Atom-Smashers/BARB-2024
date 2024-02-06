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
  private DoubleSolenoid solenoid;

  /**
   * Constructs a TrapDoorSubsystem object. 
   * Initializes the double solenoid that opens the trap door and lets us score into the amp.
   */
  public TrapDoorSubsystem(){
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.TrapDoorConstants.FORWARD_PORT, Constants.TrapDoorConstants.REVERSE_PORT);
  }

  /** @return Command that extends the pneumatics */
  public Command extendCommand() {
    return runOnce(this::extend);
  }

  /** @return Command that retracts the pneumatics */
  public Command retractCommand() {
    return runOnce(this::retract);
  }

  /** Extend the pneumatics */
  private void extend() {
    solenoid.set(Value.kForward);
  }

  /** Retract the pnematics */
  private void retract() {
    solenoid.set(Value.kReverse);
  }
}