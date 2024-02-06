// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapDoorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  
  private DoubleSolenoid leftPneumatic;
  private DoubleSolenoid rightPneumatic;

  public TrapDoorSubsystem(){
    leftPneumatic = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    rightPneumatic = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  }
  /* 
   * @return a command
  */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
//hi! -jake
  public Command extendCommand() {
    return runOnce(() -> {
      rightPneumatic.set(Value.kForward);
      leftPneumatic.set(Value.kForward);    
    });
  }

  public Command retractCommand() {
    return runOnce(() -> {
      rightPneumatic.set(Value.kReverse);
      leftPneumatic.set(Value.kReverse);
    });
  }



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
