// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
  //forward channel is 1; reverse is 2
  private final DoubleSolenoid mySolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  /** Creates a new ExampleSubsystem. */
  public PneumaticsSubsystem() {
    mySolenoid.set(kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void toggleSolenoid() {
    //toggle switches from reverse to forward/vice versa
    mySolenoid.toggle();
  }
  public void setSolenoidForward() {
    mySolenoid.set(kForward);
  }
  public void turnSolenoidOff() {
    mySolenoid.set(kOff);
  }
}
