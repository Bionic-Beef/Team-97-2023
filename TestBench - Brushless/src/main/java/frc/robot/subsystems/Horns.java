// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Horns extends SubsystemBase {
  private CANSparkMax hornsMotor = new CANSparkMax(8, MotorType.kBrushless);

  public Horns() {}

  public void spinForward(double motorSpeed) {
    hornsMotor.set(motorSpeed);
  }

  public void spinBack(double motorSpeed) {
    hornsMotor.set(-motorSpeed);
  }

  public void stopSpin() {
    hornsMotor.set(0);
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
