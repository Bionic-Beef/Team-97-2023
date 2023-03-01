// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chute extends SubsystemBase {
  private CANSparkMax chuteMotor = new CANSparkMax(5, MotorType.kBrushed);
  private double spinSpeed = 0.4;

  public Chute() {}

  public void spinForward() {
    chuteMotor.set(spinSpeed);
  }

  public void spinBack() {
    chuteMotor.set(-spinSpeed);
  }

  public void stopSpin() {
    chuteMotor.set(0);
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
