// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax FL = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax FR = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax BL = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax BR = new CANSparkMax(4, MotorType.kBrushless);
  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
