// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax FL = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax BL = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax FR = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax BR = new CANSparkMax(3, MotorType.kBrushless);
  private boolean arcade = true;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  public void switchMode() {
    arcade = !arcade;
  }

  public void arcadeDrive(double throttle, double rotation, double speed) {
    double LMtrPower = (rotation - throttle) * speed;
    double RMtrPower = (rotation + throttle) * speed;

    FL.set(LMtrPower);
    BL.set(LMtrPower);
    FR.set(RMtrPower);
    BR.set(RMtrPower);
  }

  public void tankDrive(double lThrottle, double rThrottle, double speed) {
    lThrottle *= speed;
    rThrottle *= speed;

    FL.set(lThrottle);
    BL.set(lThrottle);
    FR.set(-rThrottle);
    BR.set(-rThrottle);
  }

  public void doDrive(double throttle, double twist, double speed) {
    if (arcade) {
      arcadeDrive(throttle, twist, speed);
    } else {
      tankDrive(throttle, twist, speed);
    }
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
