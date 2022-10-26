// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

public class DriveTrain extends SubsystemBase {
  private Victor FL = new Victor(1);
  private Victor BL = new Victor(2);
  private Victor FR = new Victor(4);
  private Victor BR = new Victor(3);
  private boolean arcade = true;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  public void switchMode() {
    arcade = !arcade;
  }

  public void test_FL() {
    FL.set(0.5);
  }

  public void test_FR() {
    FR.set(0.5);
  }

  public void test_BL() {
    BL.set(0.5);
  }

  public void test_BR() {
    BR.set(0.5);
  }

  public void stop_test() {
    FL.set(0);
    BL.set(0);
    FR.set(0);
    BR.set(0);
  }

  public void arcadeDrive(double throttle, double rotation, double speed) {
    double LMtrPower = (rotation + throttle) * speed;
    double RMtrPower = (rotation - throttle) * speed;

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
