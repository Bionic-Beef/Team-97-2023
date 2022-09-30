// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DriveTrain extends SubsystemBase {
  private TalonSRX FL = new TalonSRX(1);
  private TalonSRX BL = new TalonSRX(3);
  private TalonSRX FR = new TalonSRX(0);
  private TalonSRX BR = new TalonSRX(2);
  private Joystick stick1 = new Joystick(0);
  private Joystick stick2 = new Joystick(1);
  private boolean arcade = true;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  public void switchMode() {
    arcade = !arcade;
  }

  public void arcadeDrive(double throttle, double rotation, double speed) {
    double MtrPower = (throttle + rotation) * speed;

    FL.set(ControlMode.PercentOutput, MtrPower);
    BL.set(ControlMode.PercentOutput, MtrPower);
    FR.set(ControlMode.PercentOutput, MtrPower);
    BR.set(ControlMode.PercentOutput, MtrPower);
  }

  public void tankDrive(double lThrottle, double rThrottle, double speed) {
    lThrottle *= speed;
    rThrottle *= speed;

    FL.set(ControlMode.PercentOutput, lThrottle);
    BL.set(ControlMode.PercentOutput, lThrottle);
    FR.set(ControlMode.PercentOutput, -rThrottle);
    BR.set(ControlMode.PercentOutput, -rThrottle);
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
