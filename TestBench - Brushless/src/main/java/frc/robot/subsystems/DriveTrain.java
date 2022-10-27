// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private Talon FL = new Talon(1);
  private Talon FR = new Talon(2);
  private Talon BL = new Talon(3);
  private Talon BR = new Talon(4);
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
  public void toggleMotor(String motorIdentifier, double motorSpeed) {
    switch (motorIdentifier) {
      case "FL":
        if (FL.get() == 0) {
          FL.set(motorSpeed);
        }
        else {
          FL.set(0);
        }
        break;
      case "BL":
        if (BL.get() == 0) {
          BL.set(motorSpeed);
        }
        else {
        BL.set(0);
        }
        break;
      case "FR":
      if (FR.get() == 0) {
          FR.set(motorSpeed);
        }
        else {
          FR.set(0);
        }
        break;
      case "BR":
        if (BR.get() == 0) {
          BR.set(motorSpeed);
        }
        else {
          BR.set(0);
        }
        break;
    }
  }
  private void setLeft(double motorPower) {
    FL.set(motorPower);
    BL.set(motorPower);
  }

  private void setRight(double motorPower) {
    FR.set(motorPower);
    BR.set(motorPower);
  }

  // throttle is the forward-back axis; rotation is the left-right axis
  public void arcadeDrive(double throttle, double tilt) {
    // maximum speed in a single direction
    double maximum = Math.max(Math.abs(throttle), Math.abs(tilt));
    double total = throttle + tilt;
    double difference = throttle - tilt;
    // moving forward
    if (throttle >= 0) {
      // turn right
      if (tilt >= 0) {
        setLeft(maximum);
        setRight(difference);
      }
      // turn left
      else {
        setLeft(total);
        setRight(maximum);
      }
    }
    // moving backward
    else {
      // turn left
      if (tilt >= 0) {
        setLeft(total);
        setRight(-maximum);
      }
      // turn right
      else {
        setLeft(-maximum);
        setRight(difference);
      }
    }

  }

  public void tankDrive(double lThrottle, double rThrottle, double speed) {
    lThrottle *= speed;
    rThrottle *= speed;

    FL.set(lThrottle);
    BL.set(lThrottle);
    FR.set(-rThrottle);
    BR.set(-rThrottle);
  }

  public void doDrive(double throttle, double twist, double speed, double tilt) {
    if (arcade) {
      arcadeDrive(throttle, tilt);
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
