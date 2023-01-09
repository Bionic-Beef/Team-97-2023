// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax FL = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax FR = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax BL = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax BR = new CANSparkMax(4, MotorType.kBrushless);
  private boolean arcade = true;
  private boolean isTurnSpeedSlow = false;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  public void switchMode() {
    arcade = !arcade;
  }

  public void toggleTurnSpeed() {
    isTurnSpeedSlow = !isTurnSpeedSlow;
  }
  
  private void setLeft(double motorPower) {
    FL.set(motorPower);
    BL.set(motorPower);
  }

  private void setRight(double motorPower) {
    FR.set(-motorPower);
    BR.set(-motorPower);
  }

  // throttle is the forward-back axis; rotation is the left-right axis
  public void arcadeDrive(double throttle, double tilt) {
    // maximum speed in a single direction
    if (isTurnSpeedSlow) {
      tilt /= 2;
    }
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

  public void doDrive(double throttle, double tilt) {
    // account for accidental movement
    if (Math.abs(throttle) < 0.11) {
      throttle = 0;
    }
    if (Math.abs(tilt) < 0.11) {
      tilt = 0;
    }
    System.out.println(String.format("I am arcade driving with a throttle of %s and a tilt of %s", throttle, tilt));
    arcadeDrive(throttle, tilt);
    // System.out.println("y axis: " + throttle);
    // System.out.println("x axis: " + tilt);
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
