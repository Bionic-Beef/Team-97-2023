// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private Victor FL = new Victor(1);
  private Victor BL = new Victor(2);
  private Victor FR = new Victor(4);
  private Victor BR = new Victor(3);
  // private CANSparkMax FL = new CANSparkMax(1, MotorType.kBrushless);
  // private CANSparkMax FR = new CANSparkMax(2, MotorType.kBrushless);
  // private CANSparkMax BL = new CANSparkMax(3, MotorType.kBrushless);
  // private CANSparkMax BR = new CANSparkMax(4, MotorType.kBrushless);
  private boolean arcade = true;
  private boolean isTurnSpeedSlow = false;
  MotorControllerGroup m_left = new MotorControllerGroup(FL, BL);
  MotorControllerGroup m_right = new MotorControllerGroup(FR, BR);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  // m_left.setInverted(true);
  // m_right.setInverted(true);


  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  public void switchMode() {
    arcade = !arcade;
  }
  public void setFL() {
    if (FL.get() != 0) {
      FL.set(.3);
    }
    else {
      FL.set(0);
    }
  }
  public void setFR() {
    if (FR.get() != 0) {
      FR.set(.3);
    }
    else {
      FR.set(0);
    }
  }
  public void setBL() {
    if (BL.get() != 0) {
      BL.set(.3);
    }
    else {
      BL.set(0);
    }
  }
  public void setBR() {
    if (BR.get() != 0) {
      BR.set(.3);
    }
    else {
      BR.set(0);
    }
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

  // public double getRampedValue(double )

  public void tankDrive(double lThrottle, double rThrottle) {
    // lThrottle *= speed;
    // rThrottle *= speed;

    setLeft(lThrottle);
    setRight(-rThrottle);
  }

  public void doDrive(double lThrottle, double tilt, double rThrottle) {
    // account for accidental movement
    // if (Math.abs(lThrottle) < 0.18) {
    //   lThrottle = 0;
    // }
    // if (Math.abs(tilt) < 0.18) {
    //   tilt = 0;
    // }
    // if (Math.abs(rThrottle) < 0.18) {
    //   rThrottle = 0;
    // }
    if (arcade) {     
      System.out.println(String.format("I am arcade driving with a throttle of %s and a tilt of %s", lThrottle, tilt));
      // m_drive.arcadeDrive(lThrottle, tilt);
      m_drive.curvatureDrive(lThrottle, tilt, false);
    }
    else {
      m_drive.tankDrive(lThrottle, rThrottle);
      System.out.println(String.format("I am tank driving with a lThrottle of %s and a rThrottle of %s", lThrottle, rThrottle));
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
