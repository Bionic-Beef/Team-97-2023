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
  private boolean spin = false;
  private double accelVal = 0.05;
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
  
  public void toggleSpin() {
    spin = !spin;
  }

  private void setLeft(double motorPower) {
    FL.set(motorPower);
    BL.set(motorPower);
  }
  private void setRight(double motorPower) {
    FR.set(-motorPower);
    BR.set(-motorPower);
  }

  // public double getRampedValue(double )

  public void tankDrive(double lThrottle, double rThrottle) {
    // lThrottle *= speed;
    // rThrottle *= speed;

    setLeft(lThrottle);
    setRight(-rThrottle);
  }

  public double accelerate(double currentSpeed, double targetSpeed) {
    
    if (targetSpeed > currentSpeed + accelVal) {
      currentSpeed += accelVal;
    } else if (targetSpeed < currentSpeed - accelVal) {
      currentSpeed -= accelVal;
    }

    return currentSpeed;
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
      double currentSpeed = (m_left.get() + m_right.get()) / 2;
      m_drive.curvatureDrive(accelerate(currentSpeed, lThrottle), tilt, spin);
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
