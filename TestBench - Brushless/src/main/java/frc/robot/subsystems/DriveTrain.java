// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.fasterxml.jackson.databind.introspect.ClassIntrospector.MixInResolver;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //private Victor FL = new Victor(4);
  //private Victor BL = new Victor(3);
  //private Victor FR = new Victor(2);
  //private Victor BR = new Victor(1);

  private CANSparkMax FL = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax BL = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax FR = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax BR = new CANSparkMax(1, MotorType.kBrushless);
  private RelativeEncoder lEncoder = FL.getEncoder();
  private RelativeEncoder rEncoder = FR.getEncoder();
  private boolean arcade = true;
  private boolean spin = false;
  private double accelVal = 0.05;
  private MotorControllerGroup m_left = new MotorControllerGroup(FL, BL);
  private MotorControllerGroup m_right = new MotorControllerGroup(FR, BR);
  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  private RelativeEncoder myEncoder;

  private double ftTBMove = 0;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // m_right.setInverted(true);
    // m_left.setInverted(true);
    
    resetEncoders();
  }

  public void resetEncoders()
  {
    lEncoder.setPosition(0);
    rEncoder.setPosition(0);
  }

  public void switchMode() {
    arcade = !arcade;
  }
  public void setFL() {
    System.out.println("front left motor toggled");
    if (FL.get() > 0.2) {
      FL.set(0);
    }
    else {
      FL.set(0.3);
    }
  }
  public void setFR() {
    System.out.println("front right motor toggled");
    if (FR.get() > 0.2) {
      FR.set(0);
    }
    else {
      FR.set(0.3);
    }
  }
  public void setBL() {
    System.out.println("back left motor toggled");
    if (BL.get() > 0.2) {
      BL.set(0);
    }
    else {
      BL.set(0.3);
    }
  }
  public void setBR() {
    System.out.println("back right motor toggled");
    if (BR.get() > 0.2) {
      BR.set(0);
    }
    else {
      BR.set(0.3);
    }
  }

  public void toggleSpin() {
    spin = !spin;
  }

  public double getPosition()
  {
    return (-lEncoder.getPosition() + rEncoder.getPosition()) / 2 / 12.75;
  }

  public void doDrive(double lThrottle, double rThrottle) {
      m_drive.tankDrive(-lThrottle, rThrottle);
      System.out.println("Positions: " + lEncoder.getPosition()+ ", " + -rEncoder.getPosition());
      // System.out.println(String.format("I am tank driving with a lThrottle of %s and a rThrottle of %s", lThrottle, rThrottle));
  }
 
  @Override
  public void periodic() {
    

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
