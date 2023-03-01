// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import utilities.IMUWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;

/** An example command that uses an example subsystem. */
public class MoveDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_DriveTrain;
  
  PIDController drivingPID = new PIDController(Constants.EncoderPIDKP, Constants.EncoderPIDKI, Constants.EncoderPIDKD);
  PIDController gyroZPID = new PIDController(Constants.GyroZKP, Constants.GyroZKI, Constants.GyroZKD);
  //position variables are measured in encoder ticks
  private double currentPosition;
  private double targetPosition;
  private double leftThrottle;
  private double rightThrottle;
  private double wheelRadius = 3.5;
  private double zAngle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveDistance(DriveTrain train, double togo) {
    m_DriveTrain = train;

    targetPosition = togo / (2*wheelRadius * Math.PI);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    drivingPID.reset();
    drivingPID.setSetpoint(targetPosition);
    gyroZPID.reset();
    gyroZPID.setSetpoint(0);
    m_DriveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    currentPosition = m_DriveTrain.getPosition();
    zAngle = IMUWrapper.getZAngle();
    SmartDashboard.putNumber("Z angle", zAngle);
    double pidOutputZ = -gyroZPID.calculate(zAngle);
    double clampedPIDOutputZ = MathUtil.clamp(pidOutputZ, -.5, .5);
    goDistance(targetPosition, currentPosition, clampedPIDOutputZ);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentPosition > targetPosition;
  }

  public void goDistance(double targetPosition, double currentPosition, double zPIDOutput)
  {
    if(targetPosition >= currentPosition)
    {
      double motorOutput = MathUtil.clamp(drivingPID.calculate(currentPosition), -1, 1);
      leftThrottle = motorOutput;
      rightThrottle = motorOutput;
      System.out.println("current position:" + currentPosition + "target position:" + targetPosition);
      SmartDashboard.putNumber("Current position", currentPosition);
      SmartDashboard.putNumber("Target position", targetPosition);
      SmartDashboard.putNumber("PID Output", motorOutput);
      System.out.println("pid output: " + motorOutput);
      if (zPIDOutput > 0) {
        rightThrottle *= (1 - Math.abs(zPIDOutput));
      }
      else {
        leftThrottle *= (1 - Math.abs(zPIDOutput));
      }
      SmartDashboard.putNumber("Left Throttle", leftThrottle);
      SmartDashboard.putNumber("Right Throttle", rightThrottle);
      m_DriveTrain.doDrive(leftThrottle, rightThrottle);
    }
  }
}
