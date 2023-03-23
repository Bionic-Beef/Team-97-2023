// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import utilities.IMUWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonomousBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private double yAngle;
  private double zAngle;
  private Timer m_timer;
  private PIDController gyroYPID = new PIDController(Constants.GyroYKP, Constants.GyroYKI, Constants.GyroYKD);
  private PIDController gyroZPID = new PIDController(Constants.GyroZKP, Constants.GyroZKI, Constants.GyroZKD);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousBalance(DriveTrain m_driveTrain) {
    this.m_driveTrain = m_driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyroYPID.setSetpoint(0);
    gyroYPID.reset();
    gyroZPID.setSetpoint(0);
    gyroZPID.reset();

    // IMUWrapper.calibrate();
    // m_timer = new Timer();
    // m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // if (m_timer.hasElapsed(25)) {
        // find up-down (y) angle, calculate PID output
        yAngle = IMUWrapper.getYAngle();
        if (Math.abs(yAngle) > 5) {
          double pidOutputY = gyroYPID.calculate(yAngle);
          System.out.println("y pid output: " + pidOutputY);
          double clampedPIDOutputY = MathUtil.clamp(pidOutputY, -.4, .4);
          System.out.println("clamped y pid output: " + clampedPIDOutputY);
          double pOutput = gyroYPID.getPositionError() * Constants.GyroYKP;
          double dOutput = gyroYPID.getVelocityError() * Constants.GyroYKD;
          SmartDashboard.putNumber("P output", pOutput);
          SmartDashboard.putNumber("I output", (pidOutputY - pOutput - dOutput) / Constants.GyroYKI);
          SmartDashboard.putNumber("D output", dOutput);

          // find left-right (z) angle, calculate PID output
          zAngle = IMUWrapper.getZAngle();
          double pidOutputZ = -gyroZPID.calculate(zAngle);
          double clampedPIDOutputZ = MathUtil.clamp(pidOutputZ, -.5, .5);

          double leftThrottle = clampedPIDOutputY;
          double rightThrottle = clampedPIDOutputY;

          // calculate overall motor settings based on z and y pid outputs
          if (clampedPIDOutputZ < 0) {
            rightThrottle *= (1 - Math.abs(clampedPIDOutputZ));
          }
          else if (clampedPIDOutputZ > 0) {
            leftThrottle *= (1 - Math.abs(clampedPIDOutputZ));
          }
          //clockwise/right is positive for arcadeDrive, make sure pid/gyro is consistent
          SmartDashboard.putNumber("Z PID Output", clampedPIDOutputZ);
          SmartDashboard.putNumber("Y PID Output", clampedPIDOutputY);
          SmartDashboard.putNumber("Left throttle", leftThrottle);
          SmartDashboard.putNumber("Right throttle", rightThrottle);
          SmartDashboard.putNumber("Y angle", yAngle);
          SmartDashboard.putNumber("Z angle", zAngle);
          //m_driveTrain.doDrive(leftThrottle, rightThrottle);
        } else {
          m_driveTrain.doDrive(0, 0);
        }     
    }
    // else {
    //   System.out.println("waiting to drive. time elapsed: " + m_timer.get());
    //   m_driveTrain.doDrive(0, 0);
    // }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
