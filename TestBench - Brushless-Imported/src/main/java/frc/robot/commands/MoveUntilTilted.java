// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import utilities.IMUWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveUntilTilted extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_DriveTrain;
  PIDController gyroZPID = new PIDController(Constants.GyroZKP, Constants.GyroZKI, Constants.GyroZKD);
  private double targetAngle;
  private double currentAngle;
  private double driveSpeed;
  private boolean goingToAHigherAngle;

  //position variables are measured in encoder ticks

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveUntilTilted(DriveTrain train, double angle, double driveSpeed, boolean goingToAHigherAngle) {
    m_DriveTrain = train;
    targetAngle = angle;
    this.driveSpeed = driveSpeed;
    this.goingToAHigherAngle = goingToAHigherAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    gyroZPID.reset();
    gyroZPID.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    currentAngle = IMUWrapper.getYAngle();

    // double rightThrottle = driveSpeed;
    // double leftThrottle = driveSpeed;
    // double zAngle = IMUWrapper.getZAngle();
    // double pidOutputZ = -MathUtil.clamp(gyroZPID.calculate(zAngle), -5, .5);
    // if (pidOutputZ > 0) {
    //   rightThrottle *= (1 - Math.abs(pidOutputZ));
    // }
    // else {
    //   leftThrottle *= (1 - Math.abs(pidOutputZ));
    // }
    // if (targetAngle < 0) {
    //   m_DriveTrain.doDrive(-leftThrottle, -rightThrottle);
    // } else {
    //   m_DriveTrain.doDrive(leftThrottle, rightThrottle);
    // }

    if (targetAngle < 0) {
      m_DriveTrain.doDrive(-driveSpeed, -driveSpeed);
    } else {
      m_DriveTrain.doDrive(driveSpeed, driveSpeed);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("move until tilted finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (goingToAHigherAngle) {
      return currentAngle >= targetAngle;
    }
    //going to a lower angle: check that the current angle is less
    else {
      return currentAngle <= targetAngle;
    }
  }
}
