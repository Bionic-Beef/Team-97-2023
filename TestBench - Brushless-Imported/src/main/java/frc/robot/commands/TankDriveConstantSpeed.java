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
import java.lang.Math;

/** An example command that uses an example subsystem. */
public class TankDriveConstantSpeed extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_DriveTrain;
  private double leftThrottle;
  private double rightThrottle;
  private final Timer m_timer = new Timer();
  private double timeToRun;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDriveConstantSpeed(DriveTrain train, double leftMotor, double rightMotor, double time) {
    m_DriveTrain = train;
    leftThrottle = leftMotor;
    rightThrottle = rightMotor;
    timeToRun = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
      m_DriveTrain.doDrive(leftThrottle, rightThrottle);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_DriveTrain.doDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(timeToRun);
  }
}
