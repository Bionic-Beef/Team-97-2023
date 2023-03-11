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
public class MoveUntilTilted extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_DriveTrain;
  PIDController gyroZPID = new PIDController(Constants.GyroZKP, Constants.GyroZKI, Constants.GyroZKD);
  private Timer m_timer = new Timer();
  //position variables are measured in encoder ticks

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveUntilTilted(DriveTrain train) {
    m_DriveTrain = train;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    System.out.println("timer: " + m_timer.get());
    m_DriveTrain.doDrive(-.5, -.5);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      m_DriveTrain.doDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(IMUWrapper.getYAngle()) > 10;
    return m_timer.hasElapsed(2.5);
  }
}
