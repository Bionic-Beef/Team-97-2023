// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chute;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class RotateChuteDoorAutonomous extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chute m_chute;
  private final Timer m_timer;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateChuteDoorAutonomous(Chute c) {
    m_chute = c;
    m_timer = new Timer();


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chute);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chute.spinForward();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chute.stopSpin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(Constants.timeToRotateChuteInAuto);
  }
}
