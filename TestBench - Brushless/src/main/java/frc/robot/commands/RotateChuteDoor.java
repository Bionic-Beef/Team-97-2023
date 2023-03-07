// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chute;

/** An example command that uses an example subsystem. */
public class RotateChuteDoor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chute m_chute;
  private final boolean directionToRotate;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  //the boolean d changes the direction of the spin; positive is forward
  public RotateChuteDoor(Chute c, boolean d) {
    m_chute = c;
    directionToRotate = d;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chute);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(directionToRotate) {
      m_chute.spinForward();
    }
    else {
      m_chute.spinBack();
    }
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
    return true;
  }
}
