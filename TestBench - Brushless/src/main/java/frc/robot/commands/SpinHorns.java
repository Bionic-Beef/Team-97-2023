// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Horns;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SpinHorns extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Horns m_horns;
  private double motorSpeed;

  //position variables are measured in encoder ticks

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SpinHorns(Horns horns, double motorSpeed) {
    m_horns = horns;
    this.motorSpeed = motorSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(horns);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    System.out.println("motor speed: " + motorSpeed);
    m_horns.spinForward(motorSpeed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_horns.stopSpin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
