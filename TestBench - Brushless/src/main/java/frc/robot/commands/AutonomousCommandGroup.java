// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonomousCommandGroup extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private int stage = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCommandGroup(DriveTrain train) {
    m_driveTrain = train;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(stage == 1)
    {
      if(m_driveTrain.getPosition() < 10)
      {
        new MoveDistance(m_driveTrain, 10);
      }
      else
      {
        stage++;
      }
    }
    else if(stage == 2)
    {
      //unloads game piece
      stage++;
    }
    else if(stage == 3)
    {
      if(m_driveTrain.getPosition() > -10)
      {
        new MoveDistance(m_driveTrain, -10);
      }
      else{
        stage++;
      }
    }
    else if (stage == 4)
    {
      //balances on seesaw
      stage++;
    }
    new MoveDistance(m_driveTrain, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 5;
  }
}
