// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class MoveLeft extends SequentialCommandGroup {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //steps: score point, go over chargestation and leave starting zone, come back on the charge station, and self balance
  public MoveLeft(DriveTrain train) {
    addCommands(
        new TankDriveConstantSpeed(train, 0, .3, .5),
        new TankDriveConstantSpeed(train, .5, .5, .5),
        new TankDriveConstantSpeed(train, 0, -.3, .5),
        new TankDriveConstantSpeed(train, -.5, -.5, .5)

    );
    
    // Requirements are implicitly added
  }
}
