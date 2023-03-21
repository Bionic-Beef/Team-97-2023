// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class AutonomousCommandGroup extends SequentialCommandGroup {
  boolean isBalancing = true;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //steps: score point, go over chargestation and leave starting zone, come back on the charge station, and self balance
  public AutonomousCommandGroup(DriveTrain train, Chute chute) {
    if (isBalancing) {
      addCommands(
        new RotateChuteDoorAutonomous(chute),
        new MoveDistance(train, Constants.distanceToLeaveCommunityFromStart),
        // new MoveDistanceConstantSpeed(train, Constants.distanceToLeaveCommunityFromStart),
        // new MoveDistanceConstantSpeed(train, Constants.distanceToChargeStationFromOutsideCommunity),
        // new MoveDistanceConstantSpeed(train, Constants.distanceToChargeStationFromStart),
        new AutonomousBalance(train)
      );
    } else {
      addCommands(
        new RotateChuteDoorAutonomous(chute),
        new MoveDistanceConstantSpeed(train, Constants.distanceToLeaveCommunityFromStart)
      );
    }
    
    // Requirements are implicitly added
  }
}
