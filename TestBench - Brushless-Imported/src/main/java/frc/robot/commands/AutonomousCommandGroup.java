// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
      // addCommands(
      //   new RotateChuteDoorAutonomous(chute),
      //   new MoveDistanceConstantSpeed(train, Constants.distanceToLeaveCommunityFromStart, Constants.autoDrivingSpeedPhaseOne),
      //   new MoveDistanceConstantSpeed(train, Constants.distanceToChargeStationFromOutsideCommunity, Constants.autoDrivingSpeedPhaseTwo),
      //   new AutonomousBalance(train)
      // );
      // addCommands(
      //   new RotateChuteDoorAutonomous(chute),
      //   new MoveDistanceConstantSpeed(train, Constants.autoTargetDistancePhaseOne, Constants.autoDrivingSpeedPhaseOne),
      //   new MoveDistanceConstantSpeed(train, Constants.autoTargetDistancePhaseTwo, Constants.autoDrivingSpeedPhaseTwo),
      //   new AutonomousBalance(train)
      // );
      // addCommands(
      //   new RotateChuteDoorAutonomous(chute),
      //   new MoveUntilTilted(train, Constants.autoTargetAnglePhaseOne, Constants.autoDrivingSpeedPhaseOne, true),
      //   new MoveUntilTilted(train, Constants.autoTargetAnglePhaseTwo, Constants.autoDrivingSpeedPhaseTwo, false),
      //   new AutonomousBalance(train)
      // );
      addCommands(
        new MoveDistanceConstantSpeed(train, Constants.autoTargetDistancePhaseOne, Constants.autoDrivingSpeedPhaseOne),
        new MoveDistanceConstantSpeed(train, -Constants.autoTargetDistancePhaseOne, Constants.autoDrivingSpeedPhaseOne)

      );
    } else {
      addCommands(
        new RotateChuteDoorAutonomous(chute),
        new MoveDistanceConstantSpeed(train, Constants.distanceToLeaveCommunityFromStart, .7)
      );
      
    }
    
    // Requirements are implicitly added
  }
}
