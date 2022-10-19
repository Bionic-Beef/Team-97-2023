// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Test_FL;
import frc.robot.commands.Test_FR;
import frc.robot.commands.Test_BL;
import frc.robot.commands.Test_BR;
import frc.robot.commands.stop_test;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_dDriveTrain = new DriveTrain();
  private final Test_FL test_FL = new Test_FL(m_dDriveTrain);
  private final Test_FR test_FR = new Test_FR(m_dDriveTrain);
  private final Test_BL test_BL = new Test_BL(m_dDriveTrain);
  private final Test_BR test_BR = new Test_BR(m_dDriveTrain);
  private final stop_test stop_test = new stop_test(m_dDriveTrain);
  private final Joystick joystick1 = new Joystick(0);

  final JoystickButton FL_test = new JoystickButton(joystick1, 7);
  final JoystickButton FR_test = new JoystickButton(joystick1, 8);
  final JoystickButton BL_test = new JoystickButton(joystick1, 10);
  final JoystickButton BR_test = new JoystickButton(joystick1, 9);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_dDriveTrain.setDefaultCommand(
      new RunCommand(
        () -> m_dDriveTrain.doDrive(
          joystick1.getRawAxis(1), 
          joystick1.getTwist(), 
          Math.max(0, 1-joystick1.getRawAxis(3))
          ),
      m_dDriveTrain)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    FL_test.toggleWhenPressed(test_FL);
    FR_test.toggleWhenPressed(test_FR);
    BL_test.toggleWhenPressed(test_BL);
    BR_test.toggleWhenPressed(test_BR);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
