// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ToggleMotor;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_dDriveTrain = new DriveTrain();
  private final Joystick joystick1 = new Joystick(0);
  final JoystickButton l1 = new JoystickButton(joystick1, 7);
  final JoystickButton l2 = new JoystickButton(joystick1, 8);
  final JoystickButton r1 = new JoystickButton(joystick1, 9);
  final JoystickButton r2 = new JoystickButton(joystick1, 10);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_dDriveTrain.setDefaultCommand(
      new RunCommand(
        () -> m_dDriveTrain.doDrive(
          joystick1.getRawAxis(1), 
          joystick1.getTwist(), 
          Math.max(0, 1-joystick1.getRawAxis(3)),
          joystick1.getRawAxis(2)
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
    l1.whenPressed(new ToggleMotor(m_dDriveTrain, "BL"));
    l2.whenPressed(new ToggleMotor(m_dDriveTrain, "FL"));
    r1.whenPressed(new ToggleMotor(m_dDriveTrain, "BR"));
    r2.whenPressed(new ToggleMotor(m_dDriveTrain, "FR"));

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
