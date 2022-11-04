// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SetSolenoidForward;
import frc.robot.commands.ToggleSolenoid;
import frc.robot.commands.TurnSolenoidOff;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();

  private final ToggleSolenoid m_autoCommand = new ToggleSolenoid(m_pneumaticsSubsystem);
  private final Joystick m_joystick1 = new Joystick(0);

  private final JoystickButton toggleSolenoidButton = new JoystickButton(m_joystick1, 9);
  private final JoystickButton setSolenoidForwardButton = new JoystickButton(m_joystick1, 8);
  private final JoystickButton turnSolenoidOffButton = new JoystickButton(m_joystick1, 7);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    toggleSolenoidButton.whenPressed(new ToggleSolenoid(m_pneumaticsSubsystem));
    setSolenoidForwardButton.whenPressed(new SetSolenoidForward(m_pneumaticsSubsystem));
    turnSolenoidOffButton.whenPressed(new TurnSolenoidOff(m_pneumaticsSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
