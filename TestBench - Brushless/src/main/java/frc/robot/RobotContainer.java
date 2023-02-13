// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.Position;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutonomousCommandGroup;
import frc.robot.commands.MoveDistance;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  // private final Joystick joystick1 = new Joystick(0);
  private final XboxController joystick1 = new XboxController(0);
  // x button
  // private JoystickButton toggleTurningSpeed = new JoystickButton(joystick1, 2);
  // // a button
  // private JoystickButton toggleArcadeDrive = new JoystickButton(joystick1, 1);
  private JoystickButton toggleSpin = new JoystickButton(joystick1, 5);
  //a
  // private JoystickButton move3FT = new JoystickButton(joystick1, 6);

  private JoystickButton toggleFL = new JoystickButton(joystick1, 4);
  private JoystickButton toggleBL = new JoystickButton(joystick1, 3);
  private JoystickButton toggleFR = new JoystickButton(joystick1, 2);
  private JoystickButton toggleBR = new JoystickButton(joystick1, 1);

  private final SlewRateLimiter filter = new SlewRateLimiter(2);

  // private int stage;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_dDriveTrain.setDefaultCommand(

      new RunCommand(
        () -> 
        m_dDriveTrain.doDrive(
          joystick1.getLeftY(),
          joystick1.getRightY()
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
    // toggleTurningSpeed.whenPressed(new InstantCommand(() -> {
    //   // m_dDriveTrain.toggleTurnSpeed();
    // }));
    // toggleArcadeDrive.whenPressed(new InstantCommand(() -> {
      // m_dDriveTrain.switchMode();
    // }));
      //move3FT.whenPressed(new MoveDistance(m_dDriveTrain, 12));
    toggleSpin.whenPressed(new InstantCommand(() -> {
      m_dDriveTrain.toggleSpin();
    }));
    toggleBL.whenPressed(new InstantCommand(() -> {
      m_dDriveTrain.setBL();
    }));
    toggleBR.whenPressed(new InstantCommand(() -> {
      m_dDriveTrain.setBR();
    }));
    toggleFL.whenPressed(new InstantCommand(() -> {
      m_dDriveTrain.setFL();
    }));
    toggleFR.whenPressed(new InstantCommand(() -> {
      m_dDriveTrain.setFR();
    }));
    

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new AutonomousCommandGroup(m_dDriveTrain);
    return new MoveDistance(m_dDriveTrain, 100);
  }
}
