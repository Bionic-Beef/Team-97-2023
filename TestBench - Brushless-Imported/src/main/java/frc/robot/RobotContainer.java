// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutonomousCommandGroup;
import frc.robot.commands.MoveLeft;
import frc.robot.commands.RotateChuteDoor;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.DriveTrain;
import utilities.IMUWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.cameraserver.CameraServer;


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
  private final Chute m_chute = new Chute();

  // private JoystickButton toggleFL = new JoystickButton(joystick1, 4);
  // private JoystickButton toggleBL = new JoystickButton(joystick1, 3);
  // private JoystickButton toggleFR = new JoystickButton(joystick1, 2);
  // private JoystickButton toggleBR = new JoystickButton(joystick1, 1);
  private JoystickButton toggleChuteHold = new JoystickButton(joystick1, 5);
  private JoystickButton toggleChuteHoldBackwards = new JoystickButton(joystick1, 6);
  private JoystickButton turnLeftButton = new JoystickButton(joystick1, 4);
  private JoystickButton upAccel = new JoystickButton(joystick1, 8);
  private JoystickButton downAccel = new JoystickButton(joystick1, 7);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    IMUWrapper.calibrate();

    CameraServer.startAutomaticCapture();

    m_dDriveTrain.setDefaultCommand(

      new RunCommand(
        () -> 
        m_dDriveTrain.doDrive(
          joystick1.getLeftY(),
          joystick1.getRightY(),
          true
          ),
      m_dDriveTrain)
    );
  }

  public void setModeToBrake() {
    m_dDriveTrain.setMotorsToBrake();
  }
  public void setModeToCoast() {
    m_dDriveTrain.setMotorsToCoast();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // toggleBL.whenPressed(new InstantCommand(() -> {
    //   m_dDriveTrain.setBL();
    // }));
    // toggleBR.whenPressed(new InstantCommand(() -> {
    //   m_dDriveTrain.setBR();
    // }));
    // toggleFL.whenPressed(new InstantCommand(() -> {
    //   m_dDriveTrain.setFL();
    // }));
    // toggleFR.whenPressed(new InstantCommand(() -> {
    //   m_dDriveTrain.setFR();
    // }));
    // moveSetDistanceForward.whenPressed(new MoveDistanceConstantSpeed(m_dDriveTrain, 5));
    turnLeftButton.whenPressed(new MoveLeft(m_dDriveTrain));
    toggleChuteHold.whileHeld(new RotateChuteDoor(m_chute, true));
    toggleChuteHoldBackwards.whileHeld(new RotateChuteDoor(m_chute, false));
    upAccel.whenPressed(new InstantCommand(() -> {
      m_dDriveTrain.upFactor();
    }));
    downAccel.whenPressed(new InstantCommand(() -> {
      m_dDriveTrain.downFactor();
    }));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //isBalancing should be true when we are the designated auto-balance team
    return new AutonomousCommandGroup(m_dDriveTrain, m_chute);
  }
}
