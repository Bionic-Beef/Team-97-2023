// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.ExampleSubsystem;
// import edu.wpi.first.math.Drake;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.lang.Math;


// /** An example command that uses an example subsystem. */
// public class MoveDistance extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final DriveTrain m_DriveTrain;
  
//   PIDController pid = new PIDController(.5, 0, .5);
//   //position variables are measured in encoder ticks
//   private double currentPosition;
//   private double targetPosition;
//   private double wheelRadius;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public MoveDistance(DriveTrain train, double togo) {
//     m_DriveTrain = train;
//     targetPosition = togo / (wheelRadius * Math.PI);
    
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_DriveTrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize()
//   {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute()
//   {
//     currentPosition = m_DriveTrain.getPosition();
//     goDistance(targetPosition, currentPosition);
      
//   }
//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return currentPosition > targetPosition;
//   }

//   public void goDistance(double targetPosition, double currentPosition)
//   {
//     if(targetPosition <= currentPosition)
//     {
//       m_DriveTrain.doDrive(pid.calculate(currentPosition, targetPosition), pid.calculate(currentPosition, targetPosition));
//     }
//   }
// }
