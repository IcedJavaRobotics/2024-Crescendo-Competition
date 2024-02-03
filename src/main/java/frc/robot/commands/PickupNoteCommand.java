// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem;


// public class PickupNoteCommand extends Command {

//   private final IntakeSubsystem intakeSubsystem;

//   /** Creates a new PickupNoteCommand. */
//   public PickupNoteCommand(IntakeSubsystem subsystem) {

//     intakeSubsystem = subsystem;
//     addRequirements(intakeSubsystem);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     intakeSubsystem.moveIntakeDown();
//     intakeSubsystem.turnRollerIn();
    
//     // So the intake waits for a piece
//     if (intakeSubsystem.turnRollerIn()) {
//       intakeSubsystem.moveIntakeUp(); 
//     }
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     intakeSubsystem.stopRollerMotor();
//     intakeSubsystem.moveIntakeUp();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
