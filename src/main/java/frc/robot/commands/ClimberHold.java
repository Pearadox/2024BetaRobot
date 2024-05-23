// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.subsystems.Climber;

// public class ClimberHold extends Command {
//   private Climber climber = Climber.getInstance();
  
//   public ClimberHold() {
//     addRequirements(climber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(climber.getZeroing()){
//       climber.setCurrentLimit(12);
//       climber.zeroClimber();
//     }
//     else{
//       climber.setCurrentLimit(60);
//       if(climber.getClimbSequenceStep() <= -1){
//         climber.setClimberPosition(ClimberConstants.IDLE_ROT);
//       }
//       else if(climber.getClimbSequenceStep() == 0){
//         climber.setClimberPosition(ClimberConstants.MANTIS_ROT);
//       }
//       else if(climber.getClimbSequenceStep() >= 1){
//         climber.setClimberPosition(ClimberConstants.CLIMB_ROT);
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }