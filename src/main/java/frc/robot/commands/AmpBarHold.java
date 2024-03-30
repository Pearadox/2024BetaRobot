// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.AmpBar.AmpBarMode;

public class AmpBarHold extends Command {
  AmpBar ampBar = AmpBar.getInstance();
  Climber climber = Climber.getInstance();

  /** Creates a new AmpBarHold. */
  public AmpBarHold() {
    addRequirements(ampBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampBar.ampBarHold();

    if(climber.getClimbSequenceStep() == -1){
      if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95 && ampBar.getAmpBarMode() == AmpBarMode.Stowed){
        ampBar.setDeployedMode();
      }
      else if (RobotContainer.driverController.getLeftTriggerAxis() < 0.95 && (ampBar.getAmpBarMode() == AmpBarMode.Deployed || ampBar.getAmpBarMode() == AmpBarMode.Climb)){
        ampBar.setStowedMode();
      }
    }
    else if(climber.getClimbSequenceStep() >= 0 && climber.getClimbSequenceStep() <= 1){
      ampBar.setClimbMode();
    }
    else if(climber.getClimbSequenceStep() >= 2){
      ampBar.setTrapMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
