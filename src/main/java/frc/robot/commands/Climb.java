// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.climberState;

public class Climb extends Command {
  /** Creates a new Climb. */
  private static Climber climber = Climber.getInstance();
  private static climberState state;
  public Climb() {
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    state = climber.getState();
    if(state == climberState.Climb) {
      climber.setClimbReference(ClimberConstants.CLIMBED_POSE);
    }
    else if(state == climberState.Down) {
      climber.setClimbReference(0);
    }
    else {
      climber.setClimbReference(0);
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
