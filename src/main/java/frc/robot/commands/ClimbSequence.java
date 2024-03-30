// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ClimbSequence extends Command {
  private Climber climber = Climber.getInstance();
  private AmpBar ampBar = AmpBar.getInstance();
  private Shooter shooter = Shooter.getInstance();

  public ClimbSequence() {
    addRequirements(climber);
    addRequirements(ampBar);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.getClimbMode() == Climber.ClimbMode.Climbing){
      climber.setClimbMode();
    }else if(climber.getClimbMode() == Climber.ClimbMode.Climb && ampBar.getAmpBarMode() != AmpBar.AmpBarMode.Trap){
      ampBar.setTrapMode();
      shooter.setTrapMode();
    }else{
      CommandScheduler.getInstance().schedule(new Shoot());
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
