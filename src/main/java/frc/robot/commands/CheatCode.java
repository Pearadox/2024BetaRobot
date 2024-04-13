// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class CheatCode extends Command {

  private Shooter shooter = Shooter.getInstance(); 
  private Transport transport = Transport.getInstance();

  /** Creates a new CheatCode. */
  public CheatCode() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.cheat();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (hasNote()){
    //   shooter.cheat();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.detention();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
