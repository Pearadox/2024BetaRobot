// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AmpAutoAlign extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  /** Creates a new AmpAutoAlignn. */
  public AmpAutoAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = new Pose2d(1.5, 7.5, Rotation2d.fromDegrees(90)); // todo: find amp pose
  
    PathConstraints constraints = new PathConstraints(
        3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
        
    AutoBuilder.pathfindToPose(targetPose, constraints, 0, 0);
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
