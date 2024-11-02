// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSim extends SubsystemBase {
  /** Creates a new DriveSim. */
  public DriveSim() {
//     this.drive = new Drive(
//         new GyroIOSim(this.gyroSimulation),
//         new ModuleIOSim(this.swerveDriveSimulation.getModules()[0]),
//         new ModuleIOSim(this.swerveDriveSimulation.getModules()[1]),
//         new ModuleIOSim(this.swerveDriveSimulation.getModules()[2]),
//         new ModuleIOSim(this.swerveDriveSimulation.getModules()[3])
// );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
