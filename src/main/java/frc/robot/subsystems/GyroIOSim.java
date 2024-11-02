// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.GyroIO;

// Simulation implementation
public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }
    
    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return this.gyroSimulation.getGyroReading();
    }
    
    @Override // specified by GroIOSim interface
    public double getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocityRadPerSec();
    }
}