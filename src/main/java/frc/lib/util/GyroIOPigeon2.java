// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(20);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2(boolean phoenixDrive) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(250.0);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    if (phoenixDrive) {
      yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
    } else {
      yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(
                  () -> {
                    boolean valid = yaw.refresh().getStatus().isOK();
                    if (valid) {
                      return OptionalDouble.of(yaw.getValueAsDouble());
                    } else {
                      return OptionalDouble.empty();
                    }
                  });
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public Rotation2d getGyroRotation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getGyroRotation'");
  }

  @Override
  public double getGyroAngularVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getGyroAngularVelocity'");
  }
}