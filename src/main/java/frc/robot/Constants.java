// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class SwerveConstants {
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 1;
    public static final int RIGHT_FRONT_DRIVE_ID = 2;
    public static final int LEFT_BACK_DRIVE_ID = 3;
    public static final int RIGHT_BACK_DRIVE_ID = 4;
    
    public static final int LEFT_FRONT_TURN_ID = 5;
    public static final int RIGHT_FRONT_TURN_ID = 6;
    public static final int LEFT_BACK_TURN_ID = 7;
    public static final int RIGHT_BACK_TURN_ID = 8;
    
    public static final int LEFT_FRONT_CANCODER_ID = 11;
    public static final int RIGHT_FRONT_CANCODER_ID = 12;
    public static final int LEFT_BACK_CANCODER_ID = 13;
    public static final int RIGHT_BACK_CANCODER_ID = 14;

    public static final int PIGEON_ID = 15;
    
    //Drivetrain characteristics
    public static final double LEFT_FRONT_OFFSET = 0.307;
    public static final double RIGHT_FRONT_OFFSET = -0.254;
    public static final double LEFT_BACK_OFFSET = 0.026;
    public static final double RIGHT_BACK_OFFSET = 0.144;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5.357; //SDS Mk4i L3+ (60:16 first stage)
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.5;

    public static final double kS_PERCENT = 0.035;
    public static final double kP_PERCENT = 0.020;
    ;

    public static final double DRIVETRAIN_MAX_SPEED = 6.62;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.45 * Math.PI; //TODO: Determine max angular speed

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.75); //TODO: Determine the value
    public static final double WHEEL_BASE = Units.inchesToMeters(22.75); //TODO: Determine the value
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0; //TODO: Determine the value

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    ); //TODO: Determine the values

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED; //TODO: Determine the value
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75; //TODO: Determine the value
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3; //TODO: Determine the value
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3; //TODO: Determine the value

    //Auton constrains
    public static final double AUTO_kP_TRANSLATION = 4; //TODO: Determine the value
    public static final double AUTO_kP_ROTATION = 1.5; //TODO: Determine the value
    
        public static final HolonomicPathFollowerConfig AUTO_CONFIG = new HolonomicPathFollowerConfig(
      new PIDConstants(AUTO_kP_TRANSLATION, 0.0, 0.0),
      new PIDConstants(AUTO_kP_ROTATION, 0.0, 0.0),
      DRIVETRAIN_MAX_SPEED, // Max module speed, in m/s
      DRIVE_BASE_RADIUS,
      new ReplanningConfig()); //TODO: Determine the value
  }
  public static class IntakeConstants{
    public static final int INTAKE_ID = 16;
  }
  public static class TransportConstants{
    public static final int TRANSPORT_ID= 17;
  }
  public static class ClimberConstants{
    public static final int LEFT_CLIMBER_ID = 26;
    public static final int RIGHT_CLIMBER_ID = 25;
    public static final double CLIMBED_POSE = 1; //TODO: Find actual pose
  }
}
