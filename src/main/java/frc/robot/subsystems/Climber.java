// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private static PearadoxSparkMax leftClimber;
  private static PearadoxSparkMax rightClimber;

  private static SparkPIDController leftClimberController;
  private static SparkPIDController rightClimberController;

  private static RelativeEncoder leftClimberEncoder;
  private static RelativeEncoder rightClimberEncoder;

  private static Climber climber = new Climber();

  private static climberState state = climberState.Down;

  public enum climberState{
    Climb,
    Down
  }

  public static Climber getInstance(){
    return climber;
  }


  public Climber() {
    leftClimber = new PearadoxSparkMax(ClimberConstants.LEFT_CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake , 35, false);//TODO: Check for inversion
    rightClimber = new PearadoxSparkMax(ClimberConstants.RIGHT_CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake , 35, true);
    leftClimberController = leftClimber.getPIDController();
    rightClimberController = rightClimber.getPIDController();
    leftClimberEncoder = leftClimber.getEncoder();
    rightClimberEncoder = rightClimber.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setDown(){
    state = climberState.Down;
  }

  public void setClimb(){
    state = climberState.Climb;
  }

  public climberState getState(){
    return state;
  }

  public void setClimbReference(double reference){
    leftClimberController.setReference(reference,ControlType.kPosition,0);
    rightClimberController.setReference(reference, ControlType.kPosition,0);
  }
  public void setPower(double power){
    leftClimber.set(power);
    rightClimber.set(power);
  }

  public double getRightPose(){
    return rightClimberEncoder.getPosition();
  }

  public double getLeftPose(){
    return leftClimberEncoder.getPosition();
  }
}
