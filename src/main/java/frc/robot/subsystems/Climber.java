// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.SmarterDashboard;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private static PearadoxSparkMax leftClimber;
  private static PearadoxSparkMax rightClimber;

  private static SparkPIDController leftClimberController;
  private static SparkPIDController rightClimberController;

  private static RelativeEncoder leftClimberEncoder;
  private static RelativeEncoder rightClimberEncoder;

  private static Climber climber = new Climber();

  private static ClimbMode climbMode = ClimbMode.Normal;

  private double climberAdjust = 0;
  public boolean zeroing = false;
  public boolean trapping = false;

  public enum ClimbMode{
    Normal, Climbing, Climb
  }

  public static Climber getInstance(){
    return climber;
  }


  public Climber() {
    leftClimber = new PearadoxSparkMax(ClimberConstants.LEFT_CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake , 45, false);//TODO: Check for inversion
    rightClimber = new PearadoxSparkMax(ClimberConstants.RIGHT_CLIMBER_ID, MotorType.kBrushless, IdleMode.kBrake , 45, true);
    leftClimberController = leftClimber.getPIDController();
    rightClimberController = rightClimber.getPIDController();
    leftClimberEncoder = leftClimber.getEncoder();
    rightClimberEncoder = rightClimber.getEncoder();
  }

  @Override
  public void periodic() {
    SmarterDashboard.putNumber("Left Climber Position", getLeftPosition(), "Climber");
    SmarterDashboard.putNumber("Right Climber Position", getRightPosition(), "Climber");
    SmarterDashboard.putString("ClimbMode", getClimbMode().toString(), "Climber");
    SmarterDashboard.putNumber("Climber Adjust", climberAdjust, "Climber");

    if (climbMode == ClimbMode.Climbing) {
      setClimberPosition(ClimberConstants.CLIMBING_ROT);
    } else {
      setClimberPosition(0);
    }
    
  }

  public void setNormalMode(){
    climbMode = ClimbMode.Normal;
  }


  public void setClimbingMode(){
    climbMode = ClimbMode.Climbing;
  }

  public void setClimbMode(){
    climbMode = ClimbMode.Climb;
  }

  public ClimbMode getClimbMode(){
    return climbMode;
  }

  public void setClimberPosition(double reference){
    leftClimberController.setReference(reference + climberAdjust, ControlType.kPosition, 0);
    rightClimberController.setReference(reference + climberAdjust, ControlType.kPosition, 0);
  }

  public void setPower(double power){
    leftClimber.set(power);
    rightClimber.set(power);
  }

  public void climbSequenceForward(){
    if(climbMode == ClimbMode.Climbing){
      setClimbMode();
    }else if(climbMode == ClimbMode.Climb && !trapping){
      setTrapping(true);
    }else if(climbMode == ClimbMode.Climb && trapping){
      setTrapping(false);
    }
  }

  public void setZeroing(boolean zeroing){
    this.zeroing = zeroing;
  }

  public void setTrapping(boolean trapping){
    this.trapping = trapping;
  }

  public void zeroClimber(){
    leftClimber.set(-0.15);
    rightClimber.set(-0.15);
  }

  public double getRightPosition(){
    return rightClimberEncoder.getPosition();
  }

  public double getLeftPosition(){
    return leftClimberEncoder.getPosition();
  }
}
