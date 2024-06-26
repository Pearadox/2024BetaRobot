// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.SmarterDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.AmpBarConstants;

public class AmpBar extends SubsystemBase {
  private PearadoxSparkMax ampBar;

  private RelativeEncoder ampBarEncoder;
  private SparkPIDController ampBarController;

  private double ampBarAdjust = 0;

  public enum AmpBarMode{
    Stowed, Deployed, Trap, Climb, Defense
  }

  public AmpBarMode ampBarMode = AmpBarMode.Stowed;

  private static final AmpBar AMP_BAR = new AmpBar();

  public static AmpBar getInstance(){
    return AMP_BAR;
  }

  /** Creates a new AmpBar. */
  public AmpBar() {
    ampBar = new PearadoxSparkMax(AmpBarConstants.AMP_BAR_ID, MotorType.kBrushless, IdleMode.kBrake, 40, false, 
      AmpBarConstants.AMP_BAR_kP, AmpBarConstants.AMP_BAR_kI, AmpBarConstants.AMP_BAR_kD, 
      AmpBarConstants.AMP_BAR_MIN_OUTPUT, AmpBarConstants.AMP_BAR_MAX_OUTPUT);

    ampBarEncoder = ampBar.getEncoder();
    ampBarController = ampBar.getPIDController();
  }

  @Override
  public void periodic() {
    if(RobotContainer.opController.getPOV() == 90){
      ampBarAdjust += 0.06;
    }
    else if(RobotContainer.opController.getPOV() == 270){
      ampBarAdjust -= 0.06;
    }

    SmarterDashboard.putNumber("Amp Bar Position", ampBarEncoder.getPosition(), "Amp Bar");
    SmarterDashboard.putNumber("Amp Bar Adjust", ampBarAdjust, "Amp Bar");
    SmarterDashboard.putNumber("Amp Bar Current", ampBar.getOutputCurrent(), "Amp Bar");
  }

  public void setStowedMode(){
    ampBarMode = AmpBarMode.Stowed;
  }

  public void setDeployedMode(){
    ampBarMode = AmpBarMode.Deployed;
  }

  public void setTrapMode(){
    ampBarMode = AmpBarMode.Trap;
  }

  public void setClimbMode(){
    ampBarMode = AmpBarMode.Climb;
  }

  public void setDefenseMode(){
    ampBarMode = AmpBarMode.Defense;
  }

  public AmpBarMode getAmpBarMode(){
    return ampBarMode;
  }

  public void ampBarHold(){
    if(ampBarMode == AmpBarMode.Deployed){
      ampBarController.setReference(
        AmpBarConstants.DEPLOYED_ROT + ampBarAdjust,
        ControlType.kPosition,
        0);
    }
    else if(ampBarMode == AmpBarMode.Climb){
      ampBarController.setReference(
        AmpBarConstants.CLIMB_ROT + ampBarAdjust,
        ControlType.kPosition,
        0);
    }
    else if(ampBarMode == AmpBarMode.Trap){
      ampBarController.setReference(
        AmpBarConstants.TRAP_ROT + ampBarAdjust,
        ControlType.kPosition,
        0);
    }
    else if(ampBarMode == AmpBarMode.Defense){
      ampBarController.setReference(
        AmpBarConstants.DEFENSE_ROT + ampBarAdjust,
        ControlType.kPosition,
        0);
    }
    else{
      ampBarController.setReference(
        AmpBarConstants.STOWED_ROT,
        ControlType.kPosition,
        0);
    }
  }
}
