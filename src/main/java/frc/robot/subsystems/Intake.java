// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private PearadoxSparkMax intake;

  private final static Intake INTAKE = new Intake();
  
  /* Returns the intake subsystem*/
  public static Intake getInstance(){
    return INTAKE;
  }
  enum eState{
    IN,
    OUT,
    MOVING
  }
  
  private static eState mode;
  /** Creates a new Intake. */
  public Intake() {  
    intake = new PearadoxSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless, IdleMode.kCoast, 35, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (mode) {
      case IN:
        intake.set(1);
        break;
      case OUT:
        intake.set(-1);
        break;
      case MOVING:
        intake.set(0.3);
        break;
      default:
      intake.set(0);
        break;
    }
  }

  public void setIn(){
    mode = eState.IN;
  }

  public void setOut(){
    mode = eState.OUT;
  }

  public void setRunning(){
    mode = eState.MOVING;
  }
}
