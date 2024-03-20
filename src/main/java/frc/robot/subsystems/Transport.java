// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase {

  private PearadoxSparkMax transport;

  private final static Transport TRANSPORT = new Transport();
  
  public static Transport getInstance(){
    return TRANSPORT;
  }
  enum eState{
    IN,
    OUT,
  }
  
  private static eState mode;
  /** Creates a new Intake. */
  public Transport() {  
    transport = new PearadoxSparkMax(TransportConstants.TRANSPORT_ID, MotorType.kBrushless, IdleMode.kCoast, 35, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (mode) {
      case IN:
        transport.set(1);
        break;
      case OUT:
        transport.set(-1);
        break;
      default:
      transport.set(0);
        break;
    }
  }

  public void setIn(){
    mode = eState.IN;
  }

  public void setOut(){
    mode = eState.OUT;
  }
}
