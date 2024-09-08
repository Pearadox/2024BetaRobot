// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.SmarterDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  private PearadoxSparkMax utbRoller;

  private boolean rumbled = false;

  private Debouncer debouncerRising;
  private Debouncer debouncerFalling;


  private static final Intake INTAKE = new Intake();

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.INTAKE_LL_NAME);

  public static Intake getInstance(){
    return INTAKE;
  }

  /** Creates a new Intake. */
  public Intake() {
    utbRoller = new PearadoxSparkMax(IntakeConstants.UTB_ROLLER_ID, MotorType.kBrushless, IdleMode.kCoast, 80, false);
    debouncerRising = new Debouncer(0.3, DebounceType.kRising);
    debouncerFalling = new Debouncer(0.2, DebounceType.kFalling);

  }

  @Override
  public void periodic() {
    SmarterDashboard.putNumber("Intake Current", utbRoller.getOutputCurrent(), "Intake");
    SmarterDashboard.putBoolean("Intake Has Target Rising", hasTargetRising(), "Intake");
    SmarterDashboard.putBoolean("Intake Has Target Falling", hasTargetFalling(), "Intake");
    SmarterDashboard.putBoolean("Intake Has Target", hasTargetFalling(), "Intake");

    if(!rumbled && utbRoller.getOutputCurrent() > 45){
      CommandScheduler.getInstance().schedule(rumbleController());
      rumbled = true;
    }
    if(rumbled && !(utbRoller.getOutputCurrent() > 45)){
      rumbled = false;
    } 
  }

  public void utbIntakeIn(){
    utbRoller.set(0.8);
  }

  public void utbIntakeOut(){
    utbRoller.set(-0.7);
  }

  public void utbIntakeStop(){
    utbRoller.set(0);
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.75))
      .andThen(new InstantCommand(() -> RobotContainer.opController.setRumble(RumbleType.kBothRumble, 0.75)))
      .andThen(new WaitCommand(0.25))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)))
      .andThen(new InstantCommand(() -> RobotContainer.opController.setRumble(RumbleType.kBothRumble, 0)));
  }

  public boolean hasTargetRising(){
    return debouncerRising.calculate(llTable.getEntry("tv").getDouble(0) == 1);
  }

  public boolean hasTargetFalling(){
    return debouncerFalling.calculate(llTable.getEntry("tv").getDouble(0) == 1);
  }

  public boolean hasTargetRaw(){
    return llTable.getEntry("tv").getDouble(0) == 1;
  }
}
