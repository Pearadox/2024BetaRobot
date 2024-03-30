// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkFlex;
import frc.lib.util.LerpTable;
import frc.lib.util.SmarterDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
  private PearadoxSparkFlex leftShooter;
  private PearadoxSparkFlex rightShooter;
  
  private PearadoxSparkFlex pivot;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder pivotEncoder;

  private SparkPIDController leftController;
  private SparkPIDController rightController;
  private SparkPIDController pivotController;

  private boolean zeroing = false;

  private static final NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.SHOOTER_LL_NAME);

  private double pivotPosition;
  private double pivotAdjust = 0;
  private double[] botpose_targetspace = new double[6];
  public static final Drivetrain drivetrain = Drivetrain.getInstance();

  private LerpTable pivotLerp = new LerpTable();
  private LerpTable shooterLerp = new LerpTable();

  private static final Shooter SHOOTER = new Shooter();

  public static Shooter getInstance(){
    return SHOOTER;
  }

  public enum ShooterMode{
    Auto, Manual, SourcePassing, StagePassing, Speaker, Trap
  }

  private ShooterMode shooterMode = ShooterMode.Auto;

  public static ShuffleboardTab driverTab;
  private GenericEntry leftShooterSpeedEntry;
  private GenericEntry rightShooterSpeedEntry;
  private GenericEntry shooterModeEntry;
  private GenericEntry pivotAdjustEntry;

  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter = new PearadoxSparkFlex(ShooterConstants.LEFT_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false,
      ShooterConstants.LEFT_SHOOTER_kP, ShooterConstants.LEFT_SHOOTER_kI, ShooterConstants.LEFT_SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);

    rightShooter = new PearadoxSparkFlex(ShooterConstants.RIGHT_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true,
      ShooterConstants.RIGHT_SHOOTER_kP, ShooterConstants.RIGHT_SHOOTER_kI, ShooterConstants.RIGHT_SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);

    pivot = new PearadoxSparkFlex(ShooterConstants.PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true,
      ShooterConstants.PIVOT_kP, ShooterConstants.PIVOT_kI, ShooterConstants.PIVOT_kD,
      ShooterConstants.PIVOT_MIN_OUTPUT, ShooterConstants.PIVOT_MAX_OUTPUT);

    leftEncoder = leftShooter.getEncoder();
    rightEncoder = rightShooter.getEncoder();
    pivotEncoder = pivot.getEncoder();
    
    leftController = leftShooter.getPIDController();
    rightController = rightShooter.getPIDController();
    pivotController = pivot.getPIDController();

    //done
    pivotLerp.addPoint(54, 31.4);
    pivotLerp.addPoint(48, 28.47);
    pivotLerp.addPoint(43, 25.09);
    pivotLerp.addPoint(39, 23.4);
    pivotLerp.addPoint(36, 21.9); // blue red
    pivotLerp.addPoint(33, 20.0); // 15.2 15.6
    pivotLerp.addPoint(31, 18.3);  
    pivotLerp.addPoint(29, 17.2); //12.1 12.7
    pivotLerp.addPoint(26, 15.26); //11.1 11.5

    //not done
    pivotLerp.addPoint(21, 10.2); //10 10.2
    pivotLerp.addPoint(18, 7.3); //7.3 7.3
    pivotLerp.addPoint(15, 4.6); //4.6 4.6

    shooterLerp.addPoint(53, 5.5);
    shooterLerp.addPoint(47, 5.5);
    shooterLerp.addPoint(41, 5.75);
    shooterLerp.addPoint(35, 6);
    shooterLerp.addPoint(29, 6.25);
    shooterLerp.addPoint(23, 6.5);
    shooterLerp.addPoint(15, 7);

    driverTab = Shuffleboard.getTab("Driver");
    leftShooterSpeedEntry = driverTab.add("Left Shooter Speed", 7)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 12)).withPosition(2, 0).getEntry();
    rightShooterSpeedEntry = driverTab.add("Right Shooter Speed", 5)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 12)).withPosition(2, 1).getEntry();
    shooterModeEntry = driverTab.add("Shooter Mode", shooterMode.toString()).withPosition(2, 2).getEntry();
    pivotAdjustEntry = driverTab.add("Shooter Pivot Adjust", pivotAdjust).withPosition(3, 2).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmarterDashboard.putString("Shooter Mode", getShooterMode().toString(), "Shooter");
    SmarterDashboard.putNumber("Shooter Left Speed", leftEncoder.getVelocity(), "Shooter");
    SmarterDashboard.putNumber("Shooter Right Speed", rightEncoder.getVelocity(), "Shooter");
    SmarterDashboard.putNumber("Shooter Pivot Position", pivotEncoder.getPosition(), "Shooter");
    SmarterDashboard.putNumber("Shooter Pivot Intended Position", pivotPosition, "Shooter");
    SmarterDashboard.putNumber("Shooter Pivot Current", pivot.getOutputCurrent(), "Shooter");
    SmarterDashboard.putNumber("Shooter Pivot Intended Angle", calculatePivotAngle(), "Shooter");  
    SmarterDashboard.putBoolean("Shooter Has Priority Target", hasPriorityTarget(), "Shooter"); 
    SmarterDashboard.putNumber("Shooter Pivot Adjust", pivotAdjust, "Shooter");
    SmarterDashboard.putNumber("Note Velocity", getNoteVelocity(), "Shooter");
    SmarterDashboard.putNumber("Shooter Left Temperature", leftShooter.getMotorTemperature(), "Shooter");
    SmarterDashboard.putNumber("Shooter Right Temperature", rightShooter.getMotorTemperature(), "Shooter");
    
    shooterModeEntry.setString(shooterMode.toString());
    pivotAdjustEntry.setDouble(pivotAdjust);
  }

  public void shooterHold(){
    double shooterVoltage = shooterLerp.interpolate(calculatePivotAngle());
    SmarterDashboard.putNumber("Shooter Auto Voltage", shooterVoltage, "Shooter");

    if(DriverStation.isAutonomousEnabled()){
      leftShooter.set(0.7);
      rightShooter.set(0.6);
    }
    else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){ //Amp
      leftController.setReference(
        ShooterConstants.AMP_VOLTAGE,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        ShooterConstants.AMP_VOLTAGE - 0.1,
        ControlType.kVoltage,
        0);
    }
    else if(shooterMode == ShooterMode.SourcePassing){
      leftController.setReference(
        ShooterConstants.SOURCE_PASSING_VOLTAGE,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        ShooterConstants.SOURCE_PASSING_VOLTAGE - ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET,
        ControlType.kVoltage,
        0);
    }
    else if(shooterMode == ShooterMode.StagePassing){
      leftController.setReference(
        ShooterConstants.STAGE_PASSING_VOLTAGE, 
        ControlType.kVoltage,
        0);

      rightController.setReference(
        ShooterConstants.STAGE_PASSING_VOLTAGE-ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET,
        ControlType.kVoltage,
        0);
    }
    else if(shooterMode == ShooterMode.Speaker){
      leftController.setReference(
        ShooterConstants.SPEAKER_VOLTAGE,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        ShooterConstants.SPEAKER_VOLTAGE-ShooterConstants.LEFT_TO_RIGHT_VOLTAGE_OFFSET,
        ControlType.kVoltage,
        0);
    }
    else if(shooterMode == ShooterMode.Manual){
      leftController.setReference(
        leftShooterSpeedEntry.getDouble(7),
        ControlType.kVoltage,
        0);

      rightController.setReference(
        rightShooterSpeedEntry.getDouble(5),
        ControlType.kVoltage,
        0);
    }
    else{
      leftController.setReference(
        shooterVoltage,
        ControlType.kVoltage,
        0);

      rightController.setReference(
        shooterVoltage - 2,
        ControlType.kVoltage,
        0);
    }
  }

  public void setShooterAuto(double speed){
    setAutoMode();
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void pivotHold(){
    if(zeroing){
      pivot.set(-0.15);
    }
    else if(RobotContainer.driverController.getLeftTriggerAxis() >= 0.95){
      pivotController.setReference(
        ShooterConstants.AMP_PIVOT_POSITION,
        ControlType.kPosition,
        0);

      pivotPosition = ShooterConstants.AMP_PIVOT_POSITION;
    }
    else if(shooterMode == ShooterMode.SourcePassing || shooterMode == ShooterMode.StagePassing){
      pivotController.setReference(
        ShooterConstants.PASSING_PIVOT_POSITION,
        ControlType.kPosition,
        0);

      pivotPosition = ShooterConstants.PASSING_PIVOT_POSITION;
    }
    else if(shooterMode == ShooterMode.Speaker){
      pivotController.setReference(
        ShooterConstants.SPEAKER_PIVOT_POSITION,
        ControlType.kPosition,
        0);

      pivotPosition = ShooterConstants.SPEAKER_PIVOT_POSITION;
    }
    else{
      if(shooterMode == ShooterMode.Auto && hasPriorityTarget()){
        setPivotAngle(calculatePivotAngle());
      }

      pivotController.setReference(
        pivotPosition + pivotAdjust,
        ControlType.kPosition,
        0);
    }

    if(RobotContainer.opController.getPOV() == 0){
      pivotAdjust += 0.1;
    }
    else if(RobotContainer.opController.getPOV() == 180){
      pivotAdjust -= 0.1;
    }
  }

  public void setZeroing(boolean zeroing){
    this.zeroing = zeroing;
  }

  public void resetPivotEncoder(){
    pivotEncoder.setPosition(0);
  }

  public void setBrakeMode(boolean brake){
    if(brake){
      leftShooter.setIdleMode(IdleMode.kBrake);
      rightShooter.setIdleMode(IdleMode.kBrake);
      pivot.setIdleMode(IdleMode.kBrake);

    }
    else{
      leftShooter.setIdleMode(IdleMode.kCoast);
      rightShooter.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setPivot(double speed){
    pivot.set(speed);
  }

  public double getPivotCurrent(){
    return pivot.getOutputCurrent();
  }

  public double calculatePivotAngle(){
    if(hasPriorityTarget()){
      botpose_targetspace = llTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    double x = Math.abs(botpose_targetspace[0]);
    double z = Math.abs(botpose_targetspace[2]);
    if(!isRedAlliance()){ //edit for houston comp
      z += 0.02;  
    }

    double hypot = Math.hypot(x, z);

    double angle = Math.atan((FieldConstants.SPEAKER_HEIGHT - ShooterConstants.FLOOR_TO_SHOOTER) / hypot);
    return Units.radiansToDegrees(angle);
  }

  public void setPivotAngle(double angle){
    if(hasPriorityTarget()){
      pivotPosition = pivotLerp.interpolate(angle);
    }
  }

  public void setPivotPosition(){
    pivotController.setReference(
      pivotPosition + pivotAdjust,
      ControlType.kPosition,
      0);
  }

  public double getNoteVelocity(){
    return 2 * (((leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2) * 2 * Math.PI * Units.inchesToMeters(1.5) / 60);
  }

  public boolean hasPriorityTarget(){
    if(isRedAlliance()){
      return llTable.getEntry("tid").getDouble(0) == 4;
    }
    else{
      return llTable.getEntry("tid").getDouble(0) == 7;
    }
  }

  public void setPipeline(int index){
    llTable.getEntry("pipeline").setNumber(index);
  }

  public void setPivotPosition(double position){
    pivotPosition = position;
  }

  public ShooterMode getShooterMode(){
    return shooterMode;
  }

  public void setAutoMode(){
    shooterMode = ShooterMode.Auto;
  }

  public void setManualMode(){
    shooterMode = ShooterMode.Manual;
  }

  public void setSourcePassingMode(){
    shooterMode = ShooterMode.SourcePassing;
  }
  
  public void setStagePassingMode(){
    shooterMode = ShooterMode.StagePassing;
  }

  public void setSpeakerMode(){
    shooterMode = ShooterMode.Speaker;
  }

  public void setTrapMode(){
    shooterMode = ShooterMode.Trap;
  }

  public boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public boolean readyToShoot() {
    return (Math.abs(pivotPosition + pivotAdjust - pivotEncoder.getPosition()) <= 0.9);
  }
}
