// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.SmarterDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront;
  private SwerveModule rightFront;
  private SwerveModule leftBack;
  private SwerveModule rightBack;

  private SlewRateLimiter frontLimiter;
  private SlewRateLimiter sideLimiter;
  private SlewRateLimiter turnLimiter;

  private PIDController alignPIDController;
  private PIDController noteAlignPIDController;

  private Pigeon2 gyro;

  private static final NetworkTable shooterllTable = NetworkTableInstance.getDefault().getTable(VisionConstants.SHOOTER_LL_NAME);
  public static final NetworkTable intakellTable = NetworkTableInstance.getDefault().getTable(VisionConstants.INTAKE_LL_NAME);

  public enum DriveMode{
    Normal, Align, NoteAlign
  }

  private DriveMode driveMode = DriveMode.Normal;

  private double lastHeading;

  public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
  private GenericEntry leftFrontStateEntry;
  private GenericEntry rightFrontStateEntry;
  private GenericEntry leftBackStateEntry;
  private GenericEntry rightBackStateEntry;
  private GenericEntry robotAngleEntry;
  private GenericEntry angularSpeedEntry;

  private static final Drivetrain DRIVETRAIN = new Drivetrain();

  /* 
  *<p> 
  *This returns the Drivetrain instead of creating a new subsystem
  *@return new Drivetrain
  */
  public static Drivetrain getInstance(){
    return DRIVETRAIN;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    leftFront = new SwerveModule(
      SwerveConstants.LEFT_FRONT_DRIVE_ID, 
      SwerveConstants.LEFT_FRONT_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_FRONT_CANCODER_ID, 
      SwerveConstants.LEFT_FRONT_OFFSET);

    rightFront = new SwerveModule(
      SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
      SwerveConstants.RIGHT_FRONT_TURN_ID, 
      false, 
      false, 
      SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
      SwerveConstants.RIGHT_FRONT_OFFSET);

    leftBack = new SwerveModule(
      SwerveConstants.LEFT_BACK_DRIVE_ID, 
      SwerveConstants.LEFT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_BACK_CANCODER_ID, 
      SwerveConstants.LEFT_BACK_OFFSET);
    
    rightBack = new SwerveModule(
      SwerveConstants.RIGHT_BACK_DRIVE_ID, 
      SwerveConstants.RIGHT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.RIGHT_BACK_CANCODER_ID, 
      SwerveConstants.RIGHT_BACK_OFFSET);

    frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

    alignPIDController = new PIDController(SwerveConstants.kP_PERCENT, 0, 0);
    noteAlignPIDController = new PIDController(SwerveConstants.kP_PERCENT, 0, 0);


    gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
    
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      SwerveConstants.AUTO_CONFIG,
      () -> isRedAlliance(),
      this);

    lastHeading = getHeading();

    leftFrontStateEntry = swerveTab.add("Left Front Module State", leftFront.getState().toString()).withSize(4, 1).withPosition(0, 0).getEntry();
    rightFrontStateEntry = swerveTab.add("Right Front Module State", rightFront.getState().toString()).withSize(4, 1).withPosition(0, 1).getEntry();
    leftBackStateEntry = swerveTab.add("Left Back Module State", leftBack.getState().toString()).withSize(4, 1).withPosition(0, 2).getEntry();
    rightBackStateEntry = swerveTab.add("Right Back Module State", rightBack.getState().toString()).withSize(4, 1).withPosition(0, 3).getEntry();
    robotAngleEntry = swerveTab.add("Robot Angle", getHeading()).withSize(1, 1).withPosition(4, 1).getEntry();
    angularSpeedEntry = swerveTab.add("Angular Speed", new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "\u03C0" + " rad/s").withSize(1, 1).withPosition(5, 1).getEntry();
  }

  @Override
  public void periodic() {
    RobotContainer.poseEstimation.updateOdometry(getHeadingRotation2d(), getModulePositions());

    SmarterDashboard.putString("Drive Mode", getDriveMode().toString(), "Drivetrain");
    SmarterDashboard.putNumber("Robot Angle", getHeading(), "Drivetrain");
    SmarterDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "\u03C0" + " rad/s", "Drivetrain");
    SmarterDashboard.putBoolean("Ready To Shoot", readyToShoot(), "Drivetrain");
    SmarterDashboard.putString("Tag 7 Pose", RobotContainer.aprilTagFieldLayout.getTagPose(7).get().toPose2d().toString(), "Drivetrain");

    SmarterDashboard.putData("Left Front Module State", leftFront.getState(), "Drivetrain");
    SmarterDashboard.putData("Right Front Module State", rightFront.getState(), "Drivetrain");
    SmarterDashboard.putData("Left Back Module State", leftBack.getState(), "Drivetrain");
    SmarterDashboard.putData("Right Back Module State", rightBack.getState(), "Drivetrain");
    SmarterDashboard.putData("Odometry", getPose(), "Drivetrain");

    leftFrontStateEntry.setString(leftFront.getState().toString());
    rightFrontStateEntry.setString(rightFront.getState().toString());
    leftBackStateEntry.setString(leftBack.getState().toString());
    rightBackStateEntry.setString(rightBack.getState().toString());
    robotAngleEntry.setDouble(getHeading());
    angularSpeedEntry.setString(new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "\u03C0" + "rad/s");
  }
/*
 *<p> 
 *The method to drive the robot
 *@param Front speed  The speed the robot moves forward
 *@param Side speed   The speed the robot moves to the side
 *@param Turn speed   The speed that the robot turns
 *@param Field oriented    A boolean for if the it turns/moves based on its field orientation
 *@param Center of Rotation    A new Translation2d for where the center of rotation is
 *@param Deadband   A boolean for if it takes into account controller deadband
 */
  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ //Drive with rotational speed control w/ joystick
    if(driveMode == DriveMode.Align && deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
    }
    else{
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED * (RobotContainer.driverController.getRightStickButton() ? 1.25 : 1);

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnX, double turnY, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadbandX, boolean deadbandY, boolean deadbandTurn){ //Drive with rotational heading control w/ joystick
    if(deadbandX){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
    }
    if(deadbandY){
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
    }
    if(deadbandTurn){
      turnX = Math.abs(turnX) > 0.1 ? turnX : 0;
      turnY = Math.abs(turnY) > 0.1 ? turnY : 0;
    }

    double turnSpeed;
    if(turnX == 0 && turnY == 0){
      turnSpeed = 0;
    }
    else{
      double error = getJoystickAngle(turnX, turnY) - getHeading();
    
      if(error > 180) {
        error -= 360;
      }
      else if(error < -180){
        error += 360;
      }
    
      if(Math.abs(error) > 1){
        turnSpeed = Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error;
      }
      else{
        turnSpeed = 0;
      }
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public double getJoystickAngle(double turnX, double turnY){
    double targetAngle;

    if(turnX != 0){
      targetAngle = Math.toDegrees(Math.atan2(-turnX, turnY));   
    }
    else{
      targetAngle = Math.toDegrees(Math.atan2(turnX, turnY));
    }
    
    return targetAngle;
  }

  public void swerveDrive(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){ //Drive with field relative chassis speeds
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getHeadingRotation2d());

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void turnToHeading(double heading, Translation2d centerOfRotation){
    double turnSpeed;

    if(DriverStation.isAutonomousEnabled()){
      heading = isRedAlliance() ? -heading : heading;
    }

    double error = heading - getHeading();

    if(error > 180) {
      error -= 360;
    }
    else if(error < -180){
      error += 360;
    }
    
    if(Math.abs(error) > 1){
      turnSpeed = Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error;
    }
    else{
      turnSpeed = 0;
    }

    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turnSpeed, getHeadingRotation2d());

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public boolean hasTurnedToHeading(double heading){
    heading = isRedAlliance() ? -heading : heading;
    return Math.abs(heading - getHeading()) < 1;
  }

  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }
/*Returns the estimated pose */
  public Pose2d getPose(){
    return RobotContainer.poseEstimation.getEstimatedPose();
    
  }
/* resets the estimaded pose */
  public void resetPose(Pose2d pose) {
    RobotContainer.poseEstimation.resetPose(pose);
  }
/*Returns the estimated speed of the robot */
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public ChassisSpeeds getFieldRelativeSpeeds(){
    ChassisSpeeds chassisSpeeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getHeadingRotation2d());
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }
/* Sets the heading of the gyro to the specified double value */
  public void setHeading(double heading){
    gyro.setYaw(heading);
  }
/* returns the heading */
  public double getHeading(){
    return Math.IEEEremainder(-gyro.getAngle(), 360); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getAngularSpeed() {
    return gyro.getRate();
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }
/* Sets the modules to the specified state */
  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  }
/* Returns true if we are on the red alliance */
  public boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }


  public double getAlignSpeed(){
    double alignSpeed;

    if(isRedAlliance()){
      if(shooterllTable.getEntry("tid").getDouble(0) == 4){
        double error = shooterllTable.getEntry("tx").getDouble(0);
        
        alignSpeed = Math.abs(error) > 0.5 ? -alignPIDController.calculate(shooterllTable.getEntry("tx").getDouble(0), 0) + (Math.signum(error) * SwerveConstants.kS_PERCENT): 0;
      }
      else{
        double alignAngle = getAlignAngle(4);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
        

        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }
    else{
      if(shooterllTable.getEntry("tid").getDouble(0) == 7){
        double error = shooterllTable.getEntry("tx").getDouble(0);
        
        alignSpeed = Math.abs(error) > 0.5 ? -alignPIDController.calculate(shooterllTable.getEntry("tx").getDouble(0), 0) + (Math.signum(error) * SwerveConstants.kS_PERCENT): 0;
      }
      else{
        double alignAngle = getAlignAngle(7);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
          }
        
        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }

    return alignSpeed;
  }

  public double getNoteAlignSpeed(){
    double error = intakellTable.getEntry("tx").getDouble(0) + 2;
    double kS = error > 0 ? SwerveConstants.kS_PERCENT * 1.75 : SwerveConstants.kS_PERCENT * 0.75;

    double alignSpeed = Math.abs(error) > 2 ? -noteAlignPIDController.calculate(error, 0) + (Math.signum(error) * kS) : 0;
    return alignSpeed;
  }

  public void recordLastHeading(){
    lastHeading = getHeading();
  }

  public void setLastHeading(){
    setHeading(lastHeading);
  }

  public double getSourceAlignSpeed(){
    double alignSpeed;

    if(isRedAlliance()){
      if(shooterllTable.getEntry("tid").getDouble(0) == 4){
        double error = shooterllTable.getEntry("tx").getDouble(0);
        
        alignSpeed = Math.abs(error) > 0.5 ? -alignPIDController.calculate(shooterllTable.getEntry("tx").getDouble(0), 0) + (Math.signum(error) * SwerveConstants.kS_PERCENT): 0;
      }
      else{
        double alignAngle = getAlignAngle(4);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
        
        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }
    else{
      if(shooterllTable.getEntry("tid").getDouble(0) == 7){
        double error = shooterllTable.getEntry("tx").getDouble(0) - 1;
        
        alignSpeed = Math.abs(error) > 0.5 ? -alignPIDController.calculate(shooterllTable.getEntry("tx").getDouble(0) - 1, 0) + (Math.signum(error) * SwerveConstants.kS_PERCENT): 0;
      }
      else{
        double alignAngle = getAlignAngle(7);

        double error = alignAngle - getHeading();

        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
          }
        
        if(Math.abs(error) > 1){
          alignSpeed = Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error;
        }
        else{
          alignSpeed = 0;
        }
      }
    }

    return alignSpeed;
  }

  public double getPassingAlignSpeed(){
    double alignAngle = isRedAlliance() ? getPassingAngle() - 8 : getPassingAngle() - 13;

    double error = alignAngle - getHeading();

    if(error > 180) {
      error -= 360;
    }
    else if(error < -180){
      error += 360;
    }
    
    if(Math.abs(error) > 1){
      return (Math.signum(-error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * -error);
    }
    else{
      return 0;
    }
  }

  public boolean readyToShoot(){
    double error = shooterllTable.getEntry("tx").getDouble(0) - 2;

    if(isRedAlliance()){
      return Math.abs(error) <= 0.5 && shooterllTable.getEntry("tid").getDouble(0) == 4; 
    }
    else{
      return Math.abs(error) <= 0.5 && shooterllTable.getEntry("tid").getDouble(0) == 7;
    }
  }

  public double getAlignAngle(int tagID){
    Pose2d tagPose = RobotContainer.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
    Pose2d robotPose = getPose();

    double deltaX = tagPose.getX() - robotPose.getX();
    double deltaY = (tagPose.getY() - 0.11) - robotPose.getY();

    double alignAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 2;

    if(!isRedAlliance()){
      alignAngle += 180;
      if(alignAngle > 180){
        alignAngle -= 360;
      }
    }

    return alignAngle;
  }

  public double getPassingAngle(){
    Pose2d targetPose = isRedAlliance() ? new Pose2d(new Translation2d(15.48, 7.08), new Rotation2d()) : new Pose2d(new Translation2d(0.93, 7.08), new Rotation2d());
    Pose2d robotPose = getPose();

    double deltaX = targetPose.getX() - robotPose.getX();
    double deltaY = (targetPose.getY() - 0.11) - robotPose.getY();

    double alignAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    if(!isRedAlliance()){
      alignAngle += 180;
      if(alignAngle > 180){
        alignAngle -= 360;
      }
    }

    return alignAngle;
  }

  public double getPassingDistance(){
    Pose2d targetPose = isRedAlliance() ? new Pose2d(new Translation2d(15.48, 7.08), new Rotation2d()) : new Pose2d(new Translation2d(0.93, 7.08), new Rotation2d());
    Pose2d robotPose = getPose();

    double deltaX = targetPose.getX() - robotPose.getX();
    double deltaY = targetPose.getY() - robotPose.getY();

    return Math.hypot(Math.abs(deltaX), Math.abs(deltaY)) > 8 ? Math.hypot(Math.abs(deltaX), Math.abs(deltaY)) : 8;
  }

  
  public void changeIntakePipeline(int pipeline){
    intakellTable.getEntry("pipeline").setNumber(pipeline);
  }

  public DriveMode getDriveMode(){
    return driveMode;
  }

  public void setNormalMode(){
    driveMode = DriveMode.Normal;
  }

  public void setAlignMode(){
    driveMode = DriveMode.Align;
  }

  public void setNoteAlignMode(){
    driveMode = DriveMode.NoteAlign;
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0.25))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)));
  }

  public void setRobotOrientation(String limelightName, double yaw, double yawRate, 
    double pitch, double pitchRate, double roll, double rollRate) {

      double[] entries = new double[6];
      entries[0] = yaw;
      entries[1] = yawRate;
      entries[2] = pitch;
      entries[3] = pitchRate;
      entries[4] = roll;
      entries[5] = rollRate;

      shooterllTable.getEntry("robot_orientation_set").setDoubleArray(entries);
  }
}