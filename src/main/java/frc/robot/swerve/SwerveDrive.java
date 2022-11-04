package frc.robot.swerve;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Robot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


public class SwerveDrive implements Loggable {
  public boolean autoLimeLightAim = false;
  public boolean acceleratedInputs = true;
  public boolean defensiveStop = true;

  public double previousXDistance = 0;
  public double previousYDistance = 0;
  public double previousTimestamp = 0;
  private double SDxSpeed=0;
  private double SDySpeed=0;
  private double SDrotation=0;

  public ArrayList<Double> velocities = new ArrayList<Double>();
  private static SwerveDrive SINGLE_INSTANCE = new SwerveDrive();
  
  public boolean SDFieldRelative= SwerveConstants.DEFAULT_FIELD_RELATIVE_DRIVE;
  public boolean holdRobotAngleEnabled = SwerveConstants.DEFAULT_HOLD_ROBOT_ANGLE;
  public PIDController holdRobotAngleController = new PIDController(SwerveConstants.ROBOTHoldAngleKP, 0, 0);
  
  public double holdRobotAngleSetpoint = SwerveConstants.DEFAULT_HOLD_ROBOT_ANGLE_SETPOINT;
  public double joystickDriveGovernor = SwerveConstants.SPEED_GOVERNOR;
  public String NeutralMode = "Brake";

  public final Translation2d m_frontLeftLocation = new Translation2d(SwerveConstants.WHEEL_BASE_METERS/2, SwerveConstants.TRACK_WIDE/2);
  public final Translation2d m_frontRightLocation = new Translation2d(SwerveConstants.WHEEL_BASE_METERS/2, -SwerveConstants.TRACK_WIDE/2);
  public final Translation2d m_backLeftLocation = new Translation2d(-SwerveConstants.WHEEL_BASE_METERS/2, SwerveConstants.TRACK_WIDE/2);
  public final Translation2d m_backRightLocation = new Translation2d(-SwerveConstants.WHEEL_BASE_METERS/2, -SwerveConstants.TRACK_WIDE/2);
  public final List<Translation2d> getSwerveBotTranslationList = List.of(m_frontLeftLocation, m_frontRightLocation,m_backLeftLocation, m_backRightLocation);
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  public SwerveDriveOdometry m_odometry;

  public static SwerveDrive getInstance() {
    return SINGLE_INSTANCE;
  }

  public void init(){
    velocities.add(0.0);
    velocities.add(0.0);
    m_odometry = new SwerveDriveOdometry(m_kinematics, SwerveMap.getRobotAngle());
    holdRobotAngleController.disableContinuousInput();
    holdRobotAngleController.setTolerance(Math.toRadians(2));
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }


  public void swervePeriodic() {
    joystickDrive();
    drive(
      getSDxSpeed(), 
      getSDySpeed(), 
      getSDRotation(), 
      getSDFieldRelative());
    //getVelocities();
  }
  /**
  * Method to drive the robot using the following params
  *
  * @param _xSpeed Speed of the robot in the x direction (forward).
  * @param _ySpeed Speed of the robot in the y direction (sideways).
  * @param _rot Angular rate of the robot.
  * @param _fieldRelative Whether the provided x and y speeds are relative to the field.
  */
  @SuppressWarnings("ParameterName")
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
    if (Robot.xbox.getRightStickButton() || autoLimeLightAim){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      _rot = holdRobotAngleController.calculate(SwerveMap.getRobotAngle().getRadians(), ((getRobotAngleDegrees() - limelightTX())/360)*(2*Math.PI));
      holdRobotAngleSetpoint = SwerveMap.getRobotAngle().getRadians();
    //  System.out.println(limelightTX());
    // } else if (_xSpeed == 0 && _ySpeed == 0 && _rot == 0 && holdRobotAngleEnabled) {
    //   System.out.println("Evan sucks booty%");
    
    //   NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
  } else if (Robot.xbox.getLeftStickButton()){
    _fieldRelative = false;
  } else {
    holdRobotAngleSetpoint = SwerveMap.getRobotAngle().getRadians();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }
  SwerveModuleState[] moduleStates =
  m_kinematics.toSwerveModuleStates( _fieldRelative ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, SwerveMap.getRobotAngle())
        : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));

      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_SPEED_METERSperSECOND);
      
      if (defensiveStop && _xSpeed == 0 && _ySpeed == 0 && _rot == 0) {
        SwerveMap.FrontRightSwerveModule.setSteeringAngle(135);
        SwerveMap.FrontRightSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);

        SwerveMap.FrontLeftSwerveModule.setSteeringAngle(45);
        SwerveMap.FrontLeftSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);

        SwerveMap.BackRightSwerveModule.setSteeringAngle(45);
        SwerveMap.BackRightSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);

        SwerveMap.BackLeftSwerveModule.setSteeringAngle(135);
        SwerveMap.BackLeftSwerveModule.mDriveMotor.set(ControlMode.PercentOutput, 0);

      } else {
        SwerveMap.FrontLeftSwerveModule.setDesiredState(moduleStates[0]);
        SwerveMap.FrontRightSwerveModule.setDesiredState(moduleStates[1]);
        SwerveMap.BackLeftSwerveModule.setDesiredState(moduleStates[2]);
        SwerveMap.BackRightSwerveModule.setDesiredState(moduleStates[3]);

      }
    }
  /**This ONLY saves speeds.  You must also call the drive method */  
  public void joystickDrive(){
    double x;
    double y;
    double rot;

    if (acceleratedInputs) {
      x = -Robot.xbox.getLeftY();
      y = -Robot.xbox.getLeftX();
      rot = -Robot.xbox.getRightX();
    } else {
      x = -Math.signum(Robot.xbox.getLeftY()) * Math.sqrt(Math.abs(Robot.xbox.getLeftY()));
      y = -Math.signum(Robot.xbox.getLeftX()) * Math.sqrt(Math.abs(Robot.xbox.getLeftX()));
      rot = -Math.signum(Robot.xbox.getRightX()) * Math.sqrt(Math.abs(Robot.xbox.getRightX()));
    }

    SDxSpeed = convertToMetersPerSecond(deadband(x))*joystickDriveGovernor;
    SDySpeed = convertToMetersPerSecond(deadband(y))*joystickDriveGovernor;
    SDrotation = convertToRadiansPerSecond(deadband(rot))*joystickDriveGovernor;
    //System.out.println(SDrotation);
    
  }

  /**
   * MUST BE ADDED TO PERIODIC (NOT INIT METHODS)
   * sets all the talons (steer and drive motors) to coast.
   * This allows for easy moving of the robot
   */
  public void setToCoast(){
          
    if (NeutralMode == "Brake" &&
      Math.abs(SwerveMap.FrontLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity())  < 100 &&
      Math.abs(SwerveMap.BackLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity())   < 100 &&
      Math.abs(SwerveMap.FrontRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()) < 100 &&
      Math.abs(SwerveMap.BackRightSwerveModule.mDriveMotor.getSelectedSensorVelocity())  < 100) {
        SwerveMap.FrontRightSwerveModule.swerveDisabledInit();
        SwerveMap.BackRightSwerveModule.swerveDisabledInit();
        SwerveMap.FrontLeftSwerveModule.swerveDisabledInit();
        SwerveMap.BackLeftSwerveModule.swerveDisabledInit();
        NeutralMode = "Coast";
      }

  }
/**When we drive around we want the robot to brake... nuff said */
  public void setToBrake(){
    SwerveMap.FrontRightSwerveModule.swerveEnabledInit();
    SwerveMap.BackRightSwerveModule.swerveEnabledInit();
    SwerveMap.FrontLeftSwerveModule.swerveEnabledInit();
    SwerveMap.BackLeftSwerveModule.swerveEnabledInit();
    NeutralMode = "Brake";
  }

  public void enableCurrentLimiting(){
    SwerveMap.FrontRightSwerveModule.enableCurrentLimiting();
    SwerveMap.BackRightSwerveModule.enableCurrentLimiting();
    SwerveMap.FrontLeftSwerveModule.enableCurrentLimiting();
    SwerveMap.BackLeftSwerveModule.enableCurrentLimiting();
  }

  public void disableCurrentLimiting(){
    SwerveMap.FrontRightSwerveModule.disableCurrentLimiting();
    SwerveMap.BackRightSwerveModule.disableCurrentLimiting();
    SwerveMap.FrontLeftSwerveModule.disableCurrentLimiting();
    SwerveMap.BackLeftSwerveModule.disableCurrentLimiting();
  }

  public double limelightTX() {  
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  } //Testing kP=1.5

/**Sets the robots speed parameters to zero */
  public void zeroSwerveDrive(){
    SDxSpeed = 0;
    SDySpeed = 0;
    SDrotation = 0;
  }

  private double convertToMetersPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_METERSperSECOND;
  }

  private double convertToRadiansPerSecond(double _input){
    return _input*SwerveConstants.MAX_SPEED_RADIANSperSECOND;
  }
  private double deadband(double _input){
      if(Math.abs(_input)<= Constants.XBOXDEADBAND){
        _input = 0;
      }
      return _input;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
      
      SwerveMap.getRobotAngle(),
    SwerveMap.FrontLeftSwerveModule.getState(),
    SwerveMap.FrontRightSwerveModule.getState(),
    SwerveMap.BackLeftSwerveModule.getState(),
    SwerveMap.BackRightSwerveModule.getState());
    //System.out.println("x= " + m_odometry.getPoseMeters().getX() + " y="+m_odometry.getPoseMeters().getY());
    //System.out.println("FL: " + Math.round(SwerveMap.FrontLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()) + " FR: " +Math.round(SwerveMap.FrontRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()));
    //System.out.println("BL: " + Math.round(SwerveMap.BackLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()) + " BR: " +Math.round(SwerveMap.BackRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()));
  }
  
  @Log.Gyro(name = "Robot Angle", rowIndex = 2, columnIndex = 5)
  private AHRS getGyro(){
    return SwerveMap.GYRO;
  }

  public boolean getSDFieldRelative() {
    return SDFieldRelative;
  }

  public void setSDxSpeed(double _input) {
    SDxSpeed = _input;
  }
  public void setSDySpeed(double _input) {
    SDySpeed = _input;
  }
  public void setSDRotation(double _input) {
    SDrotation = _input;
  }
  @Log.NumberBar(min = -5, max = 5, rowIndex = 0, columnIndex = 7, height = 1, width = 1) 
  public double getSDxSpeed() {
    return SDxSpeed;
  }
  @Log.NumberBar(min = -5, max = 5, rowIndex = 0, columnIndex = 8,height = 1, width = 1)
  public double getSDySpeed(){
    return SDySpeed;
  }
  @Log.Dial(rowIndex = 0, columnIndex = 9, height = 1, width = 1)
  public double getSDRotation() {
    return SDrotation;
  }

  @Config(defaultValueBoolean = true)
  public void setAcceleratedInput(boolean _input) {
    acceleratedInputs = _input;
  }

  @Config.ToggleButton(name = "FieldOriented?", defaultValue = true, rowIndex = 1, columnIndex =0, height = 1, width = 2)
  public void setSDFieldRelative(boolean _input) {
    SDFieldRelative = _input;
  }

  @Config.ToggleButton(name = "Hold Robot Angle?", defaultValue = false, rowIndex = 0, columnIndex =0, height = 1, width = 2)
  public void setHoldAngleEnabled(boolean _boolean){
    holdRobotAngleEnabled = _boolean;
  }

  @Log.Dial(name= "Current Robot Angle", min = -180, max = 180, rowIndex = 0, columnIndex =3)
  public double getRobotAngleDegrees(){
    return SwerveMap.getRobotAngle().getDegrees();
  }
  @Log.Dial(name= "Hold Angle Setpoint", min = -180, max = 180, rowIndex = 0, columnIndex =4)
  public double getHoldAngleSetpoint(){
    return Math.toDegrees(holdRobotAngleSetpoint);
  }

  public void setHoldRobotAngleSetpoint(double _holdRobotAngleSetpoint) {
    holdRobotAngleSetpoint = Math.toRadians(_holdRobotAngleSetpoint);
  }

  public void resetOdometry(){
    m_odometry.resetPosition(new Pose2d(), SwerveMap.getRobotAngle());
  }

  public void resetOdometry(Pose2d _Pose2d, Rotation2d _Rotation2d){
    m_odometry.resetPosition(_Pose2d, _Rotation2d);
  }

  public void getVelocities() {
    ChassisSpeeds mySpeeds = m_kinematics.toChassisSpeeds(SwerveMap.FrontLeftSwerveModule.getState(), SwerveMap.FrontRightSwerveModule.getState(), SwerveMap.BackLeftSwerveModule.getState(),SwerveMap.BackRightSwerveModule.getState()) ;
    velocities.set(0, mySpeeds.vxMetersPerSecond);
    velocities.set(1, mySpeeds.vyMetersPerSecond);
  }


 /* @Log.NumberBar(name = "FL Speed", min=-5,max=5 , rowIndex = 2, columnIndex =4, height = 1, width = 1)
  public double getFrontLeftSpeed(){
    return SwerveMap.FrontLeftSwerveModule.getState().speedMetersPerSecond;
  }
  @Log.Dial(name = "FL Angle", min = -90, max = 90, rowIndex = 2, columnIndex =3, height = 1, width = 1)
  public double getFrontLeftAngle(){
    return Math.IEEEremainder(SwerveMap.FrontLeftSwerveModule.getState().angle.getDegrees(),180);
  }

  @Log.NumberBar(name = "FR Speed", min=-5,max=5, rowIndex = 2, columnIndex =7, height = 1, width = 1)
  public double getFrontRightSpeed(){
    return SwerveMap.FrontRightSwerveModule.getState().speedMetersPerSecond;
  }
  @Log.Dial(name = "FR Angle", min = -90, max = 90, rowIndex = 2, columnIndex =8, height = 1, width = 1)
  public double getFrontRightAngle(){
    return Math.IEEEremainder(SwerveMap.FrontRightSwerveModule.getState().angle.getDegrees(),180);
  }

  @Log.NumberBar(name = "BL Speed", min=-5,max=5, rowIndex = 3, columnIndex =4, height = 1, width = 1)
  public double getBackLeftSpeed(){
    return SwerveMap.BackLeftSwerveModule.getState().speedMetersPerSecond;
  }
  @Log.Dial(name = "BL Angle", min = -90, max = 90, rowIndex = 3, columnIndex =3, height = 1, width = 1)
  public double getBackLeftAngle(){
    return Math.IEEEremainder(SwerveMap.BackLeftSwerveModule.getState().angle.getDegrees(),180);
  }


  @Log.NumberBar(name = "BR Speed", min=-5,max=5,  rowIndex =3, columnIndex =7, height = 1, width = 1)
  public double getBackRightSpeed(){
    return SwerveMap.BackRightSwerveModule.getState().speedMetersPerSecond;
  }
  @Log.Dial(name = "BR Angle", min = -90, max = 90, rowIndex =3, columnIndex =8, height = 1, width = 1)
  public double getBackRightAngle(){
    return Math.IEEEremainder(SwerveMap.BackRightSwerveModule.getState().angle.getDegrees(),180);
  }
  */
  @Log(rowIndex = 0, columnIndex = 5, height = 1, width = 1)
  public double getXPos(){
    return m_odometry.getPoseMeters().getX();
  }
  @Log(rowIndex = 0, columnIndex = 6, height = 1, width = 1)
  public double getYPos(){
    return m_odometry.getPoseMeters().getY();
  }
  /*
  @Log.BooleanBox(rowIndex = 1, columnIndex = 5)
  public boolean getGyroInterference(){
    return SwerveMap.GYRO.isMagneticDisturbance();
  }
  @Config.NumberSlider(name="Governor", defaultValue = 1, min = 0, max = 1, rowIndex = 2, columnIndex = 0, height = 1, width = 2)
  public void setJoystickGovernor(double _input){
    joystickDriveGovernor = _input;
  }
*/
  @Config.ToggleButton(name="ResetGyroAndOdometry", defaultValue = false, rowIndex = 3, columnIndex = 0, height = 1, width = 2)
  public void resetGyroAndOdometry(boolean _input){
    if(_input){
    SwerveMap.GYRO.reset();
    holdRobotAngleSetpoint = 0;
    Robot.SWERVEDRIVE.m_odometry.resetPosition(new Pose2d(), SwerveMap.getRobotAngle());
    _input = false;
    SwerveMap.GYRO.setAngleAdjustment(0);
    }
  }

  @Config.ToggleButton(name="RE-Zero Swerve Angle", defaultValue = false, rowIndex = 4, columnIndex = 0, height = 1, width = 2)
  public void reZeroSwerveDrive(boolean _input){
    if(_input){
      SwerveMap.FrontRightSwerveModule.REzeroSwerveAngle();
      SwerveMap.BackRightSwerveModule.REzeroSwerveAngle();
      SwerveMap.FrontLeftSwerveModule.REzeroSwerveAngle();
      SwerveMap.BackLeftSwerveModule.REzeroSwerveAngle();
    _input = false;
    }
  }
  

}
