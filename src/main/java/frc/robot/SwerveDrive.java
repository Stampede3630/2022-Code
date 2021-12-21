package frc.robot;

import java.lang.System.Logger;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


public class SwerveDrive implements Loggable {
  private static SwerveDrive SINGLE_INSTANCE = new SwerveDrive();
  private double SDxSpeed=0;
  private double SDySpeed=0;
  private double SDrotation=0;
  private boolean SDFieldRelative=true;
 // private ProfiledPIDController holdAngle = new ProfiledPIDController(.1*Constants.MAX_SPEED_RADIANSperSECOND,0.0, 0.0,TrapezoidProfile)

  public String NeutralMode = "Brake";

  public final Translation2d m_frontLeftLocation = new Translation2d(Constants.WHEEL_BASE_METERS/2, Constants.WHEEL_BASE_METERS/2);
  public final Translation2d m_frontRightLocation = new Translation2d(Constants.WHEEL_BASE_METERS/2, -Constants.WHEEL_BASE_METERS/2);
  public final Translation2d m_backLeftLocation = new Translation2d(-Constants.WHEEL_BASE_METERS/2, Constants.WHEEL_BASE_METERS/2);
  public final Translation2d m_backRightLocation = new Translation2d(-Constants.WHEEL_BASE_METERS/2, -Constants.WHEEL_BASE_METERS/2);
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  public SwerveDriveOdometry m_odometry;
  
  public static SwerveDrive getInstance() {
    return SINGLE_INSTANCE;}
    
    {
    m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(Math.toRadians(-SwerveMap.GYRO.getAngle())));
  }
  
  /**
  * Method to drive the robot using joystick info.
  *
  * @param xSpeed Speed of the robot in the x direction (forward).
  * @param ySpeed Speed of the robot in the y direction (sideways).
  * @param rot Angular rate of the robot.
  * @param fieldRelative Whether the provided x and y speeds are relative to the field.
  */
  @SuppressWarnings("ParameterName")
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
      SwerveModuleState[] moduleStates =

      m_kinematics.toSwerveModuleStates( _fieldRelative ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, new Rotation2d(Math.toRadians(-SwerveMap.GYRO.getAngle())))
        : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));

      SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.MAX_SPEED_METERSperSECOND);

      SwerveMap.FrontLeftSwerveModule.setDesiredState(moduleStates[0]);
      SwerveMap.FrontRightSwerveModule.setDesiredState(moduleStates[1]);
      SwerveMap.BackLeftSwerveModule.setDesiredState(moduleStates[2]);
      SwerveMap.BackRightSwerveModule.setDesiredState(moduleStates[3]);
}
public void init(){
  m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(Math.toRadians(-SwerveMap.GYRO.getAngle())));
}
public void joystickDrive(){
  double x = -Robot.xbox.getY(Hand.kLeft);
  double y = -Robot.xbox.getX(Hand.kLeft);
  double rot = Robot.xbox.getX(Hand.kRight);

  SDxSpeed = convertToMetersPerSecond(deadband(x))*Constants.SPEED_GOVERNOR;
  SDySpeed = convertToMetersPerSecond(deadband(y))*Constants.SPEED_GOVERNOR;
  SDrotation = convertToRadiansPerSecond(deadband(rot))*Constants.SPEED_GOVERNOR;
  //System.out.println(SDrotation);
  
}
/**
 * MUST BE ADDED TO PERIODIC (NOT INIT METHODS)
 */
public void setToCoast(){
         
  if (NeutralMode == "Brake" &&
    SwerveMap.FrontLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()  < 100 &&
    SwerveMap.BackLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()   < 100 &&
    SwerveMap.FrontRightSwerveModule.mDriveMotor.getSelectedSensorVelocity() < 100 &&
    SwerveMap.BackRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()  < 100) {
      SwerveMap.FrontRightSwerveModule.swerveDisabledInit();
      SwerveMap.BackRightSwerveModule.swerveDisabledInit();
      SwerveMap.FrontLeftSwerveModule.swerveDisabledInit();
      SwerveMap.BackLeftSwerveModule.swerveDisabledInit();
      NeutralMode = "Coast";
    }

}

public void setToBrake(){
  SwerveMap.FrontRightSwerveModule.swerveEnabledInit();
  SwerveMap.BackRightSwerveModule.swerveEnabledInit();
  SwerveMap.FrontLeftSwerveModule.swerveEnabledInit();
  SwerveMap.BackLeftSwerveModule.swerveEnabledInit();
  NeutralMode = "Brake";
}
@Config
public void zeroSwerveDrive(){
  SDxSpeed = 0;
  SDySpeed = 0;
  SDrotation = 0;
}
private double convertToMetersPerSecond(double _input){
  return _input*Constants.MAX_SPEED_METERSperSECOND;
}

private double convertToRadiansPerSecond(double _input){
  return _input*Constants.MAX_SPEED_RADIANSperSECOND;
}
public double deadband(double _input){
    if(Math.abs(_input)<= Constants.XBOXDEADBAND){
      _input = 0;
    }
    return _input;
}

/** Updates the field relative position of the robot. */

public void updateOdometry() {
  m_odometry.update(
    
  new Rotation2d(Math.toRadians(-SwerveMap.GYRO.getAngle())),
  SwerveMap.FrontLeftSwerveModule.getState(),
  SwerveMap.FrontRightSwerveModule.getState(),
  SwerveMap.BackLeftSwerveModule.getState(),
  SwerveMap.BackRightSwerveModule.getState());
  //System.out.println("x= " + m_odometry.getPoseMeters().getX() + " y="+m_odometry.getPoseMeters().getY());
  //System.out.println("FL: " + Math.round(SwerveMap.FrontLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()) + " FR: " +Math.round(SwerveMap.FrontRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()));
  //System.out.println("BL: " + Math.round(SwerveMap.BackLeftSwerveModule.mDriveMotor.getSelectedSensorVelocity()) + " BR: " +Math.round(SwerveMap.BackRightSwerveModule.mDriveMotor.getSelectedSensorVelocity()));
}


//public double holdAngle (double _input){


@Log 
public double getSDxSpeed() {
  return SDxSpeed;
}
@Log
public double getSDySpeed(){
  return SDySpeed;
}

@Log
public double getSDRotation() {
  return SDrotation;
}

@Log
public boolean getSDFieldRelative() {
  return SDFieldRelative;
}

@Config
public void setSDxSpeed(double _input) {
  SDxSpeed = _input;
}

@Config
public void setSDySpeed(double _input) {
  SDySpeed = _input;
}
@Config.NumberSlider(name = "Testlog")
public void setSDRotation(double _input) {
  SDrotation = _input;
}
@Config.ToggleButton(name = "FieldOrienter?", defaultValue = true)
public void setSDFieldRelative(boolean _input) {
  SDFieldRelative = _input;
}
@Config
public void resetOdometry(){
  m_odometry.resetPosition(new Pose2d(), new Rotation2d(Math.toRadians(-SwerveMap.GYRO.getAngle())));
}
@Log.NumberBar(min=-5,max=5)
public double getFrontLeftXVector(){
  return SwerveMap.FrontLeftSwerveModule.getState().speedMetersPerSecond;
}
@Log.NumberBar(min=-5,max=5)
public double getFrontRightXVector(){
  return SwerveMap.FrontRightSwerveModule.getState().speedMetersPerSecond;
}
@Log.NumberBar(min=-5,max=5)
public double getBackLeftXVector(){
  return SwerveMap.BackLeftSwerveModule.getState().speedMetersPerSecond;
}
@Log.NumberBar(min=-5,max=5)
public double getBackRightXVector(){
  return SwerveMap.BackRightSwerveModule.getState().speedMetersPerSecond;
}
@Log
public double getXPos(){
  return m_odometry.getPoseMeters().getX();
}
@Config.ToggleSwitch(name="ResetGyroAndOdometry", defaultValue = true)
public void resetGyroAndOdometry(boolean _input){
  if(_input){
  SwerveMap.GYRO.reset();
  Robot.SWERVEDRIVE.m_odometry.resetPosition(new Pose2d(), new Rotation2d(Math.toRadians(-SwerveMap.GYRO.getAngle())));
  _input = false;
}
}


}
