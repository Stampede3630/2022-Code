package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


public class SwerveDrive implements Loggable {
  
  @Log
  public static double SDxSpeed=0;
  @Log
  public static double SDySpeed=0;
  @Log
  public static double SDrotation=0;
  @Log
  public static boolean SDfieldRelative=false;

  public static String NeutralMode = "Brake";

  public static final Translation2d m_frontLeftLocation = new Translation2d(Constants.WHEEL_BASE_METERS/2, Constants.WHEEL_BASE_METERS/2);
  public static final Translation2d m_frontRightLocation = new Translation2d(Constants.WHEEL_BASE_METERS/2, -Constants.WHEEL_BASE_METERS/2);
  public static final Translation2d m_backLeftLocation = new Translation2d(-Constants.WHEEL_BASE_METERS/2, Constants.WHEEL_BASE_METERS/2);
  public static final Translation2d m_backRightLocation = new Translation2d(-Constants.WHEEL_BASE_METERS/2, -Constants.WHEEL_BASE_METERS/2);
  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  private static SwerveDriveOdometry m_odometry;
  
  public SwerveDrive(){
    m_odometry = new SwerveDriveOdometry(m_kinematics, SwerveMap.GYRO.getRotation2d());
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
  public static void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative) {
      SwerveModuleState[] moduleStates =

      m_kinematics.toSwerveModuleStates( _fieldRelative ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, SwerveMap.GYRO.getRotation2d())
        : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));

      SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.MAX_SPEED_METERSperSECOND);

      SwerveMap.FrontLeftSwerveModule.setDesiredState(moduleStates[0]);
      SwerveMap.FrontRightSwerveModule.setDesiredState(moduleStates[1]);
      SwerveMap.BackLeftSwerveModule.setDesiredState(moduleStates[2]);
      SwerveMap.BackRightSwerveModule.setDesiredState(moduleStates[3]);
}

public static void joystickDrive(){
  double x = Robot.xbox.getY(Hand.kLeft);
  double y = Robot.xbox.getX(Hand.kLeft);
  double rot = Robot.xbox.getX(Hand.kRight);

  SDxSpeed = convertToMetersPerSecond(deadband(x))*Constants.SPEED_GOVERNOR;
  SDySpeed = convertToMetersPerSecond(deadband(y))*Constants.SPEED_GOVERNOR;
  SDrotation = convertToRadiansPerSecond(deadband(rot))*Constants.SPEED_GOVERNOR*.05;
  System.out.println(SDrotation);
  
}
/**
 * MUST BE ADDED TO PERIODIC (NOT INIT METHODS)
 */
public static void setToCoast(){
         
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

public static void setToBrake(){
  SwerveMap.FrontRightSwerveModule.swerveEnabledInit();
  SwerveMap.BackRightSwerveModule.swerveEnabledInit();
  SwerveMap.FrontLeftSwerveModule.swerveEnabledInit();
  SwerveMap.BackLeftSwerveModule.swerveEnabledInit();
  NeutralMode = "Brake";
}

public static void zeroSwerveDrive(){
  SDxSpeed = 0;
  SDySpeed = 0;
  SDrotation = 0;
}
private static double convertToMetersPerSecond(double _input){
  return _input*Constants.MAX_SPEED_METERSperSECOND;
}

private static double convertToRadiansPerSecond(double _input){
  return _input*Constants.MAX_SPEED_RADIANSperSECOND;
}
public static double deadband(double _input){
    if(Math.abs(_input)<= Constants.XBOXDEADBAND){
      _input = 0;
    }
    return _input;
}

/** Updates the field relative position of the robot. */

public void updateOdometry() {
  m_odometry.update(
    
  SwerveMap.GYRO.getRotation2d(),
  SwerveMap.FrontLeftSwerveModule.getState(),
  SwerveMap.FrontRightSwerveModule.getState(),
  SwerveMap.BackLeftSwerveModule.getState(),
  SwerveMap.BackRightSwerveModule.getState());
}


@Log 
public static double getSDxSpeed() {
  return SDxSpeed;
}
@Log
public static double getSDYSpeed(){
  return SDySpeed;
}

@Log
public static double getSDRotation() {
  return SDrotation;
}

@Config
public static void setSDxSpeed(double _input) {
  SDxSpeed = _input;
}

@Config
public static void setSDySpeed(double _input) {
  SDySpeed = _input;
}
@Config.NumberSlider(name = "Testlog")
public static void setSDxRotation(double _input) {
  SDrotation = _input;
}
}
