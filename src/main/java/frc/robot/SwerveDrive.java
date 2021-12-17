package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import io.github.oblarg.oblog.annotations.Log;

import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive {
  
  @Log.Graph
  public static double SDxSpeed=0;
  @Log
  public static double SDySpeed=0;
  @Log
  public static double SDrotation=0;
  @Log
  public static boolean SDfieldRelative=false;

  public static final Translation2d m_frontLeftLocation = new Translation2d(0.155, 0.155);
  public static final Translation2d m_frontRightLocation = new Translation2d(0.155, -0.155);
  public static final Translation2d m_backLeftLocation = new Translation2d(-0.155, 0.155);
  public static final Translation2d m_backRightLocation = new Translation2d(-0.155, -0.155);
  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  //private static final SwerveDriveOdometry m_odometry =  new SwerveDriveOdometry(m_kinematics, new Rotation2d(Math.toRadians(RobotMap.GYRO.getYaw())));
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

      m_kinematics.toSwerveModuleStates(
        _fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed, _ySpeed, _rot, new Rotation2d(
                Math.toRadians(0)))
              : new ChassisSpeeds(_xSpeed, _ySpeed, _rot));

  SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.MAX_SPEED_METERSperSECOND);

  SwerveMap.FrontLeftSwerveModule.setDesiredState(moduleStates[0]);
  SwerveMap.FrontRightSwerveModule.setDesiredState(moduleStates[1]);
  SwerveMap.BackLeftSwerveModule.setDesiredState(moduleStates[2]);
  SwerveMap.BackRightSwerveModule.setDesiredState(moduleStates[3]);
}

public static void swerveTeleop(){
  double x = Robot.xbox.getY(Hand.kLeft);
  double y = Robot.xbox.getX(Hand.kLeft);
  double rot = Robot.xbox.getY(Hand.kRight);

  SDxSpeed = convertToMetersPerSecond(deadband(x))*Constants.SPEED_GOVERNOR;
  SDySpeed = convertToMetersPerSecond(deadband(y))*Constants.SPEED_GOVERNOR;
  SDrotation = convertToRadiansPerSecond(deadband(rot))*Constants.SPEED_GOVERNOR;;
  System.out.println(SDrotation);
  
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
    if(Math.abs(_input)<=.1){
      _input = 0;
    }
    return _input;
}

/** Updates the field relative position of the robot. */

// public void updateOdometry() {
//   m_odometry.update(
//     new Rotation2d(
//         Math.toRadians(RobotMap.GYRO.getYaw())),
//       RobotMap.FrontLeftSwerveModule.getState(),
//       RobotMap.FrontRightSwerveModule.getState(),
//       RobotMap.BackLeftSwerveModule.getState(),
//       RobotMap.BackRightSwerveModule.getState());
// }
}
