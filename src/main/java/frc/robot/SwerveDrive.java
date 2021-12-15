package frc.robot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;


public class SwerveDrive {
    private static final Translation2d m_frontLeftLocation = new Translation2d(0.155, 0.155);
    private static final Translation2d m_frontRightLocation = new Translation2d(0.155, -0.155);
    private static final Translation2d m_backLeftLocation = new Translation2d(-0.155, 0.155);
    private static final Translation2d m_backRightLocation = new Translation2d(-0.155, -0.155);
    private static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    private static final SwerveDriveOdometry m_odometry =  new SwerveDriveOdometry(m_kinematics, new Rotation2d(
            Math.toRadians(RobotMap.GYRO.getAngle())));
   /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public static void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      SwerveModuleState[] moduleStates =

      m_kinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(
                Math.toRadians(RobotMap.GYRO.getAngle())))
              : new ChassisSpeeds(xSpeed, ySpeed, rot));

  SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.MAX_SPEED);

  RobotMap.FrontLeftSwerveModule.setDesiredState(moduleStates[0]);
  RobotMap.FrontRightSwerveModule.setDesiredState(moduleStates[1]);
  RobotMap.BackLeftSwerveModule.setDesiredState(moduleStates[2]);
  RobotMap.BackRightSwerveModule.setDesiredState(moduleStates[3]);
}
/** Updates the field relative position of the robot. */

public void updateOdometry() {
  m_odometry.update(
    new Rotation2d(
        Math.toRadians(RobotMap.GYRO.getAngle())),
      RobotMap.FrontLeftSwerveModule.getState(),
      RobotMap.FrontRightSwerveModule.getState(),
      RobotMap.BackLeftSwerveModule.getState(),
      RobotMap.BackRightSwerveModule.getState());
}
}
