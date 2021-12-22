// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import io.github.oblarg.oblog.Logger;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final boolean CHARACTERIZE_ROBOT = false;
  public static SwerveDrive SWERVEDRIVE;
  public static SwerveCharacterization SWERVERCHARACTERIZATION;
  public static XboxController xbox= new XboxController(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SwerveMap.GYRO = new AHRS(SPI.Port.kMXP);
    SwerveMap.driveRobotInit();
    SwerveMap.GYRO.reset();
    SWERVEDRIVE = SwerveDrive.getInstance();
    SWERVEDRIVE.init();
    SWERVEDRIVE.zeroSwerveDrive();

    if(CHARACTERIZE_ROBOT){SWERVERCHARACTERIZATION = SwerveCharacterization.getInstance();}
    Logger.configureLoggingAndConfig(this, false);//keep this statement on the BOTTOM of your init
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Logger.updateEntries();
    SWERVEDRIVE.updateOdometry();
  }

  @Override
  public void autonomousInit() {
    if(CHARACTERIZE_ROBOT){SWERVERCHARACTERIZATION.init(true);}
    SWERVEDRIVE.setToBrake();
    SwerveTrajectory.resetTrajectoryStatus();
    

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(CHARACTERIZE_ROBOT){SWERVERCHARACTERIZATION.periodic();}
    SwerveTrajectory.trajectoryRunner(TrajectoryContainer.jonahTrajectory, SWERVEDRIVE.m_odometry, new Rotation2d(Math.toRadians(-SwerveMap.GYRO.getAngle())));
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SWERVEDRIVE.setToBrake();
    if(CHARACTERIZE_ROBOT){SWERVERCHARACTERIZATION.init(true);}
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SWERVEDRIVE.joystickDrive();
    SWERVEDRIVE.drive(
      SWERVEDRIVE.getSDxSpeed(), 
      SWERVEDRIVE.getSDySpeed(), 
      SWERVEDRIVE.getSDRotation(), 
      SWERVEDRIVE.getSDFieldRelative()
      );

   
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if(CHARACTERIZE_ROBOT){SWERVERCHARACTERIZATION.disabled(false);}
      //SwerveCharacterization.init();
    SWERVEDRIVE.zeroSwerveDrive();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    SWERVEDRIVE.setToCoast();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


}