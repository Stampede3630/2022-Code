// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import io.github.oblarg.oblog.Logger;

 /*For starting a new Stampede swerve project
  * 1. Zero out the following constants: ks, kv
  * 2. From the top of the Constants Class to the PID gains fill in as much as possible
  *    using Phoenix Tuner
  * 3. Recalibrate NAVX Gyro
  * 4. Until you characterize the robot, ur not gonna want to run trajectories
  *    so probably, turn RUN_TRAJECTORY FALSE
         */
public class Robot extends TimedRobot {
  public static final boolean CHARACTERIZE_ROBOT = true;
  public static final boolean RUN_TRAJECTORY = true;

  public static SwerveDrive SWERVEDRIVE;
  public static AutoContainer AUTOCONTAINER;
  public static Intake INTAKE;
  public static Shooter SHOOTER;
  public static Climber CLIMBER;
  public static AutoWaypoints AUTOWAYPOINTS;
  public static SwerveTrajectory SWERVETRAJECTORY;

  public static XboxController xbox = new XboxController(0);

  // public static PathPlannerTrajectory examplePath; 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
     SwerveMap.GYRO = new AHRS(SPI.Port.kMXP);
     SwerveMap.driveRobotInit();
     SwerveMap.GYRO.reset();
    // we do singleton methodologies to allow the shuffleboard (Oblarg) logger to detect the existence of these. #askSam

    //*Swerve method starts here*
     SWERVEDRIVE = SwerveDrive.getInstance();
     SWERVEDRIVE.init();
     SWERVEDRIVE.zeroSwerveDrive();

    //**Intake method starts here**
    // INTAKE = Intake.getInstance();
    // INTAKE.init();

    //*** Auto Container method starts here***
    AUTOCONTAINER = AutoContainer.getInstance();


    // ****Shooter method starts here****
    SHOOTER = Shooter.getInstance();
    SHOOTER.init();

    // // *****test climber method starts here*****
    // CLIMBER = Climber.getInstance();
    // CLIMBER.init();



    // if(RUN_TRAJECTORY) {
      // SWERVETRAJECTORY = SwerveTrajectory.getInstance();
      // examplePath = PathPlanner.loadPath("New Path", 1, .8);
    // }
    // Keep this statement on the BOTTOM of your robotInit
    // It's responsible for all the shuffleboard outputs.  
    // It's a lot easier to use than standard shuffleboard syntax
    Logger.configureLoggingAndConfig(this, false);
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
     SWERVEDRIVE.updateOdometry();
    Logger.updateEntries();
  }

  @Override
  public void autonomousInit() {
     SWERVEDRIVE.setToBrake();

    // For Trajectory instructions go to SwerverTrajectory.java
     if(RUN_TRAJECTORY) {SwerveTrajectory.resetTrajectoryStatus();}

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    AUTOCONTAINER.autoRunner("");
    if(RUN_TRAJECTORY){
    SwerveTrajectory.PathPlannerRunner(AUTOWAYPOINTS.fourBallAutoPath, SWERVEDRIVE.m_odometry, SwerveMap.getRobotAngle());
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SWERVEDRIVE.setToBrake();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Joystick Drives stores values in X,Y,Z rotation
    // Drive actually sends those values to the swerve modules
    SWERVEDRIVE.swervePeriodic();
      //intake code for teleop

    CLIMBER.periodic();

    INTAKE.intakePeriodic();
    // SHOOTER INSTANCE LOOP
    SHOOTER.shoot();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
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
  public void testPeriodic() {
  }


}