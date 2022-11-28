// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.Arrays;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.swerve.SwerveConstants;
import frc.robot.sim.SimGyroSensorModel;
import frc.robot.sim.wpiClasses.SwerveModuleSim;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveMap;
import frc.robot.swerve.SwerveTrajectory;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.RobotBase;


 /*For starting a new Stampede swerve project
  * 1. Zero out the following constants: ks, kv
  * 2. From the top of the Constants Class to the PID gains fill in as much as possible
  *    using Phoenix Tuner
  * 3. Recalibrate NAVX Gyro
  * 4. Until you characterize the robot, ur not gonna want to run trajectories
  *    so probably, turn RUN_TRAJECTORY FALSE
         */
public class Robot extends TimedRobot {
  public static SwerveDrive SWERVEDRIVE;
  public static Intake INTAKE;
  public static Shooter SHOOTER;
  public static Climber CLIMBER;
  public static double deltaTime = 0;
  public static double prevTime = 0;

  public static double myWattThingy;
  // public static AutoWaypoints AUTOWAYPOINTS;
  public static SwerveTrajectory SWERVETRAJECTORY;
  public static CompetitionLogger COMPETITIONLOGGER;
  public static AutoSegmentedWaypoints AUTOSEGMENTEDWAYPOINTS;
  public static XboxController xbox = new XboxController(0);

  @Log
  public static Field2d field = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   // --- INIT METHOD ---
  @Override
  public void robotInit() {
    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();
    SwerveMap.GYRO = new AHRS(SPI.Port.kMXP);
    SwerveMap.GYRO.reset(); 
    
    SwerveMap.checkAndSetSwerveCANStatus();

    //**Intake method starts here**
    INTAKE = Intake.getInstance();
    INTAKE.init();
    INTAKE.checkAndSetIntakeCANStatus();

    //*** Auto Container method starts here***
    // AUTOWAYPOINTS = AutoWaypoints.getInstance();
    AUTOSEGMENTEDWAYPOINTS = AutoSegmentedWaypoints.getInstance();
    // 
    //loads the selected pathplanner path
    SwerveMap.driveRobotInit();
    // AUTOWAYPOINTS.loadAutoPaths();
    AUTOSEGMENTEDWAYPOINTS.loadAutoPaths();

    // ****Shooter method starts here****
    SHOOTER = Shooter.getInstance();
    SHOOTER.init();
    SHOOTER.checkAndSetShooterCANStatus();

    // // *****test climber method starts here*****
    CLIMBER = Climber.getInstance();
    CLIMBER.init();
    CLIMBER.checkAndSetClimberCANStatus();

    SWERVETRAJECTORY = SwerveTrajectory.getInstance();
    // examplePath = PathPlanner.loadPath("New Path", 1, .8);
    // Keep this statement on the BOTTOM of your robotInit
    // It's responsible for all the shuffleboard outputs.  
    // It's a lot easier to use than standard shuffleboard syntax
 
    // we do singleton methodologies to allow the shuffleboard (Oblarg) logger to detect the existence of these. #askSam

    //*Swerve method starts here*
    SWERVEDRIVE = SwerveDrive.getInstance();
    SWERVEDRIVE.init();
    SWERVEDRIVE.zeroSwerveDrive();

    NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("camMode").setNumber(1);

    COMPETITIONLOGGER = CompetitionLogger.getInstance();
    Logger.setCycleWarningsEnabled(false);
    Logger.configureLoggingAndConfig(this, false);
    
    field.getObject("target").setPose(new Pose2d(4, 4, new Rotation2d()));

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
    deltaTime = Timer.getFPGATimestamp() - prevTime;
    prevTime = Timer.getFPGATimestamp();
    SWERVEDRIVE.updateOdometry();
    SwerveMap.checkAndSetSwerveCANStatus();
    SwerveMap.checkAndZeroSwerveAngle();
    INTAKE.checkAndSetIntakeCANStatus();
    SHOOTER.checkAndSetShooterCANStatus();
    CLIMBER.checkAndSetClimberCANStatus();
    Logger.updateEntries();

    myWattThingy =  myWattThingy + (COMPETITIONLOGGER.getMyPD() * COMPETITIONLOGGER.batteryVoltage()) / 0.02;
    SWERVEDRIVE.drawRobotOnField(field);
  }
  

  @Override
  public void autonomousInit() {
    SWERVEDRIVE.disableCurrentLimiting();
    SWERVEDRIVE.setToBrake();
    // AUTOWAYPOINTS.init();
    AUTOSEGMENTEDWAYPOINTS.init();
     

    // For Trajectory instructions go to SwerverTrajectory.java
    SwerveTrajectory.resetTrajectoryStatus();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    AUTOSEGMENTEDWAYPOINTS.autoPeriodic();
    INTAKE.intakePeriodic();
    SHOOTER.shooterPeriodic();

  }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SWERVEDRIVE.enableCurrentLimiting();
    SWERVEDRIVE.setToBrake();
    INTAKE.intakeNow = false;
    INTAKE.shootNow = false;
    // SHOOTER.homocideTheBattery = false;
    SWERVEDRIVE.autoLimeLightAim = false;
    SHOOTER.homocideTheBattery = true;
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Joystick Drives stores values in X,Y,Z rotation
    // Drive actually sends those values to the swerve modules

    SWERVEDRIVE.swervePeriodic();
    CLIMBER.periodic();
    INTAKE.intakePeriodic();
    // SHOOTER INSTANCE LOOP
    SHOOTER.shooterPeriodic();
    
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

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {
    SWERVEDRIVE.simulationPeriodic();
  }



  


} 