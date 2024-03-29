// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

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
  public static Intake INTAKE;
  public static Shooter SHOOTER;
  public static Climber CLIMBER;
  public static double myWattThingy = 0;
  public static double myJuulPod = 0;
  public double previousTimestamp;
  // public static AutoWaypoints AUTOWAYPOINTS;
  public static SwerveTrajectory SWERVETRAJECTORY;
  public static CompetitionLogger COMPETITIONLOGGER;
  public static AutoSegmentedWaypoints AUTOSEGMENTEDWAYPOINTS;
  public static XboxController xbox = new XboxController(0);
  public static GenericHID ddrPad = new GenericHID(1);
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
    SwerveMap.checkAndSetSwerveCANStatus();

    previousTimestamp = Timer.getFPGATimestamp();

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


    if(RUN_TRAJECTORY) {
    SWERVETRAJECTORY = SwerveTrajectory.getInstance();
      // examplePath = PathPlanner.loadPath("New Path", 1, .8);
    }
    // Keep this statement on the BOTTOM of your robotInit
    // It's responsible for all the shuffleboard outputs.  
    // It's a lot easier to use than standard shuffleboard syntax
 
    
    SwerveMap.GYRO.reset(); 
    // we do singleton methodologies to allow the shuffleboard (Oblarg) logger to detect the existence of these. #askSam

    //*Swerve method starts here*
    SWERVEDRIVE = SwerveDrive.getInstance();
    SWERVEDRIVE.init();
    SWERVEDRIVE.zeroSwerveDrive();


    NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("camMode").setNumber(1);

    COMPETITIONLOGGER = CompetitionLogger.getInstance();
    Logger.setCycleWarningsEnabled(false);
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
    SwerveMap.checkAndSetSwerveCANStatus();
    SwerveMap.checkAndZeroSwerveAngle();
    INTAKE.checkAndSetIntakeCANStatus();
    SHOOTER.checkAndSetShooterCANStatus();
    CLIMBER.checkAndSetClimberCANStatus();
    Logger.updateEntries();

    
    myWattThingy = myWattThingy + (COMPETITIONLOGGER.myPD.getTotalCurrent() * COMPETITIONLOGGER.batteryVoltage()) * (Timer.getFPGATimestamp() - previousTimestamp);

    myJuulPod = myWattThingy/1000;

    // System.out.println(myJuulPod);

    // System.out.println(COMPETITIONLOGGER.myPD.getTotalCurrent());
    
    previousTimestamp = Timer.getFPGATimestamp();
  }
  

  @Override
  public void autonomousInit() {
    SWERVEDRIVE.disableCurrentLimiting();
    SWERVEDRIVE.setToBrake();
    // AUTOWAYPOINTS.init();
    AUTOSEGMENTEDWAYPOINTS.init();
     

    // For Trajectory instructions go to SwerverTrajectory.java
    if(RUN_TRAJECTORY) {SwerveTrajectory.resetTrajectoryStatus();}

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    // AUTOWAYPOINTS.autoPeriodic();
    // SwerveTrajectory.PathPlannerRunner(AUTOWAYPOINTS.chosenPath.thisPathPLan,  SWERVEDRIVE.m_odometry, SwerveMap.getRobotAngle());

    AUTOSEGMENTEDWAYPOINTS.autoPeriodic();
    
    INTAKE.intakePeriodic();
    SHOOTER.shooterPeriodic();

  }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // SWERVEDRIVE.enableCurrentLimiting();
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
    //intake code for teleop
    SHOOTER.shooterPeriodic();
    // SHOOTER INSTANCE LOOP
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

  @Log (rowIndex = 3, columnIndex = 7, tabName = "CompetitionLogger")
  public double getMyPD() {
      return Robot.myWattThingy;
  }

} 