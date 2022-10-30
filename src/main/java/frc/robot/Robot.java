// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.Arrays;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.swerve.SwerveConstants;
import frc.robot.sim.SimGyroSensorModel;
import frc.robot.sim.wpiClasses.QuadSwerveSim;
import frc.robot.sim.wpiClasses.SwerveModuleSim;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveMap;
import frc.robot.swerve.SwerveTrajectory;
import io.github.oblarg.oblog.Logger;
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
  public static final boolean CHARACTERIZE_ROBOT = true;
  public static final boolean RUN_TRAJECTORY = true;

  public static SwerveDrive SWERVEDRIVE;
  public static Intake INTAKE;
  public static Shooter SHOOTER;
  public static Climber CLIMBER;
  public static double myWattThingy;
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
    if (RobotBase.isReal()) {
      SwerveMap.GYRO = new AHRS(SPI.Port.kMXP);
    } else {
      SwerveMap.simNavx = new SimGyroSensorModel();
    }
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

    myWattThingy =  myWattThingy + (COMPETITIONLOGGER.getMyPD() * COMPETITIONLOGGER.batteryVoltage()) / 0.02;
    
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
      //intake code for teleop

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

    if(!DriverStation.isEnabled()){
      for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
        swerveModuleSimList.get(idx).setInputVoltages(0.0, 0.0);
      }
    } else {
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            double azmthVolts = SwerveMap.RealSwerveModuleList.get(idx).mSteeringMotor.getMotorOutputVoltage();
            double wheelVolts = SwerveMap.RealSwerveModuleList.get(idx).mDriveMotor.getMotorOutputVoltage();
            swerveModuleSimList.get(idx).setInputVoltages(wheelVolts, azmthVolts);
        }
    }

    Pose2d prevRobotPose = simSwerve.getCurPose();

    // Update model (several small steps)
    for (int i = 0; i< 20; i++) {
      simSwerve.update(0.001);
    }
    
        //Set the state of the sim'd hardware
    for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
      double azmthPos = swerveModuleSimList.get(idx).getAzimuthEncoderPositionRev();
      azmthPos = azmthPos / SwerveConstants.TICKSperTALONFX_STEERING_DEGREE * 2 * Math.PI;
      double wheelPos = swerveModuleSimList.get(idx).getWheelEncoderPositionRev();
      wheelPos = wheelPos / SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION * 2 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS;

      double wheelVel = swerveModuleSimList.get(idx).getWheelEncoderVelocityRevPerSec();
      wheelVel = wheelVel / SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION * 2 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS;
      SwerveMap.RealSwerveModuleList.get(idx).setSimState(azmthPos, wheelPos, wheelVel);
      SwerveMap.simNavx.update(simSwerve.getCurPose(), prevRobotPose);
    }


  }

  static List<SwerveModuleSim> swerveModuleSimList = List.of(new SwerveModuleSim(DCMotor.getFalcon500(1), 
  DCMotor.getFalcon500(1), 
  SwerveConstants.WHEEL_RADIUS_METERS,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.3,
  0.7,
  56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
  0.01 
  ), new SwerveModuleSim(DCMotor.getFalcon500(1), 
  DCMotor.getFalcon500(1), 
  SwerveConstants.WHEEL_RADIUS_METERS,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.3,
  0.7,
  56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
  0.01 
  ), new SwerveModuleSim(DCMotor.getFalcon500(1), 
  DCMotor.getFalcon500(1), 
  SwerveConstants.WHEEL_RADIUS_METERS,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.3,
  0.7,
  56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
  0.01 
  ), new SwerveModuleSim(DCMotor.getFalcon500(1), 
  DCMotor.getFalcon500(1), 
  SwerveConstants.WHEEL_RADIUS_METERS,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // steering motor rotations per wheel steer rotation
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.0/SwerveConstants.STEERING_MOTOR_GEARING, // same as motor rotations because NEO encoder is on motor shaft
  1.0/SwerveConstants.DRIVE_MOTOR_GEARING,
  1.3,
  0.7,
  56.6 * 9.81 / QuadSwerveSim.NUM_MODULES, 
  0.01 
  ));
public static QuadSwerveSim simSwerve = new QuadSwerveSim(SwerveConstants.TRACK_WIDE, 
  SwerveConstants.WHEEL_BASE_METERS, 
  56.6, 1.0/12.0 * 56.6 * Math.pow((SwerveConstants.TRACK_WIDE*1.1),2) * 2, swerveModuleSimList);


} 