package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Loggable;

public class SwerveTrajectory implements Loggable {
    // Create config for trajectory
    public static double timetrajectoryStarted;
    public static String trajectoryStatus="";
    public static SwerveTrajectory SINGLE_INSTANCE = new SwerveTrajectory();    

    public static double elapsedTime;

    public static SwerveTrajectory getInstance(){
        return SINGLE_INSTANCE;
    }
    public static HolonomicDriveController HDC = new HolonomicDriveController(
        new PIDController(Constants.kP,0, 0), 
        new PIDController(Constants.kP, 0, 0), 
        new ProfiledPIDController(Constants.kP*Constants.MAX_SPEED_RADIANSperSECOND/Constants.MAX_SPEED_METERSperSECOND, 0, 0, 
        new Constraints(Constants.MAX_SPEED_RADIANSperSECOND, Constants.MAX_SPEED_RADIANSperSECOND)));
    

    /**This is WPILIBs Trajectory Runner (docs.wpilib.org), it pretends that your robot is NOT a swerve drive.  This will work, but there are better options for 2022
     * @param _trajectory Pass in a trajectory that's stored in TrajectoryContainer
     * @param _odometry Pass in the robots odometry from SwerveDrive.java
     * @param _rotation2d Pass in the current angle of the robot
     */



    //Overload this method to accomdate different starting points, this can be useful when playing with multiple paths
    /**
     * This is PathPlanner.  It's awesome :) open up pathplanner.exe on the driverstation laptop.  Point the application to the locaiton of your coding project (must contain build.gradle).  Draw the path.  It will autosave. If everything is characterized correctly and your odometry reflects reality, ie. when the robot goes 1 meter it says it goes one meter--it will work like a charm.
     * @param _pathTraj run Pathplanner.loadpath("name of file without extension") pass it here
     * @param _odometry SwerveDrive.java's odometry
     * @param _rotation2d Pass in the current angle of the robot
     */
    public static void PathPlannerRunner(PathPlannerTrajectory _pathTraj, SwerveDriveOdometry _odometry, Rotation2d _rotation2d){
        elapsedTime = Timer.getFPGATimestamp()-timetrajectoryStarted;
        switch (trajectoryStatus) {
            case "setup":    
            PathPlannerState helloPath =((PathPlannerState)_pathTraj.getInitialState());
                Robot.SWERVEDRIVE.resetOdometry(helloPath.poseMeters, helloPath.poseMeters.getRotation()); 
                timetrajectoryStarted = Timer.getFPGATimestamp();
                trajectoryStatus = "execute";
                break;
            case "execute":
                
                if (elapsedTime <  ((PathPlannerState) _pathTraj.getEndState()).timeSeconds){
                    ChassisSpeeds _speeds = HDC.calculate(
                        _odometry.getPoseMeters(), 
                        ((PathPlannerState) _pathTraj.sample(elapsedTime)),((PathPlannerState) _pathTraj.sample(elapsedTime)).holonomicRotation);
                    Robot.SWERVEDRIVE.drive(_speeds.vxMetersPerSecond,
                    _speeds.vyMetersPerSecond, 
                    _speeds.omegaRadiansPerSecond,false);
                    
                } else {
                    Robot.SWERVEDRIVE.drive(0,0,0,false);
                    Robot.SWERVEDRIVE.setHoldRobotAngleSetpoint(((PathPlannerState) _pathTraj.getEndState()).holonomicRotation.getRadians());
                    trajectoryStatus = "done";

                }
                break;
            default:
                Robot.SWERVEDRIVE.drive(0,0,0,false);
                break;
        }
    }

    public static void resetTrajectoryStatus(){
        trajectoryStatus = "setup";
    }
    
}
