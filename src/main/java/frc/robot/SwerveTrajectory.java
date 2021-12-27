package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import frc.robot.PathPlannerTrajectory.PathPlannerState;
import io.github.oblarg.oblog.Loggable;


public class SwerveTrajectory implements Loggable {
    // Create config for trajectory
    private static double timetrajectoryStarted;
    private static String trajectoryStatus="";
    
    
    public static TrajectoryConfig  config = new TrajectoryConfig(Constants.MAX_SPEED_METERSperSECOND*.1, Constants.MAX_SPEED_METERSperSECOND);
    
    public static HolonomicDriveController HDC = new HolonomicDriveController(
        new PIDController(.69,0, 0), 
        new PIDController(.69, 0, 0), 
        new ProfiledPIDController(.69/Constants.MAX_SPEED_RADIANSperSECOND*Constants.MAX_SPEED_METERSperSECOND, 0, 0, 
        new Constraints(Constants.MAX_SPEED_RADIANSperSECOND*.5, Constants.MAX_SPEED_RADIANSperSECOND*.5)));
    
    public static void trajectoryRunner(Trajectory _trajectory, SwerveDriveOdometry _odometry, Rotation2d _rotation2d){
        double elapsedTime = Timer.getFPGATimestamp()-timetrajectoryStarted;
        switch (trajectoryStatus) {
            case "setup":
                Robot.SWERVEDRIVE.resetGyroAndOdometry(true); 
                timetrajectoryStarted = Timer.getFPGATimestamp();
                trajectoryStatus = "execute";
                break;
            case "execute":
                if (elapsedTime <_trajectory.getTotalTimeSeconds()){
                    ChassisSpeeds _speeds = HDC.calculate(
                        _odometry.getPoseMeters(), 
                        _trajectory.sample(elapsedTime),_rotation2d);
                    Robot.SWERVEDRIVE.drive(_speeds.vxMetersPerSecond,
                    _speeds.vyMetersPerSecond, 
                    _speeds.omegaRadiansPerSecond,false);
                    
                } else {
                    Robot.SWERVEDRIVE.drive(0,0,0,false);
                    trajectoryStatus = "done";
                }
                break;
            default:
                break;
        }
    }

    public static void PathPlannerRunner(PathPlannerTrajectory _pathTraj, SwerveDriveOdometry _odometry, Rotation2d _rotation2d){
        double elapsedTime = Timer.getFPGATimestamp()-timetrajectoryStarted;
        switch (trajectoryStatus) {
            case "setup":
                Robot.SWERVEDRIVE.resetOdometry(((PathPlannerState) _pathTraj.getInitialState()).poseMeters, ((PathPlannerState) _pathTraj.getInitialState()).poseMeters.getRotation()); 
                timetrajectoryStarted = Timer.getFPGATimestamp();
                trajectoryStatus = "execute";
                break;
            case "execute":
                if (elapsedTime <  ((PathPlannerState) _pathTraj.getEndState()).timeSeconds){
                    ChassisSpeeds _speeds = HDC.calculate(
                        _odometry.getPoseMeters(), 
                        ((PathPlannerState) _pathTraj.sample(elapsedTime)),_rotation2d);
                    Robot.SWERVEDRIVE.drive(_speeds.vxMetersPerSecond,
                    _speeds.vyMetersPerSecond, 
                    _speeds.omegaRadiansPerSecond,false);
                    
                } else {
                    Robot.SWERVEDRIVE.drive(0,0,0,false);
                    trajectoryStatus = "done";
                }
                break;
            default:
                break;
        }
    }

    public static void resetTrajectoryStatus(){
        trajectoryStatus = "setup";
    }
    
}
