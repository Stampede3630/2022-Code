package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class TrajectoryContainer {
    private static TrajectoryConfig  config = new TrajectoryConfig(Constants.MAX_SPEED_METERSperSECOND*Constants.SPEED_GOVERNOR, Constants.MAX_SPEED_METERSperSECOND);


        public static Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(.5, -.1),
                new Translation2d(1, .1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.5, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        public static PathPlannerTrajectory heteroPath = PathPlanner.loadPath("autoTest 2022", 6, 2.5);

        public static Trajectory jonahTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 0),
                new Translation2d(1.2, 0),
                new Translation2d(1.5, 0),
                new Translation2d(2, 0),
                new Translation2d(2.5, 0),
                new Translation2d(2.7, 0),
                new Translation2d(3.2, 0),
                new Translation2d(3.5, 0.0),
                new Translation2d(3.5, 0.0),
                new Translation2d(3.6, 0.0),
                new Translation2d(4.1, .0),
                new Translation2d(4.3, .0),
                new Translation2d(4.5, .0),
                new Translation2d(4.6, .0),
                new Translation2d(4.7, -0.1)
            ),
            new Pose2d(5, .25, new Rotation2d(0)),
            // Pass config
            config
        );
    
}
