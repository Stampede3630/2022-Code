package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { // There are currently no comments explaining how this code works :(

    private static Limelight SINGLE_INSTANCE = new Limelight();

    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static final double CAM_HEIGHT = 0; // TODO: measure height of limelight from the ground
    private static final double X_FOV = Constants.HORIZONTAL_FOV;
    private static final double Y_FOV = Constants.VERTICAL_FOV;
    
    private static double camAngle; // TODO: set this to periodically get current rotation for limelight

    public boolean isLimelightOpen = false;

    public static Limelight getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() {
        Hub.reset();
    }

    // methods for vision processing and LED aren't currently implemented
    public void limelightPeriodic() {
        // Put methods you want in here
    }

    public static class Hub {
        private static final double HEIGHT = Constants.UPPER_GOAL_HEIGHT;

        private static double xDistance; // Horizontal distance from robot to hub
        private static double xVelocity;
        private static double yVelocity;
        // private static double angle; <--- probably don't need this, but don't delete

        private static boolean validTarget;

        public static void lockOnUpperGoal() {
            if (validTarget) {
                xDistance = (HEIGHT - CAM_HEIGHT) / Math.tan(camAngle);
            }

            // TODO: set x and y velocity of hub in relation to robot?
        }

        // TODO: add a function that validates whether target can be found based on FOV of limelight? ---> just in case we solely target hub/ball
        private static boolean isTargetValid() {
            // *** Placeholder Code ***
            return false;
        }

        private static void reset() {
            xDistance = 0;
            xVelocity = 0;
            yVelocity = 0;
            validTarget = false;
        }
   
        // --- Hub Getters ---

        // Gets the x distance from the robot to the hub
        public static double getxDistance() {
            return xDistance;
        }

        // Get the x velocity from the robot to the hub (doesn't take into account robot speed)
        public static double getxVelocity() {
            return xVelocity;
        }

        // Get the y velocity from the robot to the hub (doesn't take into account robot speed)
        public static double getyVelocity() {
            return yVelocity;
        }

        // Get the angle from the robot to the hub
        // public static double getAngle() {
        //     return angle;
        // }

        // Get whether hub can be detected within FOV of limelight
        public static boolean isValid() {
            return validTarget;
        }
        
    }

    // ------ Limelight Getters ------

    private static double getEntry(String entry) {
        return limelight.getEntry(entry).getDouble(0);
    }

    private static void setEntry(String entry, double value) {
        limelight.getEntry(entry).setDouble(value);
    }
    
    public static boolean isTargetValid() {
        return getEntry("tv") == 1;
    }

    public static double getTX() {
        return getEntry("tx");
    }

    public static double getTY() {
        return getEntry("ty");
    }

    public static double getTA() {
        return getEntry("ta");
    }

    public static double getTS() {
        return getEntry("ts");
    }

    public static double getTL() {
        return getEntry("tl");
    }

    public static double getTShort() {
        return getEntry("tshort");
    }

    public static double getTLong() {
        return getEntry("tlong");
    }

    public static double getTHor() {
        return getEntry("thor");
    }

    public static double getTVert() {
        return getEntry("tvert");
    }

    public static double getPipeLine() {
        return getEntry("getPipe");
    }

    public static void setPipeLine(int pipeline) {
        setEntry("pipeline", pipeline);
    }
}
