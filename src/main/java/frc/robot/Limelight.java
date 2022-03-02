package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { // There are currently no comments explaining how this code works :(
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private static double camAngle; // CRITICAL: get camAngle from servo for limelight
    private static final double camHeight = 0; // CRITICAL: measure height of limelight from the ground

    public void limelightPeriodic() {

    }

    public static class Hub {

        public enum TargetType {
            UNDEFINED, UPPERGOAL
        }

        private static double xDistance;
        private static double yDistance;
        private static double xVelocity;
        private static double yVelocity;
        private static double angle;

        private static boolean valid;

        private static TargetType trackedTargetType = TargetType.UNDEFINED;

        public void reset() {
            xDistance = 0;
            yDistance = 0;
            xVelocity = 0;
            yVelocity = 0;
            angle = 0;
            valid = false;
            trackedTargetType = TargetType.UNDEFINED;
        }

        public static void getPosition(double camAngle, double camHeight) {
            double yAngle = 90 + camAngle + Limelight.getTY();
        }

        // --- Hub Getters ---

        // Gets the x distance from the robot to the hub
        public static double getxDistance() {
            return xDistance;
        }

        // Gets the y distance from the robot to the hub
        public static double getyDistance() {
            return yDistance;
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
        public static double getAngle() {
            return angle;
        }

        // Get whether hub can be detected within FOV of limelight
        public static boolean isValid() {
            return valid;
        }

        // Get whether hub is detected
        public static TargetType getTrackedTargetType() {
            return trackedTargetType;
        }
        
    }

    // ------ Limelight Getters ------

    private static double getEntry(String entry) { // methods for vision processing and LED aren't currently implemented
        return limelight.getEntry(entry).getDouble(0);
    }

    private static void setEntry(String entry, double value) {
        limelight.getEntry(entry).setDouble(value);
    }
    
    public static boolean getTV() {
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
