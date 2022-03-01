package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class AutoWaypoints implements Loggable {

    private static AutoWaypoints SINGLE_INSTANCE = new AutoWaypoints();

    public PathPlannerTrajectory fourBallAutoPath;
    public PathPlannerTrajectory twoBallAutoPath;
    public Waypoint[] chosenWaypoints;
    public int currentWaypointNumber = 0;
    private double stateStartTime;
    private double currentX;
    private double currentY;
    private AutoPoses chosenPath;
 
    public static AutoWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() {
        SwerveMap.GYRO.reset();
        chosenPath = AutoPoses.valueOf(Robot.COMPETITIONLOGGER.autoChooser.getSelected().toString());
        // PathPlanner.loadPath("blueAutoTest", 3, 2.5);
        SwerveMap.GYRO.setAngleAdjustment(chosenPath.thisRot);
        Robot.SHOOTER.homocideTheBattery = true;
    }

    public void loadAutoPaths(){
        chooserBuilder();
        fourBallAutoPath = PathPlanner.loadPath("blueAutoTest", 3, 2.5);
        twoBallAutoPath = PathPlanner.loadPath("twoBallAuto", 3, 2.5);
    }

    public void autoPeriodic() {
        currentX = Robot.SWERVEDRIVE.getXPos();
        currentY = Robot.SWERVEDRIVE.getYPos();
        waypointRunner(chosenWaypoints);
    }

    @Log
    boolean StateHasFinished = false;
    @Log
    Boolean StateHasInitialized = false;
    @Log
    String CurrentState = "";
    boolean StartingStateOverride;
    SwerveDriveOdometry a_odometry;

    String _startPoint;
    String CurrentStartPoint;

    
    public SendableChooser<AutoPoses> m_autoChooser = new SendableChooser<>();

    public enum AutoPoses {
        FENDERFOURBALLAUTO(7.80, 1.68, -84.69, SINGLE_INSTANCE.FenderFourBallAutoWPs),
        FENDERTWOBALLAUTO(6.09, 5.23, 43.78, SINGLE_INSTANCE.FenderTwoBallAutoWPs);

        public double thisX;
        public double thisY;
        public double thisRot;
        public Waypoint[] thisWPset;

        AutoPoses(double _x, double _y, double _rot, Waypoint[] _WP){
            thisX = _x;
            thisY = _y;
            thisRot = _rot;
            thisWPset = _WP;

        }
    }

    public void chooserBuilder(){
        //SJV: DESIGN A DEFAULT AUTO WHICH SHOOTS A BALL AND MOVES OFF THE TARMAC
       for (AutoPoses myAutoPose : AutoPoses.values()){
           
        SINGLE_INSTANCE.m_autoChooser.addOption(myAutoPose.toString(), myAutoPose);
       }
    }

    private void intakeBall() {
        //SJV: NEED TO TURN INTAKE OFF WHEN STATE IS FINISHED, NEED TO PROLLY LOWER THE DISTANCE FROM HALF A METER TO LESS (TEST PLEASE)
        if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) < 0.5) {
            Robot.INTAKE.intakeNow = true;

            if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) > 0.5) {
                StateHasFinished = true;
                Robot.INTAKE.intakeNow = false;
            }
        }
    }

    private void shoot() {
       
        if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) < 0.5) {
            Robot.INTAKE.shootNow = true;

            if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) > 0.5) {
                Robot.INTAKE.shootNow = false;
                StateHasFinished = true;
            }
        }
    }

    public void done(){
        //ADDED TO NOT GO OUT OF BOUNDS IN A ARRAY WAYPOINT RUNNER
        //FEEL FREE TO ADD THING TO THE DONE STATE
    }


    //TWO BALL AUTO METHODS HERE
    //SJV: CONVERT THESE TO THE REAL TWOBALL AUTO AND FOLLOW FORMAT ON FOURBALLAUTO
    private void twoBallIntake1() {
        double ballX = 7.65;
        double ballY = 0.60;

        if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
            StateHasFinished = true;
        }
    }

    private void twoBallShoot() {
        double ballX = 7.88;
        double ballY = 2.86;

        if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
            StateHasFinished = true;
        }
    }

    //FOUR BALL AUTO METHODS HERE
    

    private double getDistance(double X1, double Y1, double X2, double Y2) { //just the distance formula - uses current x and y positions
        double distance = Math.sqrt(Math.pow((X2 - X1), 2) + Math.pow((Y2 - Y1), 2));
        return distance;
    }

    public class Waypoint{
        public Runnable action;
        public double posX ;
        public double posY;

        public Waypoint(Runnable _action, double _x, double _y) {
            action = _action;
            posX=_x;
            posY=_y;
        }
    }

    public Waypoint[] FenderTwoBallAutoWPs = new Waypoint[] {
        new Waypoint(SINGLE_INSTANCE::intakeBall, 7.65, .060),
        new Waypoint(SINGLE_INSTANCE::shoot, 7.88, 2.86),
        new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
    };

    public Waypoint[] FenderFourBallAutoWPs = new Waypoint[] {
        new Waypoint(SINGLE_INSTANCE::intakeBall, 7.801, 1.678),
        new Waypoint(SINGLE_INSTANCE::shoot, 7.882, 2.952),
        new Waypoint(SINGLE_INSTANCE::intakeBall, 5.278, 2.014),
        new Waypoint(SINGLE_INSTANCE::intakeBall, 1.273, 1.215),
        new Waypoint(SINGLE_INSTANCE::shoot, 7.581, 3.033),
        new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
    };

    public void waypointRunner(Waypoint[] chosenWaypoints){
        // If we made one round with the state, we have successfully initialized
        if (!StateHasInitialized) {StateHasInitialized = true;}
            chosenWaypoints[currentWaypointNumber].action.run();
        if (StateHasFinished){
            currentWaypointNumber++;
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
    }
}
