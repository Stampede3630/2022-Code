package frc.robot;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class AutoWaypoints implements Loggable {

    private static AutoWaypoints SINGLE_INSTANCE = new AutoWaypoints();
    public Waypoint[] FenderTwoBallAutoWPs;
    public Waypoint[] FenderFourBallAutoWPs; 
    public PathPlannerTrajectory fourBallAutoPath;
    public PathPlannerTrajectory twoBallAutoPath;
    public Waypoint[] chosenWaypoints;
    public int currentWaypointNumber = 0;
    private double currentX;
    private double currentY;
    public AutoPoses chosenPath;

    @Log
    public boolean StateHasFinished = false;
    @Log
    public Boolean StateHasInitialized = false;
    @Log(tabName = "CompetitionLogger")
    public double distance = 0;
    
    @Log(tabName = "CompetitionLogger")
    public SendableChooser<AutoPoses> m_autoChooser = new SendableChooser<>();
 
    public static AutoWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() {
        SwerveMap.GYRO.reset();
        chosenPath = AutoPoses.valueOf(m_autoChooser.getSelected().toString());
        chosenWaypoints = chosenPath.thisWPset;
        SwerveMap.GYRO.setAngleAdjustment(chosenPath.thisRot);
        Robot.SHOOTER.homocideTheBattery = true;
    }

    public void loadAutoPaths(){
       FenderTwoBallAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::intakeBall, 7.65, .060),
            new Waypoint(SINGLE_INSTANCE::shoot, 7.88, 2.86),
            new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
        };
    
        FenderFourBallAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::intakeBall, 7.801, 1.678),
            new Waypoint(SINGLE_INSTANCE::shoot, 7.882, 2.952),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 5.278, 2.014),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 1.273, 1.215),
            new Waypoint(SINGLE_INSTANCE::shoot, 7.581, 3.033),
            new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
        };
        chooserBuilder();
        fourBallAutoPath = PathPlanner.loadPath("blueAutoTest", 3, 2.5);
        twoBallAutoPath = PathPlanner.loadPath("twoBallAuto", 3, 2.5);
    }

    public void autoPeriodic() {
        currentX = Robot.SWERVEDRIVE.getXPos();
        currentY = Robot.SWERVEDRIVE.getYPos();
        waypointRunner(chosenWaypoints);
    }

    public enum AutoPoses {
        FENDERFOURBALLAUTO(7.80, 1.68, -84.69, SINGLE_INSTANCE.FenderFourBallAutoWPs,SINGLE_INSTANCE.fourBallAutoPath),
        FENDERTWOBALLAUTO(6.09, 5.23, 43.78, SINGLE_INSTANCE.FenderTwoBallAutoWPs, SINGLE_INSTANCE.twoBallAutoPath);

        public double thisX;
        public double thisY;
        public double thisRot;
        public Waypoint[] thisWPset;
        public PathPlannerTrajectory thisPathPLan;

        AutoPoses(double _x, double _y, double _rot, Waypoint[] _WP, PathPlannerTrajectory _PPT){
            thisX = _x;
            thisY = _y;
            thisRot = _rot;
            thisWPset = _WP;
            thisPathPLan = _PPT;

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

    public void chooserBuilder(){
        //SJV: DESIGN A DEFAULT AUTO WHICH SHOOTS A BALL AND MOVES OFF THE TARMAC
       for (AutoPoses myAutoPose : AutoPoses.values()){
           
        SINGLE_INSTANCE.m_autoChooser.addOption(myAutoPose.toString(), myAutoPose);
       }
    }

    public double getDistance(double X1, double Y1, double X2, double Y2) { //just the distance formula - uses current x and y positions
        distance = Math.sqrt(Math.pow((X2 - X1), 2) + Math.pow((Y2 - Y1), 2));
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

    public void waypointRunner(Waypoint[] thisWaypointSet){
        // If we made one round with the state, we have successfully initialized
        if (!StateHasInitialized) {StateHasInitialized = true;}
        thisWaypointSet[currentWaypointNumber].action.run();
        if (StateHasFinished){
            currentWaypointNumber++;
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
    }
}
