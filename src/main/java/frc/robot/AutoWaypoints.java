package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
    @Log
    public int currentWaypointNumber = 0;
    private double currentX;
    private double currentY;
    @Log
    public AutoPose chosenPath;
    public AutoPose[] myAutoContainer;
    
    @Log
    public boolean StateHasFinished = false;
    @Log
    public Boolean StateHasInitialized = false;
    @Log(tabName = "CompetitionLogger")
    public double distance = 0;
    @Log(tabName = "CompetitionLogger")
    public SendableChooser<AutoPose> m_autoChooser = new SendableChooser<>();

    public static AutoWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() {
        SwerveMap.GYRO.reset();
        if (m_autoChooser.getSelected()==null){
            chosenPath = myAutoContainer[0];
        } else {
            chosenPath = m_autoChooser.getSelected();
        }
        chosenWaypoints = chosenPath.thisWPset;
        SwerveMap.GYRO.setAngleAdjustment(chosenPath.thisRot);
        Robot.SHOOTER.homocideTheBattery = true;
        currentWaypointNumber = 0;
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
        myAutoContainer = new AutoPose[] {
            new AutoPose("FenderFourBallAutoWPs", 7.80, 1.68, -84.69, FenderFourBallAutoWPs, PathPlanner.loadPath("blueAutoTest", 3, 2.5)),
            new AutoPose("FenderTwoBallAutoWPs", 6.09, 5.23, 43.78, FenderTwoBallAutoWPs, PathPlanner.loadPath("twoBallAuto", 3, 2.5))};
        for (AutoPose myAutoPose : myAutoContainer ){
            m_autoChooser.addOption(myAutoPose.name, myAutoPose);
        }

    }

    public void autoPeriodic() {
        currentX = Robot.SWERVEDRIVE.getXPos();
        currentY = Robot.SWERVEDRIVE.getYPos();
        waypointRunner(FenderFourBallAutoWPs);
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
            Robot.INTAKE.indexTop.set(ControlMode.PercentOutput, -0.25);
            Robot.INTAKE.indexBottom.set(ControlMode.PercentOutput, -0.25);

            if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) > 0.5) {
            Robot.INTAKE.indexTop.set(ControlMode.PercentOutput, 0);
            Robot.INTAKE.indexBottom.set(ControlMode.PercentOutput, 0);
                StateHasFinished = true;
            }
        }
    }

    public void done(){
        //ADDED TO NOT GO OUT OF BOUNDS IN A ARRAY WAYPOINT RUNNER
        //FEEL FREE TO ADD THING TO THE DONE STATE
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

    public class AutoPose {


        public double thisX;
        public double thisY;
        public double thisRot;
        public Waypoint[] thisWPset;
        public PathPlannerTrajectory thisPathPLan;
        public String name;

        AutoPose(String _S, double _x, double _y, double _rot, Waypoint[] _WP, PathPlannerTrajectory _PPT){
            thisX = _x;
            thisY = _y;
            thisRot = _rot;
            thisWPset = _WP;
            thisPathPLan = _PPT;
            name = _S;

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
