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
    public Waypoint[] HighFiveBallAutoWPs;
    public Waypoint[] HighTwoBallAutoWPs;
    public PathPlannerTrajectory fourBallAutoPath;
    public PathPlannerTrajectory twoBallAutoPath;
    public Waypoint[] chosenWaypoints;
    @Log(tabName = "CompetitionLogger", rowIndex = 0, columnIndex = 5)
    public int currentWaypointNumber = 0;
    private double currentX;
    private double currentY;
    public AutoPose chosenPath;
    public AutoPose[] myAutoContainer;
    
    @Log
    public boolean StateHasFinished = false;
    @Log
    public Boolean StateHasInitialized = false;
    @Log(tabName = "CompetitionLogger", rowIndex = 0, columnIndex = 4)
    public double distance = 0;
    @Log(tabName = "CompetitionLogger", rowIndex = 0, columnIndex = 3, height = 1, width = 2)
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
    
    public void autoPeriodic() {
        currentX = Robot.SWERVEDRIVE.getXPos();
        currentY = Robot.SWERVEDRIVE.getYPos();
        waypointRunner(FenderFourBallAutoWPs);
    }

    public void loadAutoPaths(){

        HighTwoBallAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 5.19, 6.03),
            new Waypoint(SINGLE_INSTANCE::done, 0, 0)
        };

        HighFiveBallAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::shoot, 7.62, 1.76),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 7.60, 0.53),
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 5.43, 2.13),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 1.40, 1.39),
            new Waypoint(SINGLE_INSTANCE::shoot,5.87, 1.12),
            new Waypoint(SINGLE_INSTANCE::done, 0, 0)
        };

       FenderTwoBallAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::intakeBall, 5.19, 6.00),
            new Waypoint(SINGLE_INSTANCE::shoot, 6.97, 4.44),
            new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
        };
    
        FenderFourBallAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::intakeBall, 7.62, 0.51),
            new Waypoint(SINGLE_INSTANCE::shoot, 7.88, 2.51),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 5.29, 1.93),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 1.38, 1.22),
            new Waypoint(SINGLE_INSTANCE::shoot, 7.36, 2.75),
            new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
        };
        myAutoContainer = new AutoPose[] {
            new AutoPose("FenderFourBallAutoWPs", 7.60, 1.78, -93.69, FenderFourBallAutoWPs, PathPlanner.loadPath("FourBallBloop", 2.5, 1.0)),
            new AutoPose("FenderTwoBallAutoWPs", 7.64, 1.83, -84.69, FenderTwoBallAutoWPs, PathPlanner.loadPath("TwoBallBloop", 3, 1.0)),
            new AutoPose("HighFiveBallAutoWPs", 7.62, 1.76, -90, HighFiveBallAutoWPs, PathPlanner.loadPath("FiveBallHigh", 2.5, 1.0)),
            new AutoPose("HighTwoBallAutoWPs", 6.09, 5.19, 43.78, HighTwoBallAutoWPs, PathPlanner.loadPath("TwoBallHigh", 2.5, 1.0))
        };
        for (AutoPose myAutoPose : myAutoContainer ){
            m_autoChooser.addOption(myAutoPose.name, myAutoPose);
        }

    }

    private void intakeBall() {
        //SJV: NEED TO TURN INTAKE OFF WHEN STATE IS FINISHED, NEED TO PROLLY LOWER THE DISTANCE FROM HALF A METER TO LESS (TEST PLEASE)
        if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) < 1.5) {
            Robot.INTAKE.intakeNow = true;
            Robot.INTAKE.shootNow = false;
        } else if (Robot.INTAKE.intakeNow && getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) > 1.5) {
            StateHasFinished = true;

                
        }
    }

    private void shoot() {
        if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) < 0.5) {
            Robot.INTAKE.shootNow = true;
            Robot.INTAKE.intakeNow = false;
            // Robot.INTAKE.indexBottom.set(ControlMode.PercentOutput, -0.25);

        } else if (Robot.INTAKE.shootNow && getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) > 0.5) {
            
            // Robot.INTAKE.indexBottom.set(ControlMode.PercentOutput, 0);
                StateHasFinished = true;
        }
    }

    private void shootAndIntake() {
        if (getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) < 0.5) {

            if(Robot.INTAKE.indexState.equals("2 Balls") || Robot.INTAKE.indexState.equals("1 Ball")){
                Robot.INTAKE.intakeNow = false;
                Robot.INTAKE.shootNow = true;  
            } else {
                Robot.INTAKE.intakeNow = true;
                Robot.INTAKE.shootNow = false;  
            }
        } else if((Robot.INTAKE.intakeNow || Robot.INTAKE.shootNow) && getDistance(currentX, currentY, chosenWaypoints[currentWaypointNumber].posX, chosenWaypoints[currentWaypointNumber].posY) > 1.5) {

            StateHasFinished = true;
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
