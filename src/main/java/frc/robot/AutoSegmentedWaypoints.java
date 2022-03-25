package frc.robot;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoWaypoints.Waypoint;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class AutoSegmentedWaypoints implements Loggable {

    private static AutoSegmentedWaypoints SINGLE_INSTANCE = new AutoSegmentedWaypoints();
    public Waypoint[] FenderTwoBallAutoWPs;
    public Waypoint[] FenderFourBallAutoWPs; 
    public Waypoint[] HighFiveBallAutoWPs;
    public Waypoint[] HighTwoBallAutoWPs;
    public Waypoint[] HighTwoBallAutoNewWPs;
    public Waypoint[] HighFiveBallSegAutoWPs;
    public Waypoint[] HighFiveBallSeg2AutoWPs;
    public Waypoint[] HighFourBallSegAutoWPs;
    public PathPlannerTrajectory fourBallAutoPath;
    public PathPlannerTrajectory twoBallAutoPath;
    public Waypoint[] chosenWaypoints;
    @Log(tabName = "CompetitionLogger", rowIndex = 2, columnIndex = 4)
    public int currentWaypointNumber = 0;
    private double currentX;
    private double currentY;
    public AutoPose chosenPath;
    public AutoPose[] myAutoContainer;
    public PathPlannerTrajectory seg1;
    public PathPlannerTrajectory seg2;
    public PathPlannerTrajectory seg3;
    public PathPlannerTrajectory seg4;
    public PathPlannerTrajectory twoBall;
    public PathPlannerTrajectory v2seg1;
    public PathPlannerTrajectory v2seg2;
    public PathPlannerTrajectory v2seg3;
    public PathPlannerTrajectory v2seg4;
    public PathPlannerTrajectory ivIntake;
    
    public boolean StateHasFinished = false;
    public Boolean StateHasInitialized = false;
    @Log(tabName = "CompetitionLogger", rowIndex = 1, columnIndex = 4)
    public double distance = 0;
    @Log(tabName = "CompetitionLogger", rowIndex = 0, columnIndex = 3, height = 1, width = 2)
    public SendableChooser<AutoPose> m_autoChooser = new SendableChooser<>();

    public static AutoSegmentedWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() {
        SwerveMap.GYRO.reset();
        seg1 = PathPlanner.loadPath("VbhSegOne", 3.5, 3.0);
        seg2 = PathPlanner.loadPath("VbhSegTwo", 3.5, 3.0);
        seg3 = PathPlanner.loadPath("VbhSegThree", 3.5, 3.0);
        seg4 = PathPlanner.loadPath("VbhSegFour", 3.5, 3.0);
        twoBall = PathPlanner.loadPath("TwoBallHigh", 3.5, 3.0);
        v2seg1 = PathPlanner.loadPath("Vbh2SegOne", 3.5, 3.0);
        v2seg2 = PathPlanner.loadPath("Vbh2SegTwo", 3.5, 3.0);
        v2seg3 = PathPlanner.loadPath("Vbh2SegThree", 3.5, 3.0);
        v2seg4 = PathPlanner.loadPath("Vbh2SegFour", 3.5, 3.0);
        ivIntake = PathPlanner.loadPath("IVBallFrickyIntakey", 3.5, 3.0);
        
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
        waypointRunner(chosenWaypoints);
    }

    public void loadAutoPaths(){

        HighFiveBallSegAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 7.63, 0.7, seg1),
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 5.35, 2.04, seg2),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 2.40, 1.32, seg3),
            new Waypoint(SINGLE_INSTANCE::shoot, 6.39, 0.95, seg4)
        };
        HighTwoBallAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 5.32, 6.00, twoBall)
        };  
        HighFiveBallSeg2AutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 7.69, 0.59, v2seg1),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 1.40, 1.31, v2seg2),
            new Waypoint(SINGLE_INSTANCE::shoot, 6.59, 1.20, v2seg3),
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 5.41, 2.06, v2seg4)
        };
        HighFourBallSegAutoWPs = new Waypoint[] {
            new Waypoint(SINGLE_INSTANCE::shootAndIntake, 7.63, 0.7, seg1),
            new Waypoint(SINGLE_INSTANCE::intakeBall, 1.46, 1.23, ivIntake),
            new Waypoint(SINGLE_INSTANCE::shoot, 6.39, 0.95, seg4)

        };


    //     HighTwoBallAutoNewWPs = new Waypoint[] {
    //         new Waypoint(SINGLE_INSTANCE::intakeBall, 5.19, 6.03),
    //         new Waypoint(SINGLE_INSTANCE::shoot, 6.52, 5.71),
    //         new Waypoint(SINGLE_INSTANCE::done, 0, 0)
    //     };

    //     HighTwoBallAutoWPs = new Waypoint[] {
    //         new Waypoint(SINGLE_INSTANCE::shootAndIntake, 5.32, 6.00),
    //         new Waypoint(SINGLE_INSTANCE::done, 0, 0)
    //     };

    //     HighFiveBallAutoWPs = new Waypoint[] {
    //         new Waypoint(SINGLE_INSTANCE::shoot, 7.62, 1.76),
    //         new Waypoint(SINGLE_INSTANCE::intakeBall, 7.60, 0.53),
    //         new Waypoint(SINGLE_INSTANCE::shootAndIntake, 5.43, 2.13),
    //         new Waypoint(SINGLE_INSTANCE::intakeBall, 1.03, 1.12),
    //         new Waypoint(SINGLE_INSTANCE::shoot,5.87, 1.12),
    //         new Waypoint(SINGLE_INSTANCE::done, 0, 0)
    //     };

    //    FenderTwoBallAutoWPs = new Waypoint[] {
    //         new Waypoint(SINGLE_INSTANCE::intakeBall, 5.19, 6.00),
    //         new Waypoint(SINGLE_INSTANCE::shoot, 6.97, 4.44),
    //         new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
    //     };
    
    //     FenderFourBallAutoWPs = new Waypoint[] {
    //         new Waypoint(SINGLE_INSTANCE::intakeBall, 7.62, 0.51),
    //         new Waypoint(SINGLE_INSTANCE::shoot, 7.88, 2.51),
    //         new Waypoint(SINGLE_INSTANCE::intakeBall, 5.29, 1.93),
    //         new Waypoint(SINGLE_INSTANCE::intakeBall, 1.38, 1.22),
    //         new Waypoint(SINGLE_INSTANCE::shoot, 7.36, 2.75),
    //         new Waypoint(SINGLE_INSTANCE::done, 0, 0) 
    //     };
        myAutoContainer = new AutoPose[] {
            new AutoPose("HighFiveBallSegAutoWPs", 7.57, 1.84, -91.17, HighFiveBallSegAutoWPs),
            new AutoPose("HighTwoBallAutoWPs", 6.09, 5.19, 43.78, HighTwoBallAutoWPs),
            new AutoPose("HighFourBallSegAutoWPs", 7.57, 1.84, -91.17, HighFourBallSegAutoWPs)
        };
        for (AutoPose myAutoPose : myAutoContainer ){
            m_autoChooser.addOption(myAutoPose.name, myAutoPose);
        }

    }

    private void intakeBall() {
        //SJV: NEED TO TURN INTAKE OFF WHEN STATE IS FINISHED, NEED TO PROLLY LOWER THE DISTANCE FROM HALF A METER TO LESS (TEST PLEASE)
        if (SwerveTrajectory.trajectoryStatus.equals("done")) {
            Robot.INTAKE.intakeNow = true;
            Robot.INTAKE.shootNow = false;
        } else if (Robot.INTAKE.indexState.equals("default")) {
            if (chosenWaypoints.length != currentWaypointNumber+1){
            StateHasFinished = true;
            }
                
        }
    }

    private void shoot() {
            // Robot.SWERVEDRIVE.autoLimeLightAim = true;
        if (SwerveTrajectory.trajectoryStatus.equals("done")) {
            Robot.INTAKE.shootNow = true;
            Robot.INTAKE.intakeNow = false;
            
            // Robot.INTAKE.indexBottom.set(ControlMode.PercentOutput, -0.25);

        } else if (Robot.INTAKE.indexState.equals("default")) {
            if (chosenWaypoints.length != currentWaypointNumber+1){
            // Robot.INTAKE.indexBottom.set(ControlMode.PercentOutput, 0);
                // Robot.SWERVEDRIVE.autoLimeLightAim = false;
                StateHasFinished = true;
            }
                Robot.INTAKE.shootNow = false;
        }
    }

    private void shootAndIntake() {
                Robot.INTAKE.intakeNow = true;
        if (SwerveTrajectory.trajectoryStatus.equals("done")) {
                 Robot.INTAKE.shootNow = true;
            // if(Robot.INTAKE.indexState.equals("2 Balls") || Robot.INTAKE.indexState.equals("1 Ball")){
            //     Robot.INTAKE.intakeNow = false;
            //     Robot.INTAKE.shootNow = true;  
            //     Robot.SWERVEDRIVE.autoLimeLightAim = true;
            // } else {
            //     Robot.INTAKE.intakeNow = true;
            //     Robot.INTAKE.shootNow = false; 
            //     Robot.SWERVEDRIVE.autoLimeLightAim = false; 
            // }
        } else if(Robot.INTAKE.indexState.equals("default") && SwerveTrajectory.trajectoryStatus.equals("done")) {
            if (chosenWaypoints.length != currentWaypointNumber+1){

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
        public PathPlannerTrajectory pathPlannerSegment;

        public Waypoint(Runnable _action, double _x, double _y, PathPlannerTrajectory _PPS) {
            action = _action;
            posX=_x;
            posY=_y;
            pathPlannerSegment = _PPS;
        }
    }
    

    public class AutoPose {

        public double thisX;
        public double thisY;
        public double thisRot;
        public Waypoint[] thisWPset;
        public String name;

        AutoPose(String _S, double _x, double _y, double _rot, Waypoint[] _WP){
            thisX = _x;
            thisY = _y;
            thisRot = _rot;
            thisWPset = _WP;
            name = _S;

        }
    }

    public void waypointRunner(Waypoint[] thisWaypointSet){
        // If we made one round with the state, we have successfully initialized
        if (!StateHasInitialized) {
            SwerveTrajectory.resetTrajectoryStatus();
            
            
            StateHasInitialized = true;}
            SwerveTrajectory.PathPlannerRunner(Robot.AUTOWAYPOINTS.chosenPath.thisPathPLan,  Robot.SWERVEDRIVE.m_odometry, SwerveMap.getRobotAngle());
        thisWaypointSet[currentWaypointNumber].action.run();
        if (StateHasFinished){
            currentWaypointNumber++;
            
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
    }
}
