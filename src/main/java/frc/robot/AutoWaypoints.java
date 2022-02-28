package frc.robot;

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

    private double stateStartTime;
    private double currentX;
    private double currentY;
 
    public static AutoWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() {
        //SJV: PUT ALL PATH PLANNER PATH LOADS INTO A SEPARATE METHOD AND EXECUTE AT ROBOTINIT
        fourBallAutoPath = PathPlanner.loadPath("blueAutoTest", 3, 2.5);
        SwerveMap.GYRO.reset();
        ///SJV: SET THIS ANGLE ACCORDING TO THE CHOSEN PATH IN COMPETITION
        SwerveMap.GYRO.setAngleAdjustment(-90);
        chooserBuilder();
    }

    public void autoPeriodic() {
        currentX = Robot.SWERVEDRIVE.getXPos();
        currentY = Robot.SWERVEDRIVE.getYPos();
        autoRunner("");
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
        //SJV: NAME THESE SOMETHING SIMILAR TO THE PATH IN PATH PLANNER FOR READABILITY SAKES
        STARTINGPOINTFBA(7.80, 1.68, 0.00, "STARTINGPOINTFBA"),
        STARTINGPOINTTBA(6.09, 5.23, -43.78, "STARTINGPOINTTBA");

        private double thisX;
        private double thisY;
        private double thisRot;
        //SJV: I DONT THINK YOU NEED THIS YOU CAN GET THE NAME OF THE ENUM BY DOING "toString()"
        private String thisStartPoint;

        AutoPoses(double _x, double _y, double _rot, String _startPoint){
            thisX = _x;
            thisY = _y;
            thisRot = _rot;
            thisStartPoint = _startPoint;

        }

        public double getThisX(){
            return thisX;
        }

        public double getThisY(){
            return thisY;
        }
        public double getThisRot(){
            return thisRot;
        }

        public String getStartPoint(){
            return thisStartPoint;
        }
  
    }
    public void chooserBuilder(){
        //SJV: DESIGN A DEFAULT AUTO WHICH SHOOTS A BALL AND MOVES OFF THE TARMAC
       for (AutoPoses myAutoPose : AutoPoses.values()){
           
        SINGLE_INSTANCE.m_autoChooser.addOption(myAutoPose.toString(), myAutoPose);
       }
        
    }

    //SJV: EVALUATE WHETHER OR NOT WE NEED THIS?
    public void startPointRunner(String _startPoint){
        if(_startPoint != ""){
            CurrentStartPoint = _startPoint;
        }
        if (CurrentStartPoint == "") {
            CurrentStartPoint = AutoPoses.values()[0].toString();
        }

    }

    public enum FourBallAuto {
        BALL1TRANSITION(SINGLE_INSTANCE::intakeBall, 7.801, 1.678, "SHOOT1TRANSITION"),
        SHOOT1TRANSITION(SINGLE_INSTANCE::shoot, 7.882, 2.952, "BALL2TRANSITION"),
        BALL2TRANSITION(SINGLE_INSTANCE::intakeBall, 5.278, 2.014, "BALL3TRANSITION"),
        BALL3TRANSITION(SINGLE_INSTANCE::intakeBall, 1.273, 1.215,"SHOOT2TRANSITION"),
        SHOOT2TRANSITION(SINGLE_INSTANCE::shoot, 7.581, 3.033, "SHOOT2TRANSITION");
        
        private Runnable action;
        private String nextState;
        private double posX, posY;

        FourBallAuto(Runnable _action, double _posX, double _posY, String _nextState){
            action = _action;
            nextState = _nextState;
            posX = _posX;
            posY = _posY;

        }

        public Runnable getAction() {
            return action;
        }

        public String getNextState() {
            return nextState;
        }
    }

    public enum TwoBallAuto {
        TWOBALLTRANSITION1(SINGLE_INSTANCE::twoBallIntake1, "TWOBALLTRANSITION2"),
        TWOBALLTRANSITION2(SINGLE_INSTANCE::twoBallShoot, "TWOBALLTRANSITION2");

    
        private Runnable action;
        private String nextState;

        TwoBallAuto(Runnable _action, String _nextState){
            action = _action;
            nextState = _nextState;
        }

        public Runnable getAction() {
            return action;
        }

        public String getNextState() {
            return nextState;
        }

    }

    public void autoRunner(String _startingState){
        if (_startingState != "" && StartingStateOverride){
            CurrentState = _startingState;
            StartingStateOverride = false;
        } 
       
        if (CurrentState == "") {
            //SJV: MAKE THIS TAKE THE CHOSEN PATH FROM THE CHOOSER SO YOU DONT NEED AN AUTO RUNNER FOR EACH AUTO
            CurrentState = FourBallAuto.values()[0].toString();
        }

         // If we made one round with the state, we have successfully initialized
         if (!StateHasInitialized) {StateHasInitialized = true;}
         FourBallAuto.valueOf(CurrentState).getAction().run();
         if (StateHasFinished){
             CurrentState = FourBallAuto.valueOf(CurrentState).getNextState();
             StateHasFinished = false; 
             StateHasInitialized = false;
         }
    }
        

    private void intakeBall() {
        FourBallAuto _state = FourBallAuto.valueOf(CurrentState);
        //SJV: NEED TO TURN INTAKE OFF WHEN STATE IS FINISHED, NEED TO PROLLY LOWER THE DISTANCE FROM HALF A METER TO LESS (TEST PLEASE)
        if (getDistance(currentX, currentY, _state.posX, _state.posY) < 0.5) {
            Robot.INTAKE.intakeNow = true;

            if (getDistance(currentX, currentY, _state.posX, _state.posY) > 0.5) {
                StateHasFinished = true;
            }
        }

        
    }

    private void shoot() {
        FourBallAuto _state = FourBallAuto.valueOf(CurrentState);
        //SJV: TO ALLOW FOR SPINUP, WE MAY WANT TO SET KILL THE BATTERY EARLIER IN INIT OR HERE
        if (getDistance(currentX, currentY, _state.posX, _state.posY) < 0.5) {
            Robot.INTAKE.shootNow = true;
            StateHasFinished = true;
        }
    }
    
    public void autoTwoBallRunner(String _startingState){
        if (_startingState != "" && StartingStateOverride){
            CurrentState = _startingState;
            StartingStateOverride = false;
        } 
        
        if (CurrentState == "") {
            CurrentState = FourBallAuto.values()[0].toString();
        }

        //if we made one round with the state, we have successfully initialized
        FourBallAuto.valueOf(CurrentState).getAction().run();
        if (!StateHasInitialized) {StateHasInitialized = true;}
        if (StateHasFinished){
            CurrentState = FourBallAuto.valueOf(CurrentState).getNextState();
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
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
 //SJV: I THINK THESE ARE NOW OBSOLETE, HAVING A SEPARATE BUTTON FOR EACH AUTO WILL MAKE OUR ROBOT SCHIZOPHRENIC
    @Config.ToggleButton(name = "Four Ball Auto", defaultValue = false)
    public void fbaStartButton(boolean _input, double thisX, double thisY, double thisRot){
        if(_input){
            _startPoint = "STARTINGPOINTFBA";
            SwerveMap.GYRO.setAngleAdjustment(thisRot);
            currentX = thisX;
            currentY = thisY;
            _input = false;
        }
    }

        @Config.ToggleButton(name = "Two Ball Auto", defaultValue = false)
    public void tbaStartButton(boolean _input, double thisX, double thisY, double thisRot){
        if(_input){
            _startPoint = "STARTINGPOINTTBA";
            SwerveMap.GYRO.setAngleAdjustment(thisRot);
            currentX = thisX;
            currentY = thisY;
            _input = false;
        }
    }
}
