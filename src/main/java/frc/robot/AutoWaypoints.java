package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory.State;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class AutoWaypoints implements Loggable {

    private static AutoWaypoints SINGLE_INSTANCE = new AutoWaypoints();

    private double stateStartTime;
    private double currentX;
    private double currentY;
 
    public static AutoWaypoints getInstance() {
        return SINGLE_INSTANCE;
    }

    public void autoPeriodic() {
        currentX = Robot.SWERVEDRIVE.getXPos();
        currentY = Robot.SWERVEDRIVE.getYPos();
    }

    @Log
    boolean StateHasFinished = false;
    @Log
    Boolean StateHasInitialized = false;
    @Log
    String CurrentState = "";
    boolean StartingStateOverride;
    SwerveDriveOdometry a_odometry;

    public enum AutoState {
        STARTSTATE(SINGLE_INSTANCE::amritUwU, "BALL1TRANSITION"),
        BALL1TRANSITION(SINGLE_INSTANCE::intakeBall1, "SHOOT1TRANSITION"),
        SHOOT1TRANSITION(SINGLE_INSTANCE::shoot1, "BALL2TRANSITION"),
        BALL2TRANSITION(SINGLE_INSTANCE::intakeBall2, "BALL3TRANSITION"),
        BALL3TRANSITION(SINGLE_INSTANCE::intakeBall3, "SHOOT2TRANSITION"),
        SHOOT2TRANSITION(SINGLE_INSTANCE::shoot2, "SHOOT2TRANSITION");
        
        private Runnable action;
        private String nextState;

        AutoState(Runnable _action, String _nextState){
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
            CurrentState = AutoState.values()[0].toString();
        }

        //if we made one round with the state, we have successfully initialized
        AutoState.valueOf(CurrentState).getAction().run();
        if (!StateHasInitialized) {StateHasInitialized = true;}
        if (StateHasFinished){
            CurrentState = AutoState.valueOf(CurrentState).getNextState();
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
    }

    private void intakeBall1() {
        double ballX = 7.801352311565382;
        double ballY = 1.6783324705889917;

        if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
            Robot.INTAKE.intakeNow = true;
            StateHasFinished = true;
        }
    }

    private void intakeBall2() {
        double ballX = 5.278066252335036;
        double ballY = 2.0139989647067895;

        if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
            Robot.INTAKE.intakeNow = true;
            StateHasFinished = true;
        }
    }

    private void intakeBall3() {
        double ballX = 1.2732177363088897;
        double ballY = 1.2153442028403045;

        if (getDistance(currentX, currentY, ballX, ballY) < 0.5) {
            Robot.INTAKE.intakeNow = true;
            StateHasFinished = true;
        }
    }

    private void shoot1() {
        double posX = 7.882375258421403;
        double posY = 2.951550206897882;

        if (getDistance(currentX, currentY, posX, posY) < 0.5) {
            Robot.INTAKE.intakeNow = true;
            StateHasFinished = true;
        }
    }

    private void shoot2() {
        double posX = 7.581432884384757;
        double posY = 3.0325731537539022;

        if (getDistance(currentX, currentY, posX, posY) < 0.5) {
            Robot.INTAKE.intakeNow = true;
            StateHasFinished = true;
        }
    }

    private void amritUwU(){
        StateHasFinished = true;

    }

    private double getDistance(double currentX, double currentY, double newX, double newY) {
        double distance = Math.sqrt(Math.pow((newX - currentX), 2) + Math.pow((newY - currentY), 2));
        return distance;
    }
    
}
