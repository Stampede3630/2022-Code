package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;



public class AutoContainer implements Loggable {
    @Log
    boolean StateHasFinished = false;
    @Log
    Boolean StateHasInitialized = false;
    @Log
    String CurrentState = "";
    boolean StartingStateOverride;
    SwerveDriveOdometry a_odometry;

    private static AutoContainer SINGLE_INSTANCE = new AutoContainer();

    public static AutoContainer getInstance() {
        return SINGLE_INSTANCE;
    }

    // waypoint 1: x = 8.37, y = 5.43
    // waypoint 2: x = 8.89, y = 7.79
    // waypoint 3: x = 8.81, y = 5.32
    // waypoint 4: x = 11.23, y = 6.18
    // waypoint 5: x = 15.11, y = 6.97
    // waypoint 6: x = 9.16, y = 5.23


    public enum AutoState {
        STATEAUTOSTART(SINGLE_INSTANCE::bruh, "gruh"),
        gruh(SINGLE_INSTANCE::bruh, "gruh");
        
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
        if (!StateHasInitialized) {StateHasInitialized = true;}
        AutoState.valueOf(CurrentState).getAction().run();
        if (StateHasFinished){
            CurrentState = AutoState.valueOf(CurrentState).getNextState();
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
    }

    public void bruh() {
        
    }

    private void intake() {
        
        //x = 8.89, y = 7.79
    }

    @Config.ToggleButton(name="Field Position 1", defaultValue = false, rowIndex = 1, columnIndex = 1, height = 1, width = 2)
    private static void fieldPosition1(boolean pos1) {
        if (pos1) { setAuto(1); }
        else { setAuto(5); }
    }

    @Config.ToggleButton(name="Field Position 2", defaultValue = false, rowIndex = 1, columnIndex = 3, height = 1, width = 2)
    private static void fieldPosition2(boolean pos2) {
        if (pos2) { setAuto(2); }
        else { setAuto(6); }
    }

    @Config.ToggleButton(name="Field Position 3", defaultValue = false, rowIndex = 1, columnIndex = 5, height = 1, width = 2)
    private static void fieldPosition3(boolean pos3) {
        if (pos3) { setAuto(3); }
        else { setAuto(7); }
    }

    @Config.ToggleButton(name="Field Position 4", defaultValue = false, rowIndex = 1, columnIndex = 7, height = 1, width = 2)
    private static void fieldPosition4(boolean pos4) {
        if (pos4) { setAuto(4); }
        else {setAuto(8); }
    }

    private static void setAuto(int autoPosition) {
        
        switch (autoPosition) { // These are fake values, put in real stuff
            case 1:
                SwerveMap.GYRO.setAngleAdjustment(45);
                break;
            case 2:
                SwerveMap.GYRO.setAngleAdjustment(90);
                break;
            case 3:
                SwerveMap.GYRO.setAngleAdjustment(180);
                break;
            case 4:
                SwerveMap.GYRO.setAngleAdjustment(270);
                break;
            case 5: SwerveMap.GYRO.setAngleAdjustment(-45);
                break;
            case 6: SwerveMap.GYRO.setAngleAdjustment(-90);
                break;
            case 7: SwerveMap.GYRO.setAngleAdjustment(-180);
                break;
            case 8: SwerveMap.GYRO.setAngleAdjustment(-270);
            default:
                SwerveMap.GYRO.setAngleAdjustment(0);
                break;
        }
    }
}
