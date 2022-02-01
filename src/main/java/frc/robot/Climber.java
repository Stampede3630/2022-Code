package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Climber implements Loggable{
    @Log
    boolean StateHasFinished;
    @Log
    Boolean StateHasInitialized = false;
    @Log
    String CurrentState = "";

    @Log
    Timer climbTimer = new Timer();
    boolean StartingStateOverride;

    private static Climber SINGLE_INSTANCE = new Climber();
    public static Climber getInstance() {
        return SINGLE_INSTANCE;
    }
    
    public static enum ClimberState{ 
        
        STATE1(SINGLE_INSTANCE::turnBlinker1On, "STATE2"), 
        STATE2(SINGLE_INSTANCE::turnBlinker2On, "STATE3"), 
        STATE3(SINGLE_INSTANCE::turnBlinker3On, "STATE4"), 
        STATE4(SINGLE_INSTANCE::turnBlinker4On, "STATE5"), 
        STATE5(SINGLE_INSTANCE::turnBlinker5On, "DONE"), 
        DONE(SINGLE_INSTANCE::DoneAction, "DONE");  

        private Runnable action;
        private String nextState;

        ClimberState(Runnable _action, String _nextState){
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

    public void climberRunner(String _startingState){
        if (_startingState != "" || StartingStateOverride){
            CurrentState = _startingState;
            StartingStateOverride = false;
        } 

        if (CurrentState == "") {
            CurrentState = ClimberState.values()[0].toString();
        }

        ClimberState.valueOf(CurrentState).getAction();

        //if we made one round with the state, we have successfully initialized
        if (!StateHasInitialized) {StateHasInitialized = true;}

        if (StateHasFinished){
            CurrentState = ClimberState.valueOf(CurrentState).getNextState();
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
    }

    @Log
    Boolean blinker1 = false;
    @Log
    Boolean blinker2 = false;
    @Log
    Boolean blinker3 = false;
    @Log
    Boolean blinker4 = false;
    @Log
    Boolean blinker5 = false;

    public void turnBlinker1On(){
        if(!StateHasInitialized){
            climbTimer.start();
        }
        blinker1 = true;
        if(climbTimer.hasElapsed(5)) {
            climbTimer.stop();
            climbTimer.reset();
            StateHasFinished  = true;
        }
    }

    public void turnBlinker2On(){
        if(!StateHasInitialized){
            climbTimer.start();
        }
        blinker2 = true;
        if(climbTimer.hasElapsed(5)) {
            climbTimer.stop();
            climbTimer.reset();
            StateHasFinished  = true;
        }
    }

    public void turnBlinker3On(){
        if(!StateHasInitialized){
            climbTimer.start();
        }
        blinker3= true;
        if(climbTimer.hasElapsed(5)) {
            climbTimer.stop();
            climbTimer.reset();
            StateHasFinished  = true;
        }
    }

    
    public void turnBlinker4On(){
        if(!StateHasInitialized){
            climbTimer.start();
        }
        blinker4 = true;
        if(climbTimer.hasElapsed(5)) {
            climbTimer.stop();
            climbTimer.reset();
            StateHasFinished  = true;
        }
    }

    public void turnBlinker5On(){
        if(!StateHasInitialized){
            climbTimer.start();
        }
        blinker5 = true;
        if(climbTimer.hasElapsed(5)) {
            climbTimer.stop();
            climbTimer.reset();
            StateHasFinished  = true;
        }
    }


    /* public void myFirstAction(){
        if(!StateHasInitialized){

        }
        
        //do more things

        //exit conditions
        if(true) {
            StateHasFinished  = true;
        }
    }*/

    public void DoneAction() {
        if(!StateHasInitialized){
            climbTimer.start();
        }
        if (climbTimer.hasElapsed(5)) {
            blinker1 = !blinker1;
            blinker2 = !blinker2;
            blinker3 = !blinker3;
            blinker4 = !blinker4;
            blinker5 = !blinker5;
            climbTimer.stop();
            climbTimer.reset();
            climbTimer.start();
        }
        

    } 


    

    
}
