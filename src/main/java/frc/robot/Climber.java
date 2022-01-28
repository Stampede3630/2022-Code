package frc.robot;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Climber implements Loggable{
    @Log
    boolean StateHasFinished;
    @Log
    Boolean StateHasInitialized;
    @Log
    String CurrentState = "";
    boolean StartingStateOverride;

    private static Climber SINGLE_INSTANCE = new Climber();
    
    public static enum ClimberState{ 
        
        STATE1(SINGLE_INSTANCE::myFirstAction, "STATE2"), 
        STATE2(SINGLE_INSTANCE::myFirstAction, "STATE3"), 
        STATE3(SINGLE_INSTANCE::myFirstAction, "STATE4"), 
        STATE4(SINGLE_INSTANCE::DoneAction, "DONE");  

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

    public void myFirstAction(){
        if(!StateHasInitialized){
            //do things
            
            //These are required
            StateHasFinished = false; 
            StateHasInitialized = true;
        }
        
        //do more things

        //exit conditions
        if(true) {
            StateHasFinished  = true;
        }
    }

    public void DoneAction(){
        if(!StateHasInitialized){
            //do things
            
            //These are required
        }
        //do more things
        //NO EXIT CONDITIONS PLEASE
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

        if (StateHasFinished){
            CurrentState = ClimberState.valueOf(CurrentState).getNextState();
            StateHasFinished = false; 
            StateHasInitialized = true;
        }
    }



    

    
}
