package frc.robot;

import java.lang.reflect.Method;
import java.util.concurrent.Callable;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import io.github.oblarg.oblog.Loggable;

public class Climber implements Loggable{
    static boolean StateHasFinished;
    static boolean StateHasInitialized;
    public static enum ClimberState{ 
        STATE1(Climber::myFirstAction, "STATE2"), 
        STATE2(Climber::mySecondAction, "STATE3"), 
        STATE3(Climber::myThirdAction, "STATE4"), 
        STATE4(Climber::myFourthAction, "NONE");  

        private Runnable action;
        private String NextState;

        ClimberState(Runnable _Action, String nextState){
            action = _Action;}
    }

    public static void myFirstAction(){
        if(!StateHasInitialized){
            //do things
            StateHasInitialized = true;
        };
        //do more things

        //exit conditions
        if(true) {
            StateHasFinished  = true;
        }
    }

    
}
