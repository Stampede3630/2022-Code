package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Climber implements Loggable{
    @Log
    boolean StateHasFinished = false;
    @Log
    Boolean StateHasInitialized = false;
    @Log
    String CurrentState = "";
    WPI_TalonFX climberTalon;
    DoubleSolenoid climberSolenoid;
 
    boolean StartingStateOverride;
    boolean atOrigin;
    boolean upFive;
    final double TICKSPERREVOLUTION=2048;
    final double TICKSATTOP=239200;
    final double INCHESATTOP=27;
    final double TICKSPERINCH=TICKSATTOP/INCHESATTOP;
    final int FULLEXTEND = 27;
    final int HALFEXTEND = 12;
    final int CLICKARMS = 5; // <-- place holder value, position to move climber arms down (in inches)

    private static Climber SINGLE_INSTANCE = new Climber();
    public static Climber getInstance() {
        return SINGLE_INSTANCE;
    }
    
    public void init(){
        climberTalon = new WPI_TalonFX(14);
        climberTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        climberTalon.setSelectedSensorPosition(0,0,20);
        climberTalon.configSelectedFeedbackCoefficient(1/TICKSPERINCH, 0, 20);
        climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 7);
        climberTalon.config_kP(0, 250, 20);
        atOrigin = false;
        upFive = false;
    }

    public void periodic() {
        manualClimberSolenoid();
        manualClimberMotor();
        if(atOrigin){
            climberRunner("");
        } else{
            reZero();
        }
        
    }

    

    public void manualClimberMotor(){
        if (Robot.xbox.getPOV()==0){
            raiseAndExtend();
        }
        else if (Robot.xbox.getPOV()==180){
            lowerArm28();
        }
        }


    public void manualClimberSolenoid(){
        if (Robot.xbox.getPOV()==90){
            openSolenoid();
        } else if (Robot.xbox.getPOV()==270){
            closeSolenoid();
        }

    }
    //I took this off static, because it wasn't necessary.
    public enum ClimberState{
        STATESTART(SINGLE_INSTANCE::getUserInput, "STATE1RAISEARM28"),
        STATE1RAISEARM28(SINGLE_INSTANCE::raiseArm28, "STATE1USERINPUT"), 
        STATE1USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE1LOWERARM28"), 
        STATE1LOWERARM28(SINGLE_INSTANCE::lowerArm28, "STATE2USERINPUT"), 
        STATE2USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE1RAISEANDEXTEND"), 
        STATE1RAISEANDEXTEND(SINGLE_INSTANCE::raiseAndExtend, "STATE3USERINPUT"), 
        STATE3USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE1OPENSOLENOID"),
        STATE1OPENSOLENOID(SINGLE_INSTANCE::openSolenoid, "STATE4USERINPUT"),
        STATE4USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE2RAISEANDEXTEND"), 
        STATE2RAISEANDEXTEND(SINGLE_INSTANCE::raiseAndExtend, "STATE5USERINPUT"),
        STATE5USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE6USERINPUT"),
        STATE6USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE2LOWERARM28"),
        STATE2LOWERARM28(SINGLE_INSTANCE::lowerArm28, "STATE7USERINPUT"),
        STATE7USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE3RAISEANDEXTEND"), 
        STATE3RAISEANDEXTEND(SINGLE_INSTANCE::raiseAndExtend, "STATE8USERINPUT"), 
        STATE8USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE2OPENSOLENOID"),
        STATE2OPENSOLENOID(SINGLE_INSTANCE::openSolenoid, "STATE9USERINPUT"),
        STATE9USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE4RAISEANDEXTEND"), 
        STATE4RAISEANDEXTEND(SINGLE_INSTANCE::raiseAndExtend, "STATE10USERINPUT"),
        STATE10USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE11USERINPUT"),
        STATE11USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE3LOWERARM28"),
        STATE3LOWERARM28(SINGLE_INSTANCE::lowerArm28, "DONE"),
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
        if (_startingState != "" && StartingStateOverride){
            CurrentState = _startingState;
            StartingStateOverride = false;
        } 
       
        if (CurrentState == "") {
            CurrentState = ClimberState.values()[0].toString();
        }


        //if we made one round with the state, we have successfully initialized
        if (!StateHasInitialized) {StateHasInitialized = true;}
        ClimberState.valueOf(CurrentState).getAction().run();
        if (StateHasFinished){
            CurrentState = ClimberState.valueOf(CurrentState).getNextState();
            StateHasFinished = false; 
            StateHasInitialized = false;
        }
    }

    public void reZero() { 

    if (atOrigin == false && upFive == false) {
        climberTalon.set(ControlMode.Position, 5);
        if (climberTalon.getSelectedSensorPosition(0) == 5) {
            upFive = true;
        }
    } else if (atOrigin == false && upFive == true) {
        climberTalon.set(ControlMode.PercentOutput, -0.1);
        if (climberTalon.getSelectedSensorVelocity(0) > -0.5 && climberTalon.getSelectedSensorPosition(0) < 2) {
            atOrigin = true;
            climberTalon.set(ControlMode.PercentOutput, 0);
            climberTalon.setSelectedSensorPosition(0, 0, 20);
        }
    } 
}

    public void raiseAndExtend()  {
        if (climberTalon.getSelectedSensorPosition(0) < 24) {
            climberTalon.set(ControlMode.Position, 26);
        } else if (climberTalon.getSelectedSensorPosition(0) >= 24) {
            closeSolenoid();
            climberTalon.set(ControlMode.Position, 27);
            
            if (climberTalon.getSelectedSensorPosition(0) >= 27) {
                StateHasFinished = true;
            }
        }
    }

    public void raiseArm28(){
        climberTalon.set(ControlMode.Position, 28);

        if(climberTalon.getSelectedSensorPosition(0)>=28){
            StateHasFinished  = true;
        }
    }

    public void lowerArm28(){
        climberTalon.set(ControlMode.Position, 1);
        
        if(climberTalon.getSelectedSensorPosition(0)<=0){
            StateHasFinished = true;
        }
    }

    public void raiseArm14(){
        climberTalon.set(ControlMode.Position, 14);

        if(climberTalon.getSelectedSensorPosition(0)>=14){
            StateHasFinished =true;
        }
    }

    public void openSolenoid(){
        climberSolenoid.set(Value.kForward);
        StateHasFinished =true;
    }
    
    public void closeSolenoid(){
        climberSolenoid.set(Value.kReverse);
        StateHasFinished =true;
    }

    public void getUserInput(){
        if(Robot.xbox.getAButton()){
            StateHasFinished =true;
        }
    }
 
 
    
    public void DoneAction() {
    } 

    
}