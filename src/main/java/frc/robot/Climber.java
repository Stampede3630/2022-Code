package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
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
    boolean upOne;
    boolean tiltArms = false;
    final double TICKSPERREVOLUTION=2048;
    final double TICKSATTOP=239200;
    final double INCHESATTOP=27;
    final double TICKSPERINCH=TICKSATTOP/INCHESATTOP;
    final int FULLEXTEND = 27;
    final int HALFEXTEND = 12;
    final int CLICKARMS = 5; // <-- place holder value, position to move climber arms down (in inches)

    public static DigitalInput climberHomeLeft;
    public static DigitalInput climberHomeRight;
    

    private static Climber SINGLE_INSTANCE = new Climber();
    public static Climber getInstance() {
        return SINGLE_INSTANCE;
    }
    
    public void init(){
        climberTalon = new WPI_TalonFX(14);
        climberHomeLeft = new DigitalInput(2);
        climberHomeRight = new DigitalInput(3);
        climberTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        climberTalon.setSelectedSensorPosition(0,0,20);
        climberTalon.configSelectedFeedbackCoefficient(1/TICKSPERINCH, 0, 20);
        climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        climberTalon.config_kP(0, 500, 20);
        atOrigin = false;
        upOne = false;

        // We want the default state to keep climber upright
        climberSolenoid.set(Value.kForward);
    }

    public void periodic() {
        manualClimberSolenoid();
        manualClimberMotor();
        // solenoidController();
        if(atOrigin){
            climberRunner("");
        } else{
            reZero();
           
        }
        
    }

    public void manualClimberMotor(){
        if (Robot.xbox.getPOV()==0){
            //raiseAndExtend();
            climberTalon.set(ControlMode.PercentOutput, 0.3);
        }
        else if (Robot.xbox.getPOV()==180){
            //lowerArm28();
            climberTalon.set(ControlMode.PercentOutput, -0.3);
        } else {
            climberTalon.set(ControlMode.PercentOutput, 0);
        }
    }
    
    public void manualClimberSolenoid(){
        if (Robot.xbox.getPOV()==90 ){ 
           openSolenoid();
        } else if (Robot.xbox.getPOV()==270 ){ 
           closeSolenoid();
        }
    }

    //// public void solenoidController() {
    //     if (tiltArms) {
    //         closeSolenoid();
    //     } else {
    //         openSolenoid();
    //     }
    // }

    public enum ClimberState{
        STATESTART(SINGLE_INSTANCE::getUserInput, "STATE1RAISEARM28"),
        STATE1RAISEARM28(SINGLE_INSTANCE::openSolenoid, "STATECRINGERAISEARM28"), // change back to raise28 when done 
        STATECRINGERAISEARM28(SINGLE_INSTANCE::raiseArm28, "STATE1USERINPUT"),
        STATE1USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE1LOWERARM28"), 
        STATE1LOWERARM28(SINGLE_INSTANCE::lowerArm28, "STATE2USERINPUT"), // change back to lower28 when done (hasn't been tested yet)
        STATE2USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE1RAISEANDEXTEND"), // EVAN IT WORKS EVAN EVAN TEST B RO okike dokie
        STATE1RAISEANDEXTEND(SINGLE_INSTANCE::raiseAndExtend, "STATE3USERINPUT"), 
        STATE3USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE1OPENSOLENOID"),
        STATE1OPENSOLENOID(SINGLE_INSTANCE::openSolenoid, "STATE4USERINPUT"),
        STATE4USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE2RAISEANDEXTEND"), 
        STATE2RAISEANDEXTEND(SINGLE_INSTANCE::lowerArm28, "STATE5USERINPUT"),
        STATE5USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE6USERINPUT"),
        STATE6USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE2LOWERARM28"),
        STATE2LOWERARM28(SINGLE_INSTANCE::raiseAndExtend, "STATE7USERINPUT"),
        STATE7USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE3RAISEANDEXTEND"), 
        STATE3RAISEANDEXTEND(SINGLE_INSTANCE::openSolenoid, "STATE8USERINPUT"), 
        STATE8USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE2OPENSOLENOID"),
        STATE2OPENSOLENOID(SINGLE_INSTANCE::lowerArm28, "STATE9USERINPUT"),
        STATE9USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE4RAISEANDEXTEND"), 
        STATE4RAISEANDEXTEND(SINGLE_INSTANCE::raiseAndExtend, "STATE10USERINPUT"),
        STATE10USERINPUT(SINGLE_INSTANCE::getUserInput, "STATE11USERINPUT"),
        STATE11USERINPUT(SINGLE_INSTANCE::openSolenoid, "STATE3LOWERARM28"),
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

        if (climberHomeLeft.get() || climberHomeRight.get()) {
            atOrigin = true;
            climberTalon.set(ControlMode.PercentOutput, 0);
            climberTalon.setSelectedSensorPosition(0, 0, 20);
        } else if (!atOrigin && !upOne) {
            climberTalon.set(ControlMode.Position, 1);
            if (climberTalon.getSelectedSensorPosition(0) >= 1) {
                upOne = true;
            }
        } else if (!atOrigin && upOne) {
            climberTalon.set(ControlMode.PercentOutput, -0.3);
        }
    }

    public void raiseAndExtend()  {
        if (climberTalon.getSelectedSensorPosition(0) < 8) {
            climberTalon.set(ControlMode.Position, 10);
            climberSolenoid.set(Value.kReverse);
        } else if (climberTalon.getSelectedSensorPosition(0) >= 8) {
            climberTalon.set(ControlMode.Position, 27);
            
            if (climberTalon.getSelectedSensorPosition(0) >= 27) {
                StateHasFinished = true;
            }
        }
    }

    public void raiseArm28() {
        climberTalon.set(ControlMode.Position, 28);

        if (climberTalon.getSelectedSensorPosition(0) >= 28) {
            StateHasFinished  = true;
        }
    }

    public void lowerArm28() {
        climberTalon.set(ControlMode.PercentOutput, -0.5);

        // **** Add fault tolerance for arms ****
        if ((climberHomeLeft.get() || climberHomeRight.get()) && climberTalon.getSelectedSensorPosition(0) <= 2) {
            climberTalon.setSelectedSensorPosition(0, 0, 20);
            climberTalon.set(ControlMode.PercentOutput, 0);
            StateHasFinished  = true;
        }
    }

    public void raiseArm14() {
        climberTalon.set(ControlMode.Position, 14);

        if (climberTalon.getSelectedSensorPosition(0)>=14) {
            StateHasFinished =true;
        }
    }

    public void openSolenoid() {
        climberSolenoid.set(Value.kForward);
        tiltArms = true;
        StateHasFinished = true;
    }
    
    public void closeSolenoid() {
        climberSolenoid.set(Value.kReverse);
        tiltArms = false;
        StateHasFinished = true;

    }

    public void getUserInput() {
        if(Robot.xbox.getAButton()){
            StateHasFinished =true;
        }
    }

    public void DoneAction() {
    } 

    @Log.BooleanBox(rowIndex = 1, columnIndex = 2)
    public boolean getClimberHomeLeft() {
      return climberHomeLeft.get();
    }
    
    @Log.BooleanBox(rowIndex = 3, columnIndex = 4)
    public boolean getClimberHomeRight() {
      return climberHomeRight.get();
    }
    @Log
    public double getClimberTalonPosition() {
      return climberTalon.getSelectedSensorPosition();
}
}