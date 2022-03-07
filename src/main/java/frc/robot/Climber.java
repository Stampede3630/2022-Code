package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
    final double TICKSATTOP=(283675-1475);
    final double INCHESATTOP=27;
    final double TICKSPERINCH=TICKSATTOP/INCHESATTOP;
    final int FULLEXTEND = 27;
    final int HALFEXTEND = 12;
    final int CLICKARMS = 5; // <-- Place holder value, position to move climber arms down (in inches)


    public DigitalInput climberHomeLeft;
    public DigitalInput climberHomeRight;
    

    private static Climber SINGLE_INSTANCE = new Climber();
    public static Climber getInstance() {
        return SINGLE_INSTANCE;
    }
    
    public void init(){
        climberTalon = new WPI_TalonFX(8);
        climberTalon.setNeutralMode(NeutralMode.Brake);
        climberHomeLeft = new DigitalInput(2);
        climberHomeRight = new DigitalInput(3);
        climberTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        climberTalon.setSelectedSensorPosition(0,0,20);
        climberTalon.configSelectedFeedbackCoefficient(1/TICKSPERINCH, 0, 20);
        climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
        climberTalon.config_kP(0, 1000, 20);
        atOrigin = false;
        upOne = false;


        // We want the default state to keep climber upright
        climberSolenoid.set(Value.kForward);
    }

    public void periodic() {
        manualClimberSolenoid();
        manualClimberMotor();
        if (!atOrigin) {
          reZero();
        } else if (Robot.COMPETITIONLOGGER.beginClimb) {
            climberRunner("");
        }
        
    }

    private void manualClimberMotor(){
        if (Robot.xbox.getPOV()==0){
            climberTalon.set(ControlMode.PercentOutput, 1); //SJV:MAYBE PUT SOME SAFETY HERE TOO?!?!
        }
        else if (Robot.xbox.getPOV()==180){//SJV Put some safety in here?!?!
            climberTalon.set(ControlMode.PercentOutput, -1);
        } else if (climberHomeLeft.get() || climberHomeRight.get()) {
            climberTalon.set(ControlMode.PercentOutput, 0.0);
        }
        else {
            climberTalon.set(ControlMode.PercentOutput, 0);
        }
    }
    
    private void manualClimberSolenoid(){
        if (Robot.xbox.getPOV()==90 ){ 
           openSolenoid();
        } else if (Robot.xbox.getPOV()==270 ){ 
           closeSolenoid();
           
        }
    }

    public enum ClimberState{
        START(SINGLE_INSTANCE::getUserInput, "OPENSOLENOID1"),
        OPENSOLENOID1(SINGLE_INSTANCE::openSolenoid, "RAISEARM1"), // Change back to raise28 when done 
        RAISEARM1(SINGLE_INSTANCE::raiseArm28, "USERINPUT1"),
        USERINPUT1(SINGLE_INSTANCE::getUserInput, "LOWERARM1"), 
        LOWERARM1(SINGLE_INSTANCE::lowerArm28, "USERINPUT2"), // Change back to lower28 when done (hasn't been tested yet)
        USERINPUT2(SINGLE_INSTANCE::getUserInput, "RAISEANDEXTEND1"), 
        RAISEANDEXTEND1(SINGLE_INSTANCE::raiseAndExtend, "USERINPUT3"), 
        USERINPUT3(SINGLE_INSTANCE::getAltUserInput, "OPENSOLENOID2"),
        OPENSOLENOID2(SINGLE_INSTANCE::openSolenoid, "USERINPUT4"),
        USERINPUT4(SINGLE_INSTANCE::getAltUserInput, "RAISEANDEXTEND2"), 
        RAISEANDEXTEND2(SINGLE_INSTANCE::lowerArm28, "USERINPUT5"),
        USERINPUT5(SINGLE_INSTANCE::getUserInput, "USERINPUT6"),
        USERINPUT6(SINGLE_INSTANCE::getUserInput, "LOWERARM2"),
        LOWERARM2(SINGLE_INSTANCE::raiseAndExtend, "USERINPUT7"),
        USERINPUT7(SINGLE_INSTANCE::getUserInput, "RAISEANDEXTEND3"), 
        RAISEANDEXTEND3(SINGLE_INSTANCE::openSolenoid, "USERINPUT8"), 
        USERINPUT8(SINGLE_INSTANCE::getAltUserInput, "OPENSOLENOID3"),
        OPENSOLENOID3(SINGLE_INSTANCE::lowerArm28, "USERINPUT9"),
        USERINPUT9(SINGLE_INSTANCE::getUserInput, "RAISEANDEXTEND4"), 
        RAISEANDEXTEND4(SINGLE_INSTANCE::raiseAndExtend, "USERINPUT10"),
        USERINPUT10(SINGLE_INSTANCE::getUserInput, "USERINPUT11"),
        USERINPUT11(SINGLE_INSTANCE::openSolenoid, "LOWERARM3"),
        LOWERARM3(SINGLE_INSTANCE::lowerArm28, "DONE"),
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


        // If we made one round with the state, we have successfully initialized
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
        if (climberTalon.getSelectedSensorPosition(0) < 10.0) {
            climberTalon.set(ControlMode.Position, 12.0);
            //(ControlMode.Position, DemandType.ArbitraryFeedForward, -.15);

        } else if (climberTalon.getSelectedSensorPosition(0) >= 10.0) {
            climberSolenoid.set(Value.kReverse);
            climberTalon.set(ControlMode.Position, 26.0);
            if (climberTalon.getSelectedSensorPosition(0) >= 26.0) {
                StateHasFinished = true;
            }
        }
    }

    public void raiseArm28() {
        climberTalon.set(ControlMode.Position, 25.0);

        if (climberTalon.getSelectedSensorPosition(0) >= 25.0) {
            StateHasFinished  = true;
        }
    }

    public void lowerArm28() {
        climberTalon.set(ControlMode.Position, 5, DemandType.ArbitraryFeedForward, -0.15);

        // **** Add fault tolerance for arms ****
        if ((climberHomeLeft.get() || climberHomeRight.get()) && climberTalon.getSelectedSensorPosition(0) <= 5) {
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
    public void getAltUserInput() {
        if(Robot.xbox.getXButton()){
            StateHasFinished =true;
        }
    }

    public void DoneAction() {
    } 

    public void checkAndSetClimberCANStatus() {
        if(climberTalon.hasResetOccurred()){
            int mycounter = 0;
          
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10,100) !=ErrorCode.OK) {mycounter++;} //WE ARE CHECKING VELOCITY SO KEEP IT AT 10
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 5000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 5000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000,100) !=ErrorCode.OK) {mycounter++;}
          if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000,100) !=ErrorCode.OK) {mycounter++;}
          System.out.println("RESET DETECTED FOR TALONFX " + climberTalon.getDeviceID() + " Errors: " + mycounter);
        }
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