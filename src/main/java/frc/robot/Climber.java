package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.swerve.SwerveMap;
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
    
    boolean ddrTime;
    boolean StartingStateOverride;
    boolean atOrigin;
    boolean upCompleted;
    boolean tiltArms = false;
    boolean fullyExtended = false;
    boolean autoExtend = true;
    // boolean climberSafety = true;

    public double safePitch = 2; 
    public float currentPitch;
    public float currentPitchSpeed;
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
        climberTalon = new WPI_TalonFX(Constants.ClimberMotorId);
        climberHomeLeft = new DigitalInput(Constants.LeftClimberSwitchID);
        climberHomeRight = new DigitalInput(Constants.RightClimberSwitchID);
        climberTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        climberTalon.setSelectedSensorPosition(0,0,200);
        climberTalon.configSelectedFeedbackCoefficient(1/TICKSPERINCH, 0, 20);
        climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClimberSolenoidForwardID, Constants.ClimberSolenoidReverseID);
        climberTalon.config_kP(0, 1000, 20);
        atOrigin = false;
        upCompleted = false;
        

        // We want the default state to keep climber upright
        climberSolenoid.set(Value.kForward);
    }

    public void periodic() {
        currentPitch = SwerveMap.GYRO.getRoll();
        currentPitchSpeed = SwerveMap.GYRO.getVelocityY();
        if (!atOrigin) {
          reZero();
        } else {
            manualClimberSolenoid();
            manualClimberMotor();
        }
    }

    public void reZero() { 
        if (climberHomeLeft.get() || climberHomeRight.get() || (upCompleted && climberTalon.getSelectedSensorPosition(0) < 1 && climberTalon.getSelectedSensorVelocity(0) >-.5 )) {
            atOrigin = true;
            climberTalon.set(ControlMode.PercentOutput, 0);
            climberTalon.setSelectedSensorPosition(0, 0, 20);
        } else if (!atOrigin && !upCompleted) {
            climberTalon.set(ControlMode.Position, 5);
            if (climberTalon.getSelectedSensorPosition(0) >= 4) {
                upCompleted = true;
            }
        } else if (!atOrigin && upCompleted) {
            climberTalon.set(ControlMode.PercentOutput, -0.3);
        }
    }

    private void manualClimberMotor(){
        if (climberTalon.getSelectedSensorPosition(0) >= 28) {
            fullyExtended = true;
        } else {
            fullyExtended = false;
        }

        // if (ddrTime){  
        //     if (Robot.ddrPad.getPOV() == 0 && !fullyExtended){
        //         climberTalon.set(ControlMode.PercentOutput, 1); 
        //     } else if (Robot.ddrPad.getPOV() == 180 && !(climberHomeLeft.get() || climberHomeRight.get())) {
        //         climberTalon.set(ControlMode.PercentOutput, -1);
        //     } else {
        //         climberTalon.set(ControlMode.PercentOutput, 0);
        //     }
        // } else {
            if (Robot.xbox.getPOV() == 0 && !fullyExtended){
                climberTalon.set(ControlMode.PercentOutput, 1); 
            } else if (Robot.xbox.getPOV() == 180 && !(climberHomeLeft.get() || climberHomeRight.get())){
                climberTalon.set(ControlMode.PercentOutput, -1);
            } else {
                climberTalon.set(ControlMode.PercentOutput, 0);
            }
        //}
    }
    
    private void manualClimberSolenoid(){
        // if (ddrTime){
        //     if (Robot.ddrPad.getPOV() == 90 ){ 
        //     openSolenoid();
        //     } else if (Robot.ddrPad.getPOV() == 270 ){ 
        //     closeSolenoid(); 
        //     }
        // } else {
            if (Robot.xbox.getPOV() == 90 ){ 
                openSolenoid();
            } else if (Robot.xbox.getPOV() == 270 ){ 
                closeSolenoid();
            }
        //}
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

    public void checkAndSetClimberCANStatus() {
        if(climberTalon.hasResetOccurred()){
            int mycounter = 0;
          
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100)!=ErrorCode.OK) {mycounter++;}
            if(climberTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
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

}