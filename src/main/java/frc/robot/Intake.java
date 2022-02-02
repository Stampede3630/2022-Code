package frc.robot;

import java.text.RuleBasedCollator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake implements Loggable {
  
  private static Intake SINGLE_INSTANCE = new Intake();
  
  private static WPI_TalonFX intakeDrive;
  private static WPI_TalonFX indexBottom;
  private static WPI_TalonFX indexTop;
  private static WPI_TalonFX indexShooter;
  private static DoubleSolenoid intakeSolenoid;
  // SWITCHES: GREEN = NOT PRESSED, RED = PRESSED
  // SWITCHES RETURN TRUE WHEN NOT PRESSED, FALSE WHEN PRESSED
  private static DigitalInput bottomLimitSwitch;
  private static DigitalInput topLimitSwitch;
  private static Shooter shooter; // finish this later

    public static Intake getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init(){
      intakeDrive  = new WPI_TalonFX(13);
      indexBottom = new WPI_TalonFX(16);
      indexTop = new WPI_TalonFX(12);
      indexShooter = new WPI_TalonFX(0);

      intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 7);

      bottomLimitSwitch = new DigitalInput(1);
      topLimitSwitch = new DigitalInput(0);

      shooter = Shooter.getInstance();
    }

    public void intakePneumatics() {
      if (Robot.xbox.getRightTriggerAxis() > 0){
        intakeSolenoid.set(Value.kReverse);
      } else {
        intakeSolenoid.set(Value.kForward);
      }
    }

    // public void spinIntake() { 
    //     // Actually make the motors spin on button press
    //   if (Robot.xbox. > 0){
    //     intakeDrive.set(ControlMode.PercentOutput, .5);
    //   } else if(Robot.xbox.getYButton()) {
    //     intakeDrive.set(ControlMode.PercentOutput, -.5);  
    //   } else {
    //     intakeDrive.set(ControlMode.PercentOutput, 0);
    //   }
    // }

    public void enableIndexing()  {
      if (Robot.xbox.getRightTriggerAxis() > 0) {
        indexerDrive();
      // } else {
      //   indexerDrive("Don't Index");
      }
    }

    // NOT ACTIVE CODE RIGHT NOW
    public void indexerDrive() {
      // put method in as key
      // if (indexManager() != null) {
      //   key = indexManager();
      // }
        switch (indexManager()) {
          case "Spin Bottom":
            indexBottom.set(ControlMode.PercentOutput, 0.2);
            indexTop.set(ControlMode.PercentOutput, 0);
            break;

           case "Don't Index":
            indexBottom.set(ControlMode.PercentOutput, 0); 
            indexTop.set(ControlMode.PercentOutput, 0);
            break;

          case "Spin Top":
            indexTop.set(ControlMode.PercentOutput, 0.2);
            indexBottom.set(ControlMode.PercentOutput, 0.0);
            break;

          default:
            indexBottom.set(ControlMode.PercentOutput, 0.2);
            indexTop.set(ControlMode.PercentOutput, 0.2);
            break;
        }
      } 
//Try swapping ! around
    public String indexManager() {
      if (!topLimitSwitch.get()){
        return "Spin Bottom";
      } else if (!bottomLimitSwitch.get() && !topLimitSwitch.get()){
        return "Don't Index";
      } else if (!bottomLimitSwitch.get()){
        return "Spin Top";
      } else {
        return "default";
      }
    }

    @Log.BooleanBox(rowIndex = 1, columnIndex = 2)
    public boolean getBottomLimitSwitch() {
      return bottomLimitSwitch.get();
    }

    @Log.BooleanBox(rowIndex = 3, columnIndex = 4)
    public boolean getTopLimitSwitch() {
      return topLimitSwitch.get();
    }
    
    // OLD AND BAD CODE
    public void crapIndex() {
      if ( 0 == 1) {
        // if (topLimitSwitch.get()) {
        //   indexBottom.set(ControlMode.PercentOutput, 0.5);
        // } else {
        //   indexBottom.set(ControlMode.PercentOutput, 0.5);
        //   indexTop.set(ControlMode.PercentOutput, 0.5);
        // }
        
      } else if (Robot.xbox.getRightTriggerAxis() > 0) {
        indexBottom.set(ControlMode.PercentOutput, 0.5);
        indexTop.set(ControlMode.PercentOutput, 0.5);
        //indexShooter.set(ControlMode.PercentOutput, .9);
      } else {
        indexTop.set(ControlMode.PercentOutput, 0);
        indexBottom.set(ControlMode.PercentOutput, 0);
        // indexShooter.set(ControlMode.PercentOutput, 0
        // );
      }
    }
    
}
