package frc.robot;

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
  private static WPI_TalonFX indexLeft;
  private static WPI_TalonFX indexRight;
  private static DoubleSolenoid intakeSolenoid;
  private static DigitalInput bottomLimitSwitch;
  private static DigitalInput topLimitSwitch;

    public static Intake getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init(){
      intakeDrive  = new WPI_TalonFX(13);
      indexLeft = new WPI_TalonFX(11);
      indexRight = new WPI_TalonFX(12);

      intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 7);

      bottomLimitSwitch = new DigitalInput(0);
      topLimitSwitch = new DigitalInput(1);
    }

    public void intakePneumatics() {
      if (Robot.xbox.getXButton() == true){
        intakeSolenoid.set(Value.kReverse);
      } else {
        intakeSolenoid.set(Value.kForward);
      }
    }

    public void spinIntake() { 
        // Actually make the motors spin on button press
      if (Robot.xbox.getXButton()){
        intakeDrive.set(ControlMode.PercentOutput, .5);
      } else if(Robot.xbox.getYButton()) {
        intakeDrive.set(ControlMode.PercentOutput, -.5);  
      } else {
        intakeDrive.set(ControlMode.PercentOutput, 0);
      }
    }

    public void crapIndex() {
      if (Robot.xbox.getXButton()) {
        if (topLimitSwitch.get()) {
          indexLeft.set(ControlMode.PercentOutput, 0);
        } else {
          indexLeft.set(ControlMode.PercentOutput, 0.5);
          indexRight.set(ControlMode.PercentOutput, 0.5);
        }
        
      } else if (Robot.xbox.getYButton()) {
        indexLeft.set(ControlMode.PercentOutput, -0.5);
        indexRight.set(ControlMode.PercentOutput, -0.5);
      } else {
        indexRight.set(ControlMode.PercentOutput, 0);
        indexLeft.set(ControlMode.PercentOutput, 0);
      }
    }

    // public void indexerDrive() {
    //   // put method in as key
    //     switch (indexManager()) {
    //       case 1:
    //         indexLeft.set(ControlMode.PercentOutput, value);
            
    //         break;
        
    //       default:
    //         break;
    //     }
    //   } 

    // public int indexManager() {
    //   if (bottomLimitSwitch.get()){
    //     return 1;
    //   }
    //   else if (topLimitSwitch.get()){
    //     return 2;
    //   }
    //   else if (bottomLimitSwitch.get() && topLimitSwitch.get()){
    //     return 3;
    //   } else {
    //     return 0;
    //   }
    // }

    @Log.BooleanBox(rowIndex = 1, columnIndex = 2)
    public boolean getBottomLimitSwitch() {
      return bottomLimitSwitch.get();
    }

    @Log.BooleanBox(rowIndex = 3, columnIndex = 4)
    public boolean getTopLimitSwitch() {
      return topLimitSwitch.get();
    }
    
    
}
