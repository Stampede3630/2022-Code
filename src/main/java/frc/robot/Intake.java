package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake {
  
  private static Intake SINGLE_INSTANCE = new Intake();
  
  private static WPI_TalonFX intakeDrive;
  private static DoubleSolenoid intakeSolenoid;

    public static Intake getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init(){
      intakeDrive  = new WPI_TalonFX(13);
      intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 7);
    }

    public void intakePneumatics() {
      if (Robot.xbox.getXButton() == true){
        intakeSolenoid.set(Value.kForward);
      } else {
        intakeSolenoid.set(Value.kReverse);
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
    
    
}
