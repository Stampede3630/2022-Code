package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake {
  
  private static Intake SINGLE_INSTANCE = new Intake();
  
  private static WPI_TalonFX intakeDrive;
  private static DoubleSolenoid leftIntakeSolenoid;
  private static DoubleSolenoid rightIntakeSolenoid;


    public static Intake getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init(){
      intakeDrive  = new WPI_TalonFX(12);
      leftIntakeSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 2, 3);
      rightIntakeSolenoid = new DoubleSolenoid(4, PneumaticsModuleType.REVPH, 5, 6);
    }

    public void intakePneumatics() {
      if (Robot.xbox.getXButton() == true){
        leftIntakeSolenoid.set(Value.kForward);
        rightIntakeSolenoid.set(Value.kForward);
      } else {
        leftIntakeSolenoid.set(Value.kReverse);
        rightIntakeSolenoid.set(Value.kReverse);
      }
    }

    public void spinIntake() { 
        //Actually make the motors spin on button press
      if (Robot.xbox.getXButton() == true){
        intakeDrive.set(ControlMode.PercentOutput, 1);
      } else {
        intakeDrive.set(ControlMode.PercentOutput, 0);  
      }
    }
    
}
