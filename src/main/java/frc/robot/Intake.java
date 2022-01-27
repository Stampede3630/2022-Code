package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake {


    public static void spinIntake() {
        TalonFX spinMotor = new TalonFX(0); //Filler port number 
        //Actually make the motors spin on button press
      if (Robot.xbox.getXButton() == true){
        spinMotor.set(ControlMode.PercentOutput, 1);
      } else {
        spinMotor.set(ControlMode.PercentOutput, 0);  
      }
    }
    
}
