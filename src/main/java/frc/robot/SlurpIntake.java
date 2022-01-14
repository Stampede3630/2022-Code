package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;

import io.github.oblarg.oblog.Loggable;

public class SlurpIntake implements Loggable{
    
    private static SlurpIntake SINGLE_INSTANCE = new SlurpIntake();

    WPI_TalonSRX intakeTalon;


    public static SlurpIntake getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init(){
      intakeTalon  = new WPI_TalonSRX(13);
    }

public void spinIntake() {
        //this makes the intake motor spin when right bumper is pressed
      if (Robot.xbox.getRightBumper()){
         intakeTalon.set(ControlMode.PercentOutput, -1);
      }else if (Robot.xbox.getLeftBumper()){
          intakeTalon.set(ControlMode.PercentOutput, 1);
        }else{
          intakeTalon.set(ControlMode.PercentOutput, 0);
      }
    //intakeTalon.set(ControlMode.PercentOutput, 1);
    
}

}
