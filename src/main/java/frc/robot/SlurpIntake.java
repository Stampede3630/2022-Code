package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import io.github.oblarg.oblog.Loggable;

public class SlurpIntake implements Loggable{
    
    private static SlurpIntake SINGLE_INSTANCE = new SlurpIntake();

    WPI_TalonSRX intakeTalon = new WPI_TalonSRX(0);


    public static SlurpIntake getInstance() {
        return SINGLE_INSTANCE;
    }

public void spinIntake() {

     if (Robot.xbox.getRightBumperPressed()){
        intakeTalon.set(ControlMode.PercentOutput, 0.5);
     }else{
         intakeTalon.set(ControlMode.PercentOutput, 0);
     }
    
}

}
