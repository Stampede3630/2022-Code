package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Shooter implements Loggable {
    private static Shooter instance;
    private static double shooterSpeed = 20000;
    private static WPI_TalonFX shooterDrive;

    static {
        instance = new Shooter();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public void init() {  //TEST!!!!!
        shooterDrive = new WPI_TalonFX(0);
        shooterDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);

        shooterDrive.config_kF(0,
            1023*0.75/18000, 20);

        shooterDrive.config_kP(0,
            0.05686333, 20);
    }

    public void shoot() {
        double targetVelocity_UnitsPer100ms = shooterSpeed;

        if (Robot.xbox.getLeftTriggerAxis() > 0) {
            shooterDrive.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

        } else {
            shooterDrive.set(0);
        }
    }


    @Config.NumberSlider(name="Set Shooter Speed", min = 0, max = 18000, blockIncrement = 1000, rowIndex = 0, columnIndex = 0, height = 5, width = 5)
    public void setShooterSpeed(double targetVelocity) {
        shooterSpeed = targetVelocity;
       
    }
}
