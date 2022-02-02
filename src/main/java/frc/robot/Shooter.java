package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config.NumberSlider;

public class Shooter implements Loggable {
    private static Shooter instance;
    private static double shooterSpeed = 20000;
    private static WPI_TalonFX LEFT_SHOOTER_FALCON;

    static {
        instance = new Shooter();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public void init() {  //TEST!!!!!
        LEFT_SHOOTER_FALCON = new WPI_TalonFX(0);
        LEFT_SHOOTER_FALCON.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);

        LEFT_SHOOTER_FALCON.config_kF(0,
            1023/18000, 20);

        LEFT_SHOOTER_FALCON.config_kP(0,
            0.05686333, 20);
    }

    public void shoot() {
        double targetVelocity_UnitsPer100ms = shooterSpeed;

        if (Robot.xbox.getLeftTriggerAxis() > 0 ) {
            LEFT_SHOOTER_FALCON.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            
            // belt.set(-.6);
        } else {
            LEFT_SHOOTER_FALCON.set(0);
        }
    }

    public static double rpmToRotatPer100Mili(double rpm) {
        return rpm / 600;
    }

    @Config.NumberSlider(name="Set Shooter Speed", min = 0, max = 18000, blockIncrement = 1000, columnIndex = 0, height = 5, width = 5)
    public void setShooterSpeed(double targetVelocity) {
        shooterSpeed = targetVelocity;
       
    }
}
