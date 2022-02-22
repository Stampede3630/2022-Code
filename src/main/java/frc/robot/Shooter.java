package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Shooter implements Loggable {
    private static Shooter SINGLE_INSTANCE = new Shooter();
    private double shooterSpeed = 5000;
    private WPI_TalonFX shooterDrive;

    public boolean homocideTheBattery;

    public static Shooter getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() {  //TEST!!!!!
        shooterDrive = new WPI_TalonFX(0);
        shooterDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        shooterDrive.config_kF(0,
            1023*0.94/18000, 20);

        shooterDrive.config_kP(0,
            0.075, 20);

        homocideTheBattery = false;
    }

    public boolean shooterAtSpeed(){
        if (shooterDrive.getSelectedSensorVelocity(0) >= shooterSpeed){ // Checks if the shooter is spinning fast enough to shoot *see intake file*
            return true;
        } else {
            return false;
        }
        
    }

    public void shoot() {
        if (Robot.xbox.getLeftTriggerAxis() > 0 || Robot.INTAKE.shootNow || homocideTheBattery) {
            shooterDrive.set(ControlMode.Velocity, shooterSpeed);
        } else {
            shooterDrive.set(0);
        }
    }

    public void turnToShooter() {
        limelightSolenoid.set(Value.kForward);
        limelightIsOpen = true;
    // Figure out which way later
  }

    @Config.NumberSlider(name="Set Shooter Speed", min = 0, max = 18000, blockIncrement = 1000, rowIndex = 0, columnIndex = 0, height = 5, width = 5)
    public void setShooterSpeed(double targetVelocity) {
        shooterSpeed = targetVelocity;
       
    }

    @Config.ToggleButton(name = "kill battery?",defaultValue = false, rowIndex = 1, columnIndex = 0)
    public void killTheBattery(boolean _input) {
        homocideTheBattery = _input;
    }
}
