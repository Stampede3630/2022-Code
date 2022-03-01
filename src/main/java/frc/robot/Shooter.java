package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    public void init() { // TEST!!!!!
        shooterDrive = new WPI_TalonFX(10);
        shooterDrive.setInverted(TalonFXInvertType.Clockwise);
        shooterDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        shooterDrive.config_kF(0,
                1023 * 0.94 / 18000, 20);

        shooterDrive.config_kP(0,
                0.075, 20);

        homocideTheBattery = false;
    }

    public boolean shooterAtSpeed() {
        if (shooterDrive.getSelectedSensorVelocity(0) >= shooterSpeed * 0.95) { // Checks if the shooter is spinning fast enough to shoot *see intake file*
            return true;
        } else {
            return false;
        }

    }
    //SJV: I hope ur fine with me renaming this method
    public void shooterPeriodic() {
        if (Robot.xbox.getLeftTriggerAxis() > 0 || Robot.INTAKE.shootNow || (homocideTheBattery && !Robot.INTAKE.topLimitSwitch.get())) {
            shooterDrive.set(ControlMode.Velocity, shooterSpeed);
            turnToShooter();
        } else {
            shooterDrive.set(0);
        }
    }

    public void turnToShooter() {
        Robot.INTAKE.limelightSolenoid.set(Value.kForward);
        //Make shooter pipeline
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        Robot.INTAKE.limelightIsOpen = true;
        // Figure out which way later
    }


    //SJV: ONCE WE FIGURE OUT OUR SHOOTER ANGLE AND SPEED MAKE BOOLEAN FOR EACH OF THE SHOOTER SPEEDS AND PUT IT ON THE COMPETITION LOGGER
    @Config.NumberSlider(name = "Set Shooter Speed", defaultValue = 15000, min = 0, max = 18000, blockIncrement = 1000, rowIndex = 0, columnIndex = 0, height = 5, width = 5)
    public void setShooterSpeed(double targetVelocity) {
        shooterSpeed = targetVelocity;

    }

    @Config.ToggleButton(name = "kill battery?", defaultValue = false, rowIndex = 1, columnIndex = 0)
    public void killTheBattery(boolean _input) {
        homocideTheBattery = _input;
    }
}
