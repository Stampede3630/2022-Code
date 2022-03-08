package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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
        shooterDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        shooterDrive.setNeutralMode(NeutralMode.Coast);

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
        if (Robot.xbox.getLeftTriggerAxis() > 0 || Robot.INTAKE.shootNow || (homocideTheBattery && !Robot.INTAKE.topLimitSwitch.get())) { ///SJV dont like this logic completely
            shooterDrive.set(ControlMode.Velocity, shooterSpeed);
            turnToShooter();
        } else {
            shooterDrive.set(0);
        }
    }

    public void turnToShooter() {
        if (!Robot.INTAKE.limelightIsOpen) {
            Robot.INTAKE.limelightSolenoid.set(Value.kForward);
        }
        //Make shooter pipeline
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        Robot.INTAKE.limelightIsOpen = true;
        // Figure out which way later
    }

    public void checkAndSetShooterCANStatus() {
        if(shooterDrive.hasResetOccurred()){
          int mycounter = 0;
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100) !=ErrorCode.OK) {mycounter++;}
          if(shooterDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100) !=ErrorCode.OK) {mycounter++;}
          System.out.println("RESET DETECTED FOR TALONFX " + shooterDrive.getDeviceID() + " Errors: " + mycounter );
        }
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
