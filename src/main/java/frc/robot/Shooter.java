package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter implements Loggable {
    private static Shooter SINGLE_INSTANCE = new Shooter();
    @Log
    private double shooterSpeed = 5000;
    private double shooterSpeedOffset = 0;
   @Log
    private double hoodAngle = 0;
    private double hoodAngleOffset = 0;
    private WPI_TalonFX shooterDrive;
    private WPI_TalonFX hoodMotor;
    // SWITCHES DEFAULT TO TRUE WHEN NOT PRESSED
    // TRUE = SHOOTER NOT AT HOME, FALSE = SHOOTER AT HOME
    private DigitalInput leftHoodSwitch;
    private DigitalInput rightHoodSwitch;
    private boolean hoodAtOrigin = false;
    private boolean rotComplete = false;
    public boolean homocideTheBattery;
    public boolean limelightShooting = true;
    public boolean bloopShot = false;

    // public static SimpleMotorFeedforward shooterMotorFeedforward;

    public static Shooter getInstance() {
        return SINGLE_INSTANCE;
    }

    public void init() { // TEST!!!!!
        shooterDrive = new WPI_TalonFX(10);
        shooterDrive.setInverted(TalonFXInvertType.Clockwise);
        shooterDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        shooterDrive.setNeutralMode(NeutralMode.Coast);
        shooterDrive.configVoltageCompSaturation(12.0,100);
        shooterDrive.enableVoltageCompensation(true);

        limelightShooting = true;

        // Don't mess with this value, suck a big one Evan
        shooterDrive.config_kF(0, (1023 * .8) / 20000, 100);
        shooterDrive.config_kD(0, 15, 100);
        shooterDrive.config_kP(0, 0.45, 100);
        
        hoodMotor = new WPI_TalonFX(49);
        hoodMotor.config_kP(0, 0.07625, 100);
        hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        hoodMotor.setSelectedSensorPosition(0, 0, 200);
        hoodMotor.setNeutralMode(NeutralMode.Brake);

        rotComplete = false;
        hoodAtOrigin = false;
        

        homocideTheBattery = false;

        leftHoodSwitch = new DigitalInput(4);
        rightHoodSwitch = new DigitalInput(5);

        // shooterMotorFeedforward = new SimpleMotorFeedforward(0.89886, 0.00029265, 0.000062406);
    }

    public void shooterPeriodic() {
       // System.out.println(shooterDrive.getSelectedSensorVelocity(0) + " " + shooterSpeed + " " + shooterAtSpeed());
        if (!hoodAtOrigin) {
            rezeroHood();
        } else if (hoodAtOrigin) {
            hoodAngle = calculateShooterAngle();
            shooterSpeed = calculateShooterSpeed();
            rotateHood(hoodAngle);
        }

        //bloop shot
        

        // DemandType.ArbitraryFeedForward, shooterMotorFeedforward.calculate(shooterSpeed) / 12
        if (Robot.xbox.getLeftTriggerAxis() > 0 || Robot.INTAKE.shootNow || (homocideTheBattery && !Robot.INTAKE.topLimitSwitch.get())) { ///SJV dont like this logic completely
        // if (Robot.xbox.getLeftTriggerAxis() > 0 || homocideTheBattery) {
            shooterDrive.set(ControlMode.Velocity, shooterSpeed);
        } else {
            shooterDrive.set(0);
        }
    }
    
    public boolean shooterAtSpeed() {
        double velocityError = Math.abs(shooterDrive.getSelectedSensorVelocity(0) - shooterSpeed);
        // TODO: IF TOO MUCH VARIATION IN SHOTS, NARROW THESE VALUES SO IT STILL SHOOTS
        if (shooterSpeed * 0.01 < velocityError && velocityError <= shooterSpeed * 0.07) { // Checks if the shooter is spinning fast enough to shoot *see intake file*
            return true;

        } else {
            return false;
        }

    }

    // Find shooter angle based on distance from hub
    // TODO: Show distance in shuffleboard
    @Log
    public double calculateShooterAngle() { 
        if ((NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) && limelightShooting  && !bloopShot && !Robot.xbox.getLeftBumper()){
            double angle = 35.0 + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            double distance = (((103.0 - 36.614) / Math.tan(Math.toRadians(angle))) + 12.4) / 12;
            // System.out.println(distance); 
            // angle = -14.75x^3 + 497.7x^2 -3726x + 11270, x = distance
            angle = -37840 + 10740 * distance - 686.3 * (Math.pow(distance, 2)) + 15.22 * (Math.pow(distance, 3)) + hoodAngleOffset;

            if (0 < angle && angle < 32000) {
                return angle;
            } else {
                return hoodAngle;
            }

        } else if (bloopShot || Robot.xbox.getLeftBumper()){
            return 20000.00;
        }else {
            return hoodAngle;
        }
    }

    // Find shooter speed based on shooter angle
    @Log
    public double calculateShooterSpeed() {
        if ((NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) && limelightShooting && !bloopShot && !Robot.xbox.getLeftBumper()){
        double angle = 35.0 + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double distance = (((103.0 - 36.614) / Math.tan(Math.toRadians(angle))) + 12.4) / 12;
        // shooterSpeed = -2.846x^3 + 116.5x^2 -1196x + 18110, x = distance
        double shooterSpeed = (358.7*distance) + 10960;

        return shooterSpeed + shooterSpeedOffset;
        } else if (bloopShot || Robot.xbox.getLeftBumper()){
            return 5000.00;
        }else {
            return shooterSpeed;
        }
    }

    public void rotateHood(double angle) {
        hoodMotor.set(ControlMode.Position, angle);
    }

    private void rezeroHood() { // check default on hood switches
        // if (!leftHoodSwitch.get()
        // ) { //SJV:create climbsafety variable to override limit switches incase malfunctions occur
        //     hoodAtOrigin = true;
        //     hoodMotor.set(ControlMode.PercentOutput, 0);
        //     hoodMotor.setSelectedSensorPosition(0, 0, 20);
        // } else if (!hoodAtOrigin && !rotComplete) {
        //     hoodMotor.set(ControlMode.Position, 5000);
        //     if (hoodMotor.getSelectedSensorPosition(0) >= 4500) {
        //         rotComplete = true;
        //     }
        // } else if (!hoodAtOrigin && rotComplete) {
        //     hoodMotor.set(ControlMode.PercentOutput, -0.1);
        // }

        if (!leftHoodSwitch.get() || !rightHoodSwitch.get()){     //this code works, but i've left the previous code in jic -e 
            hoodAtOrigin = true;        //**this is also gross bc nested if statements 
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodMotor.setSelectedSensorPosition(0, 0, 20);
        } else if ((leftHoodSwitch.get() || rightHoodSwitch.get()) && !hoodAtOrigin && !rotComplete){
            hoodMotor.set(ControlMode.Position, 3000);
                if (hoodMotor.getSelectedSensorPosition(0) >= 2300){
                    rotComplete = true; 
                        if (rotComplete){
                            hoodMotor.set(ControlMode.PercentOutput, -0.1);
                        }
                }
        }
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

    @Config.NumberSlider(name = "Set Shooter Angle", defaultValue = 0, min = 0, max = 32000, blockIncrement = 1000, rowIndex = 3, columnIndex = 2, height = 1, width = 2)
    public void setHoodAngle(double targetAngle) {
        hoodAngle = targetAngle;
    }

    @Config.NumberSlider(name = "Set Shooter Angle Offset", defaultValue = 0, min = -5000, max = 5000, blockIncrement = 500, rowIndex = 2, columnIndex = 2, height = 1, width = 2)
    public void setHoodAngleOffset(double targetAngleOffset) {
        hoodAngleOffset = targetAngleOffset;
    }

    //SJV: ONCE WE FIGURE OUT OUR SHOOTER ANGLE AND SPEED MAKE BOOLEAN FOR EACH OF THE SHOOTER SPEEDS AND PUT IT ON THE COMPETITION LOGGER
    @Config.NumberSlider(name = "Set Shooter Speed", defaultValue = 15000, min = 0, max = 18000, blockIncrement = 1000, rowIndex = 0, columnIndex = 2, height = 1, width = 2)
    public void setShooterSpeed(double targetVelocity) {
        shooterSpeed = targetVelocity;

    }
    

    @Config.NumberSlider(name = "Set Shooter Speed Offset", defaultValue = 0, min = -5000, max = 5000, rowIndex = 1, columnIndex = 2, blockIncrement = 500, height = 1, width = 2)
    public void setShooterSpeedOffset(double targetOffest) {
        shooterSpeedOffset = targetOffest;
    }

    @Config.ToggleButton(name = "kill battery?", defaultValue = false, rowIndex = 0, columnIndex = 0, height = 1, width = 2)
    public void killTheBattery(boolean _input) {
        homocideTheBattery = _input;
    }
 
    @Log(rowIndex = 1, columnIndex = 0)
    public boolean getLeftHoodLimit() {
        return leftHoodSwitch.get();
    }

    @Log(rowIndex = 1, columnIndex = 1)
    public boolean getRightHoodLimit() {
        return rightHoodSwitch.get();
    }

}
