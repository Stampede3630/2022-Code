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
    public boolean fancyShot = true;
    public boolean shotBlock;
    InterpolatingTreeMap<Double, Double> shootSpeedTable;
    InterpolatingTreeMap<Double, Double> shootAngleTable;
    
    public double fakeDistance;

    public double distance;
    
    public static SimpleMotorFeedforward shooterMotorFeedforward;
    
    public static Shooter getInstance() {
        return SINGLE_INSTANCE;
    }
    
    public void init() { // TEST!!!!!
        // Shooter motor config
        shooterDrive = new WPI_TalonFX(Constants.ShooterMotorID);
        shooterDrive.setInverted(TalonFXInvertType.Clockwise);
        shooterDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        shooterDrive.setNeutralMode(NeutralMode.Coast);
        shooterDrive.configVoltageCompSaturation(12.0, 100);
        shooterDrive.enableVoltageCompensation(true);
        
        limelightShooting = true;
        
        // Don't mess with these values, suck a big one Evan
        shooterDrive.config_kF(0, 0, 100);
        shooterDrive.config_kD(0, 0, 100);
        shooterDrive.config_kP(0, 0.015, 100);

        // Hood motor config
        hoodMotor = new WPI_TalonFX(Constants.HoodMotorID);
        hoodMotor.config_kP(0, 0.07625, 100);
        hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        hoodMotor.setSelectedSensorPosition(0, 0, 200);
        hoodMotor.setNeutralMode(NeutralMode.Brake);
        
        rotComplete = false;
        hoodAtOrigin = false;

        homocideTheBattery = false;
        
        leftHoodSwitch = new DigitalInput(Constants.LeftHoodSwitchID);
        rightHoodSwitch = new DigitalInput(Constants.RightHoodSwitchID);
        
        shooterMotorFeedforward = new SimpleMotorFeedforward(0.74223, 0.11079, 0.0081274);

        // shooterDrive.config_kF(0, (1023 * .8) / 20000, 100);
        // shooterDrive.config_kD(0, 15, 100);
        // shooterDrive.config_kP(0, 0.45, 100);

        // shootSpeedTable = new InterpolatingTreeMap<>();
        //     shootSpeedTable.put(7.0, 14665.0);
        //     shootSpeedTable.put(8.37, 13808.0);
        //     shootSpeedTable.put(9.37, 14791.0);
        //     shootSpeedTable.put(10.37, 15467.0);
        //     shootSpeedTable.put(11.7, 16081.0);
        //     shootSpeedTable.put(12.37, 16511.0);
        //     shootSpeedTable.put(13.07, 17064.0);
        //     shootSpeedTable.put(15.0, 16912.0);
        //     shootSpeedTable.put(20.0, 18000.0);

        // shootAngleTable = new InterpolatingTreeMap<>();
        //     shootAngleTable.put(7.0, 4302.0);
        //     shootAngleTable.put(8.37, 13542.0);
        //     shootAngleTable.put(9.37, 16542.0);
        //     shootAngleTable.put(10.37, 19071.0);
        //     shootAngleTable.put(11.37, 20819.0);
        //     shootAngleTable.put(12.37, 22348.0);
        //     shootAngleTable.put(13.07, 24532.0);
        //     shootAngleTable.put(15.0, 17424.0);
        //     shootAngleTable.put(20.0, 17868.0);

    // New values (3/29/22): Distances percieved by limelight were shorter than reality
    shootSpeedTable = new InterpolatingTreeMap<>();
        shootSpeedTable.put(6.8, 14644.0); //14344.0
        shootSpeedTable.put(8.0, 14544.0);  //14344.0
        shootSpeedTable.put(9.03, 14997.0);
        shootSpeedTable.put(10.07, 15127.0);
        shootSpeedTable.put(10.70, 15595.0);
        shootSpeedTable.put(11.04, 16064.0);
        shootSpeedTable.put(12.37, 16411.0); 
        // shootSpeedTable.put(12.99, 16100.0);
        shootSpeedTable.put(13.13, 17631.0);
        shootSpeedTable.put(14.08, 17849.0);
        shootSpeedTable.put(15.16, 18590.0);
        shootSpeedTable.put(16.07, 19000.0);
        shootSpeedTable.put(17.03, 20000.0);//TODO: test this shot again

    shootAngleTable = new InterpolatingTreeMap<>();
        shootAngleTable.put(6.8, 10484.0);
        shootAngleTable.put(8.0, 17037.0);
        shootAngleTable.put(9.03, 17474.0);
        shootAngleTable.put(10.07, 19658.0);
        shootAngleTable.put(10.70, 21115.0);
        shootAngleTable.put(11.04, 21952.0);
        shootAngleTable.put(12.37, 22348.0);
        // shootAngleTable.put(12.99, 25100.0);
        shootAngleTable.put(13.13, 25481.0);
        shootSpeedTable.put(14.08, 27481.0);
        shootAngleTable.put(15.16, 29481.0);
        shootSpeedTable.put(16.07, 30000.0);
        shootAngleTable.put(17.23, 32000.0);//TODO: test this shot again

    }
    @Config
    public void setFakeDistance(double _input){
        fakeDistance = _input;
    }
    
    public void shooterPeriodic() {

        //System.out.println(homocideTheBattery);
        distance = (((103.0 - 36.614) / Math.tan(Math.toRadians(35.0 + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0)))) + 12.4) / 12;
        // System.out.println(shooterDrive.getSelectedSensorVelocity(0) + " " + shooterSpeed + " " + shooterAtSpeed());
        if (!hoodAtOrigin) {
            rezeroHood();
        } else if (hoodAtOrigin) {
            hoodAngle = calculateShooterAngle();
            shooterSpeed = calculateShooterSpeed();
            
            rotateHood(hoodAngle + hoodAngleOffset);

        }
        
        //bloop shot
        
        if (Robot.xbox.getLeftBumper()) {
            shooterDrive.set(ControlMode.Velocity, 12000, DemandType.ArbitraryFeedForward, (shooterMotorFeedforward.calculate(12000 / 2048.0 * 10.0) / 12.0 - 0.03));
        } else if (Robot.INTAKE.shootNow || Robot.xbox.getLeftTriggerAxis() > 0 || (homocideTheBattery && !Robot.INTAKE.topLimitSwitch.get())) { ///SJV dont like this logic completely
            // if (Robot.xbox.getLeftTriggerAxis() > 0 || homocideTheBattery) {
            shooterDrive.set(ControlMode.Velocity, shooterSpeed, DemandType.ArbitraryFeedForward, (shooterMotorFeedforward.calculate(shooterSpeed / 2048.0 * 10.0) / 12.0 - 0.03));
        } else {
            shooterDrive.set(0);
        }
    }
    
    public boolean shooterAtSpeed() {
        double velocityError = Math.abs(shooterDrive.getSelectedSensorVelocity(0) - shooterSpeed);
        // TODO: IF TOO MUCH VARIATION IN SHOTS, NARROW THESE VALUES SO IT STILL SHOOTS
        // if (velocityError <= shooterSpeed * 0.03) { // Checks if the shooter is spinning fast enough to shoot *see intake file*
        if(shooterDrive.getSelectedSensorVelocity(0)<=shooterSpeed*1.02 && shooterDrive.getSelectedSensorVelocity() >= shooterSpeed * 0.96){
            return true;

        } else {
            return false;
        }
            
    }
        
    // Find shooter angle based on distance from hub
    // TODO: Show distance in shuffleboard
    @Log
    public double calculateShooterAngle() { //put stuff here
        if ((NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) 
        && limelightShooting  && !bloopShot && !Robot.xbox.getLeftBumper()) {

            double calculatedHoodAngle;
            // distance = (((103.0 - 36.614) / Math.tan(Math.toRadians(angle))) + 12.4) / 12;
            //distance = fakeDistance;//sjv
            // System.out.println(distance); 
            if(fancyShot){   
                calculatedHoodAngle =  shootAngleTable.get(distance);
                    
            } else {
                calculatedHoodAngle = -37840 + 10740 * distance - 686.3 * (Math.pow(distance, 2)) + 15.22 * (Math.pow(distance, 3));

            }
            // angle = -14.75x^3 + 497.7x^2 -3726x + 11270, x = distance
                
            // angle = 25460.0 - 6868.0 * distance + 908.4 * (Math.pow(distance, 2)) - 29.87 * (Math.pow(distance, 3)) + hoodAngleOffset;
            if (0 < calculatedHoodAngle && calculatedHoodAngle < 32000) {
                return calculatedHoodAngle;
            } else { 
                return hoodAngle;
            }
                
        } else if (bloopShot || Robot.xbox.getLeftBumper()){
            return 15975.00;
        } else if (!limelightShooting) {
            return hoodAngle;
        } else {
            return hoodAngle;
        }
    }
    
    // Find shooter speed based on shooter angle
    @Log
    public double calculateShooterSpeed() {
        if ((NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) && 
        limelightShooting && !bloopShot && !Robot.xbox.getLeftBumper()) {  
            
            //distance = fakeDistance;
            // shooterSpeed = -2.846x^3 + 116.5x^2 -1196x + 18110, x = distance
            double _methodSpeed = (358.7*distance) + 11760 + shooterSpeedOffset;
            
            //shooterSpeed = -43.49 * (Math.pow(distance, 3)) + 1382.0 * (Math.pow(distance, 2)) -13801.0 * distance + 58310.0;
            
            if(fancyShot){
                
                return shootSpeedTable.get(distance) + shooterSpeedOffset;
                
            } else{
                return _methodSpeed;

            }

        } else if (!limelightShooting){
            return shooterSpeed;
        
        } else {
            // System.out.println((((103.0 - 36.614) / Math.tan(Math.toRadians(35.0 + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0)))) + 12.4) / 12);
            return shooterSpeed;
            }
    }
    @Log
    public double getIPSpeed(){
        return shooterSpeed;
    }

    public void rotateHood(double angle) {
        hoodMotor.set(ControlMode.Position, angle);
    }
        
    private void rezeroHood() { // check default on hood switches
        
    if (!leftHoodSwitch.get() || !rightHoodSwitch.get()) {
        hoodAtOrigin = true;        //**this is also gross bc nested if statements | yea it is gross
        hoodMotor.set(ControlMode.PercentOutput, 0);
         hoodMotor.setSelectedSensorPosition(0, 0, 20);

        } else if ((leftHoodSwitch.get() || rightHoodSwitch.get()) && !hoodAtOrigin && !rotComplete) {
            hoodMotor.set(ControlMode.Position, 3000);

            if (hoodMotor.getSelectedSensorPosition(0) >= 2300) {
            rotComplete = true; 

                if (rotComplete){
                    hoodMotor.set(ControlMode.PercentOutput, -0.1);
                }
             }
         }
    }

    public void blockShot(boolean shotBlock) {
        if (shotBlock) {
            hoodMotor.setSelectedSensorPosition(100, 0, 20);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            Robot.SWERVEDRIVE.defensiveStop = false;
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
            Robot.SWERVEDRIVE.defensiveStop = true;
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
    
    @Config.NumberSlider(tabName = "CompetitionLogger", name = "Set Shooter Angle Offset", defaultValue = 0, min = -5000, max = 5000, blockIncrement = 100, rowIndex = 5, columnIndex = 0, height = 1, width = 2)
    public void setHoodAngleOffset(double targetAngleOffset) {
        hoodAngleOffset = targetAngleOffset;
    }

    
    //SJV: ONCE WE FIGURE OUT OUR SHOOTER ANGLE AND SPEED MAKE BOOLEAN FOR EACH OF THE SHOOTER SPEEDS AND PUT IT ON THE COMPETITION LOGGER
    @Config.NumberSlider(name = "Set Shooter Speed", defaultValue = 15000, min = 0, max = 18000, blockIncrement = 1000, rowIndex = 0, columnIndex = 2, height = 1, width = 2)
    public void setShooterSpeed(double targetVelocity) {
        shooterSpeed = targetVelocity;
        
    }
    
    @Config.NumberSlider(tabName = "CompetitionLogger", name = "Set Shooter Speed Offset", defaultValue = 0, min = -5000, max = 5000, rowIndex = 2, columnIndex = 5, blockIncrement = 100, height = 1, width = 2)
    public void setShooterSpeedOffset(double targetOffest) {
        shooterSpeedOffset = targetOffest;
    }


    
    // @Log(rowIndex = 1, columnIndex = 0)
    // public boolean getLeftHoodLimit() {
    //     return leftHoodSwitch.get();
    // }

    // @Log(rowIndex = 1, columnIndex = 1)
    // public boolean getRightHoodLimit() {
    //     return rightHoodSwitch.get();
    // }

}

// public double shootTicksToVMS(){
//     return -.0000000544*Math.pow(shooterSpeed+shooterSpeedOffset, 2) + .002423*(shooterSpeed+shooterSpeedOffset) - 10.06;
// }

// public Rotation2d hoodAngleTicksToActual() {
//     return new Rotation2d((-.001875 * (hoodAngle + hoodAngleOffset) + 77.5)*Math.PI/180);
// }

// public double vbo(){
//     return hoodAngleTicksToActual().getCos() * shootTicksToVMS();
// }

// private Rotation2d getAlpha(){
//     // System.out.println(SwerveMap.GYRO.getRotation2d().plus(new Rotation2d(Robot.SWERVEDRIVE.limelightTX())).getDegrees());
//     return SwerveMap.GYRO.getRotation2d().plus(new Rotation2d(Robot.SWERVEDRIVE.limelightTX()));
// }


// private Rotation2d getBeta(){

//     if(90 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=180){
//         return getAlpha().minus(new Rotation2d(Math.PI/2));
//     } else if(-180 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=-90){
//         return getAlpha().unaryMinus().minus(new Rotation2d(Math.PI/2));
//     } else if(-90 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=0){
//         return getAlpha().plus(new Rotation2d(Math.PI/2)); 
//     } else if(0 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=90){
//         return getAlpha().unaryMinus().plus(new Rotation2d(Math.PI/2)); 
//     } else {
//         return new Rotation2d(0);
//     }
// }

// public Rotation2d getPhi(){
//     if(fancyShot){
//         if(90 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=180){
//             return getBeta().unaryMinus().plus(getGamma());
//         } else if(-180 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=-90){
//             return getBeta().minus(getGamma());
//         } else if(-90 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=0){
//             return getBeta().minus(getGamma());
//         } else if(0 <= getAlpha().getDegrees() && getAlpha().getDegrees() <=90){
//             return getBeta().unaryMinus().plus(getGamma());
//         } else {
//             return new Rotation2d(0);
//         }
//     } else {
//         return new Rotation2d(0);
//     }
// }

// private Rotation2d getGamma(){
//     return new Rotation2d(Math.atan(getVnx()/getVny()));
// }

// public double getVnx(){
//     System.out.println("yVelocity: " + Robot.SWERVEDRIVE.velocities.get(1));
//     System.out.println("yVelocitx: " + Robot.SWERVEDRIVE.velocities.get(0));
//     return vbo()*getBeta().getCos() + Math.signum(getAlpha().minus(new Rotation2d(Math.PI/2)).getCos())*Robot.SWERVEDRIVE.velocities.get(1);


// }

// public double getVny(){
//     return vbo()*getBeta().getSin() + Math.signum(getAlpha().minus(new Rotation2d(Math.PI/2)).getSin())*Robot.SWERVEDRIVE.velocities.get(0);
// }

// public double getVnew(){
//     return Math.sqrt(Math.pow(getVnx(),2) + Math.pow(getVny(),2));
// }

// public double getVnewTicksoffset(){

//     return (getVnew()+1.552)/0.0148 - shooterSpeedOffset - shooterSpeed;
// }

