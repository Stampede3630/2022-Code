package frc.robot.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveMap {
    public static AHRS GYRO;



    

    public static TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    public static SimpleMotorFeedforward driveMotorFeedforward = new SimpleMotorFeedforward(SwerveConstants.kS, SwerveConstants.kV, SwerveConstants.kA);
    public static final SwerveModule FrontRightSwerveModule = new SwerveModule(
        new DriveMotor(SwerveConstants.FRDriveID, SwerveConstants.FRInvertType, SwerveConstants.FRDriveGains), 
        new SteeringMotor(SwerveConstants.FRSteerID, SwerveConstants.FRSteerGains), 
        new SteeringSensor(SwerveConstants.FRSensorID,SwerveConstants.FRSensorOffset));
    public static final SwerveModule FrontLeftSwerveModule = new SwerveModule(
        new DriveMotor(SwerveConstants.FLDriveID,SwerveConstants.FLInvertType, SwerveConstants.FLDriveGains),
        new SteeringMotor(SwerveConstants.FLSteerID, SwerveConstants.FLSteerGains), 
        new SteeringSensor(SwerveConstants.FLSensorID,SwerveConstants.FLSensorOffset));
    public static final SwerveModule BackRightSwerveModule = new SwerveModule(
        new DriveMotor(SwerveConstants.BRDriveID,SwerveConstants.BRInvertType, SwerveConstants.BRDriveGains) , 
        new SteeringMotor(SwerveConstants.BRSteerID, SwerveConstants.BRSteerGains), 
        new SteeringSensor(SwerveConstants.BRSensorID,SwerveConstants.BRSensorOffset));
    public static final SwerveModule BackLeftSwerveModule = new SwerveModule(
        new DriveMotor(SwerveConstants.BLDriveID,SwerveConstants.BLInvertType, SwerveConstants.BLDriveGains), 
        new SteeringMotor(SwerveConstants.BLSteerID, SwerveConstants.BLSteerGains), 
        new SteeringSensor(SwerveConstants.BLSensorID,SwerveConstants.BLSensorOffset));

    public static Rotation2d getRobotAngle(){
        return GYRO.getRotation2d();
        //return new Rotation2d(-Math.toRadians(GYRO.getAngle()));
    }
    //SJV MAYBE DESIGN A METHOD THAT SETS IT TO CANCODER DRIVING?
    public static void checkAndSetSwerveCANStatus(){
        FrontRightSwerveModule.setSWERVEMODULECANStatusFrames();
        BackRightSwerveModule.setSWERVEMODULECANStatusFrames();
        FrontLeftSwerveModule.setSWERVEMODULECANStatusFrames();
        BackLeftSwerveModule.setSWERVEMODULECANStatusFrames();
    }
    public static void driveRobotInit() {
        FrontRightSwerveModule.swerveRobotInit();
        BackRightSwerveModule.swerveRobotInit();
        FrontLeftSwerveModule.swerveRobotInit();
        BackLeftSwerveModule.swerveRobotInit();
    }

    public static void checkAndZeroSwerveAngle() {
        FrontRightSwerveModule.zeroSwerveAngle();
        BackRightSwerveModule.zeroSwerveAngle();
        FrontLeftSwerveModule.zeroSwerveAngle();
        BackLeftSwerveModule.zeroSwerveAngle();
    }


    public static class SteeringMotor extends WPI_TalonFX{  
        public SwerveConstants.Gains kGAINS;

        public SteeringMotor(int _talonID, SwerveConstants.Gains _gains) {
            super(_talonID, "Swerve");
            kGAINS = _gains;
        }
    }

    public static class DriveMotor extends WPI_TalonFX{
        public TalonFXInvertType kWheelDirectionType;
        public SwerveConstants.Gains kGAINS;
        public DriveMotor (int _talonID, TalonFXInvertType _direction, SwerveConstants.Gains _gains){
            super(_talonID, "Swerve");
            kWheelDirectionType = _direction;
            kGAINS=_gains;
        }
    }

    public static class SteeringSensor extends CANCoder{
        public double kOffsetDegrees;

        public SteeringSensor (int _sensorID, double _offsetDegrees){
            super(_sensorID, "Swerve");
            kOffsetDegrees = _offsetDegrees;
         }       
    }

    

    /**
     * Helpful hints:
     * 1. when determining your steering motor offsets first rotate 
     *    all modules to face a certain direction (inward/outward/left/right)
     * 2. Once that's done make sure you determine which drive motors need to go
     *    clockwise positive/negative
     * 3. NOW, ur ready to play with the offsets
     * 4. Use phoenix tuner to determin PID coefficients for EACH wheel, each wheel may
     *    may be slightly to vastly different
     * 
     */
    public static class SwerveModule {
        public final SteeringMotor mSteeringMotor;
        public final SteeringSensor mSteeringSensor;
        public final DriveMotor mDriveMotor;
        public boolean hasSwerveZeroingOccurred=false;
        public double swerveZeroingRetryCount = 0;
        public  StatorCurrentLimitConfiguration steerCurrentLimitConfigurationEnable;
        public  StatorCurrentLimitConfiguration steerCurrentLimitConfigurationDisable;
        public  StatorCurrentLimitConfiguration driveCurrentLimitConfigurationEnable;
        public  StatorCurrentLimitConfiguration driveCurrentLimitConfigurationDisable;

        public SwerveModule (DriveMotor _DriveMotor, SteeringMotor _SteeringMotor, SteeringSensor _SteeringSensor){
            mSteeringMotor = _SteeringMotor;
            mSteeringSensor = _SteeringSensor;
            mDriveMotor = _DriveMotor;

            steerCurrentLimitConfigurationEnable  = new StatorCurrentLimitConfiguration();
            steerCurrentLimitConfigurationDisable = new StatorCurrentLimitConfiguration();
            driveCurrentLimitConfigurationEnable  = new StatorCurrentLimitConfiguration();
            driveCurrentLimitConfigurationDisable = new StatorCurrentLimitConfiguration();

            steerCurrentLimitConfigurationEnable.enable = true;
            steerCurrentLimitConfigurationEnable.triggerThresholdCurrent = 60;
            steerCurrentLimitConfigurationEnable.triggerThresholdTime = .1;
            steerCurrentLimitConfigurationEnable.currentLimit = 30;

            steerCurrentLimitConfigurationDisable.enable = false;
            driveCurrentLimitConfigurationDisable.enable = false; 

            driveCurrentLimitConfigurationEnable.enable = true;
            driveCurrentLimitConfigurationEnable.triggerThresholdCurrent = 80;
            driveCurrentLimitConfigurationEnable.triggerThresholdTime = .1;
            driveCurrentLimitConfigurationEnable.currentLimit = 60;

                       
        }

        public void swerveRobotInit(){
            

            
            //Setup the drive motor, but first set EVERYTHING to DEFAULT
            //mDriveMotor.configFactoryDefault();

            mDriveMotor.setInverted(mDriveMotor.kWheelDirectionType);
            mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kDefaultTimeout);
            mDriveMotor.configStatorCurrentLimit(driveCurrentLimitConfigurationEnable, 1000);
            mDriveMotor.configStatorCurrentLimit(driveCurrentLimitConfigurationDisable, 1000);
            //mDriveMotor.config_kF(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kF, Constants.kDefaultTimeout);
            //mDriveMotor.config_kP(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kP, Constants.kDefaultTimeout);
            //mDriveMotor.config_kI(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kI, Constants.kDefaultTimeout);
            //mDriveMotor.config_kD(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kD, Constants.kDefaultTimeout);  
            //mDriveMotor.config_IntegralZone(0, mDriveMotor.kGAINS.kIzone);

            //Setup the Steering Sensor
            CANCoderConfiguration myCanCoderConfig = new CANCoderConfiguration();
            myCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            myCanCoderConfig.sensorDirection = false;
            myCanCoderConfig.magnetOffsetDegrees = mSteeringSensor.kOffsetDegrees;
            myCanCoderConfig.sensorCoefficient = 360/4096;
            myCanCoderConfig.unitString = "deg";
            myCanCoderConfig.sensorTimeBase= SensorTimeBase.PerSecond;
            myCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
            
            
            if(mSteeringSensor.configAllSettings(myCanCoderConfig,1000)==ErrorCode.OK){
                System.out.println("CANCoder " + mSteeringSensor.getDeviceID() + " configured.");
            } else {
                System.out.println("WARNING! CANCoder " + mSteeringSensor.getDeviceID() + " NOT configured correctly!");
            }
            mSteeringSensor.setPositionToAbsolute(1000);

            //Setup the the closed-loop PID for the steering module loop
            
            TalonFXConfiguration mySteeringMotorConfiguration = new TalonFXConfiguration();
            mySteeringMotorConfiguration.feedbackNotContinuous = false;
            mySteeringMotorConfiguration.slot0.kP = mSteeringMotor.kGAINS.kP;
            mySteeringMotorConfiguration.slot0.kI = mSteeringMotor.kGAINS.kI;
            mySteeringMotorConfiguration.slot0.kD = mSteeringMotor.kGAINS.kD;
            mySteeringMotorConfiguration.slot0.kF = mSteeringMotor.kGAINS.kF;
            mySteeringMotorConfiguration.slot0.allowableClosedloopError = SwerveConstants.kDefaultClosedLoopError;
            mySteeringMotorConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
            mySteeringMotorConfiguration.remoteFilter0.remoteSensorDeviceID= mSteeringSensor.getDeviceID();
            if(mSteeringMotor.configAllSettings(mySteeringMotorConfiguration,1000)==ErrorCode.OK)   {
                System.out.println("Steer Motor " + mSteeringMotor.getDeviceID() + " configured.");
            } else {
                System.out.println("WARNING! Steer Motor  " + mSteeringMotor.getDeviceID() + " NOT configured correctly!");
            }
            mSteeringMotor.configStatorCurrentLimit(steerCurrentLimitConfigurationEnable, 1000);
            mSteeringMotor.configStatorCurrentLimit(steerCurrentLimitConfigurationDisable, 1000);
            mSteeringMotor.setInverted(TalonFXInvertType.Clockwise);
            mSteeringMotor.configSelectedFeedbackCoefficient(1/SwerveConstants.TICKSperTALONFX_STEERING_DEGREE,0,Constants.kDefaultTimeout);
            mSteeringMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,Constants.kDefaultTimeout);
        }

        public void setSWERVEMODULECANStatusFrames(){
            if(mDriveMotor.hasResetOccurred()){
                int mycounter = 0;
                
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100)!=ErrorCode.OK) {mycounter++;}
                if(mDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100)!=ErrorCode.OK) {mycounter++;}
                System.out.println("RESET DETECTED FOR TALONFX " + mDriveMotor.getDeviceID() + " Errors: " + mycounter);
            }
            if(mSteeringMotor.hasResetOccurred()){
                int mycounter = 0;
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255,1000) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255,100) !=ErrorCode.OK) {mycounter++;}
                if(mSteeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255,100) !=ErrorCode.OK) {mycounter++;}
                System.out.println("RESET DETECTED FOR TALONFX " + mSteeringMotor.getDeviceID()+ " Errors: " + mycounter);
            }
        }

        public void enableCurrentLimiting(){
            mDriveMotor.configStatorCurrentLimit(driveCurrentLimitConfigurationEnable, 250);
            mSteeringMotor.configStatorCurrentLimit(steerCurrentLimitConfigurationEnable, 250);
        }

        public void disableCurrentLimiting(){
            mDriveMotor.configStatorCurrentLimit(driveCurrentLimitConfigurationDisable, 250);
            mSteeringMotor.configStatorCurrentLimit(steerCurrentLimitConfigurationDisable, 250);
        }

        public void swerveDisabledInit(){
            mDriveMotor.setNeutralMode(NeutralMode.Coast);
            mSteeringMotor.setNeutralMode(NeutralMode.Coast);
        }
        public void swerveEnabledInit(){
            mDriveMotor.setNeutralMode(NeutralMode.Brake);
            mSteeringMotor.setNeutralMode(NeutralMode.Brake);
        }

        public void zeroSwerveAngle() {
            if(!hasSwerveZeroingOccurred && swerveZeroingRetryCount >=10) {
                if(mSteeringSensor.setPositionToAbsolute(1000)==ErrorCode.OK){
                    if((int) mSteeringSensor.configGetParameter(ParamEnum.eSensorInitStrategy,0, 1000) == SensorInitializationStrategy.BootToZero.value) {
                        if(mSteeringSensor.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition,1000).value!=ErrorCode.OK.value) {
                            System.out.println("ERROR: COULDN'T SET THE INITIALIZATION STRATEGY! CANCODER: " + mSteeringSensor.getDeviceID());
                        } else {
                            System.out.println("ERROR: INITIALIZATION STRATEGY SET! REBOOT ROBOT! CANCODER: " + mSteeringSensor.getDeviceID());
                            mSteeringMotor.setSelectedSensorPosition(mSteeringSensor.getPosition(),0,1000);
                            System.out.println("ERROR: ZEROED SENSOR VALUES FOR CANCODER ANYWAY " + mSteeringSensor.getDeviceID() + " " + mSteeringSensor.getPosition());
                        }
                    } else if(hasSwerveZeroingOccurred || mSteeringMotor.setSelectedSensorPosition(mSteeringSensor.getPosition(),0,1000).value==0){
                        hasSwerveZeroingOccurred = true;
                        System.out.println("Zeroed Sensor values for " + mSteeringSensor.getDeviceID() + " " + mSteeringSensor.getPosition());
                        mSteeringSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255, 1000);
                    } else {
                        System.out.println("ERROR: COULDNT ZERO MODULE: " + mSteeringMotor.getDeviceID());
                    }
                }  else {
                    swerveZeroingRetryCount = 0;
                    System.out.println("ERROR: COULDNT SET POSITION TO ABSOLUTE! CANCODER: " + mSteeringSensor.getDeviceID());
                }
            } else if (!hasSwerveZeroingOccurred && swerveZeroingRetryCount <10) {
                swerveZeroingRetryCount++;
            }
        }

        public void REzeroSwerveAngle() {
            mSteeringSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 1000);
            mSteeringMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,0,Constants.kDefaultTimeout);       
        }

        public SwerveModuleState getState() {

            return new SwerveModuleState( 
                mDriveMotor.getSelectedSensorVelocity()*
                SwerveConstants.METERSperWHEEL_REVOLUTION/(SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION*
                SwerveConstants.SECONDSper100MS), new Rotation2d(Math.toRadians(mSteeringMotor.getSelectedSensorPosition())));
        }


        public void setDesiredState(SwerveModuleState desiredState){
            SwerveModuleState kState = optimize(desiredState, new Rotation2d(Math.toRadians(mSteeringMotor.getSelectedSensorPosition())));
            double convertedspeed = kState.speedMetersPerSecond*(SwerveConstants.SECONDSper100MS)*SwerveConstants.DRIVE_MOTOR_TICKSperREVOLUTION/(SwerveConstants.METERSperWHEEL_REVOLUTION);           
            setSteeringAngle(kState.angle.getDegrees());
            
            if (Robot.CHARACTERIZE_ROBOT){

                mDriveMotor.set(ControlMode.PercentOutput, kState.speedMetersPerSecond/SwerveConstants.MAX_SPEED_METERSperSECOND); 
            } else if(SwerveConstants.kS == 0 && SwerveConstants.kV == 0) {
                
                mDriveMotor.set(ControlMode.Velocity, convertedspeed);
            } else {
                mDriveMotor.setVoltage(driveMotorFeedforward.calculate(kState.speedMetersPerSecond));
            }
            
        }
        
        /** 
         * This method takes in setAngle in DEGREES, 
         * 
         * compares that angle with the current position of 
         * the swerve module and decides which direction to 
         * rotate the module.
         * 
         * The angle is then converted to sensor units (4096 
         * equals 1 full rotation) units equals and fed
         * to the steering motor to update.
         * @param _angle (IN DEGREES)
         */
        public void setSteeringAngle(double _angle){
            //double newAngleDemand = _angle;
            double currentSensorPosition = mSteeringMotor.getSelectedSensorPosition();
            double remainder = Math.IEEEremainder(currentSensorPosition, 360);
            double newAngleDemand = _angle + currentSensorPosition -remainder;
           
            //System.out.println(mSteeringMotor.getSelectedSensorPosition()-remainder );
            if(newAngleDemand - currentSensorPosition > 180.1){
                  newAngleDemand -= 360;
              } else if (newAngleDemand - currentSensorPosition < -180.1){
                  newAngleDemand += 360;
              }
              
            mSteeringMotor.set(ControlMode.Position, newAngleDemand );
        }
      
        public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
          var delta = desiredState.angle.minus(currentAngle);
          if (Math.abs(delta.getDegrees()) > 90.0) {  //SJV: If this doesn'twork try 360
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
          } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
          }
        }
      
    }

}

    
    

