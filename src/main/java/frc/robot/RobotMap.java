package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class RobotMap {
    public SwerveModule FrontRightSwerveModule = new SwerveModule(new DriveMotor(Constants.FRDriveID, TalonFXInvertType.CounterClockwise), new SteeringMotor(Constants.FRSteerID, Constants.FRGains), new SteeringSensor(Constants.FRSensorID,0));
    public SwerveModule FrontLeftSwerveModule = new SwerveModule(new DriveMotor(Constants.FLDriveID,TalonFXInvertType.CounterClockwise), new SteeringMotor(Constants.FLSteerID, Constants.FLGains), new SteeringSensor(Constants.FLSensorID,0));
    public SwerveModule BackRightSwerveModule = new SwerveModule(new DriveMotor(Constants.BRDriveID,TalonFXInvertType.CounterClockwise) , new SteeringMotor(Constants.BRSteerID, Constants.BRGains), new SteeringSensor(Constants.BRSensorID,0));
    public SwerveModule BackLeftSwerveModule = new SwerveModule(new DriveMotor(Constants.BLDriveID,TalonFXInvertType.CounterClockwise), new SteeringMotor(Constants.BLSteerID, Constants.BLGains), new SteeringSensor(Constants.BLSensorID,0));
    
    
    private RobotMap() {
        FrontRightSwerveModule.init();
        BackRightSwerveModule.init();
        FrontLeftSwerveModule.init();
        BackLeftSwerveModule.init();
    }


    public class SteeringMotor extends WPI_TalonFX{  
        public Constants.Gains kGAINS;

        public SteeringMotor(int _talonID, Constants.Gains _gains) {
            super(_talonID);
            kGAINS = _gains;
        }
    }

    public class DriveMotor extends WPI_TalonFX{
        public TalonFXInvertType kWheelDirectionType;
        public DriveMotor (int _talonID, TalonFXInvertType _direction){
            super(_talonID);
            kWheelDirectionType = _direction;
        }
    }

    public class SteeringSensor extends CANCoder{
        public double kOffsetDegrees;

        public SteeringSensor (int _sensorID, int _offsetDegrees){
            super(_sensorID);
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
    public class SwerveModule {
        public final SteeringMotor kSteeringMotor;
        public final SteeringSensor kSteeringSensor;
        public final DriveMotor kDriveMotor;

        public SwerveModule (DriveMotor _DriveMotor, SteeringMotor _SteeringMotor, SteeringSensor _SteeringSensor){
            kSteeringMotor = _SteeringMotor;
            kSteeringSensor = _SteeringSensor;
            kDriveMotor = _DriveMotor;
                       
        }
        public void init(){
            //Setup the drive motor
            kDriveMotor.setInverted(kDriveMotor.kWheelDirectionType);

            //Setup the Steering Sensor
            kSteeringSensor.configSensorDirection(false);
            kSteeringSensor.configMagnetOffset(kSteeringSensor.kOffsetDegrees);

            //Setup the the closed-loop PID for the steering module loop
            
            kSteeringMotor.configFactoryDefault();
            kSteeringMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,0,Constants.kDefaultTimeout);
            kSteeringMotor.configAllowableClosedloopError(Constants.kDefaultPIDSlotID, Constants.kDefaultClosedLoopError, Constants.kDefaultTimeout);
            kSteeringMotor.config_kF(Constants.kDefaultPIDSlotID, kSteeringMotor.kGAINS.kF, Constants.kDefaultTimeout);
            kSteeringMotor.config_kP(Constants.kDefaultPIDSlotID, kSteeringMotor.kGAINS.kP, Constants.kDefaultTimeout);
            kSteeringMotor.config_kI(Constants.kDefaultPIDSlotID, kSteeringMotor.kGAINS.kI, Constants.kDefaultTimeout);
            kSteeringMotor.config_kD(Constants.kDefaultPIDSlotID, kSteeringMotor.kGAINS.kD, Constants.kDefaultTimeout);  
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
            double newAngleDemand;
            if(Math.abs(_angle*4096/360 - kSteeringSensor.getPosition()) > Math.abs((_angle - 180)*4096/360 - kSteeringSensor.getPosition())){
                newAngleDemand = _angle*4096/360 - kSteeringSensor.getPosition();
            } else {
                newAngleDemand = (_angle - 180)*4096/360 - kSteeringSensor.getPosition();
            }   
            
            kSteeringMotor.set(ControlMode.Position, newAngleDemand);
        }

        public double getSteeringAngle(){
            return kSteeringSensor.getAbsolutePosition();
        }
    }
    
}
