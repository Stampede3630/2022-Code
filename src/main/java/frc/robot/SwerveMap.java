package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveMap {
    public static AHRS GYRO;
    public static final SwerveModule FrontRightSwerveModule = new SwerveModule(
        new DriveMotor(Constants.FRDriveID, TalonFXInvertType.Clockwise, Constants.FRDriveGains), 
        new SteeringMotor(Constants.FRSteerID, Constants.FRSteerGains), 
        new SteeringSensor(Constants.FRSensorID,Constants.FRSensorOffset));
    public static final SwerveModule FrontLeftSwerveModule = new SwerveModule(
        new DriveMotor(Constants.FLDriveID,TalonFXInvertType.CounterClockwise, Constants.FLDriveGains),
        new SteeringMotor(Constants.FLSteerID, Constants.FLSteerGains), 
        new SteeringSensor(Constants.FLSensorID,Constants.FLSensorOffset));
    public static final SwerveModule BackRightSwerveModule = new SwerveModule(
        new DriveMotor(Constants.BRDriveID,TalonFXInvertType.Clockwise, Constants.BRDriveGains) , 
        new SteeringMotor(Constants.BRSteerID, Constants.BRSteerGains), 
        new SteeringSensor(Constants.BRSensorID,Constants.BRSensorOffset));
    public static final SwerveModule BackLeftSwerveModule = new SwerveModule(
        new DriveMotor(Constants.BLDriveID,TalonFXInvertType.CounterClockwise, Constants.BLDriveGains), 
        new SteeringMotor(Constants.BLSteerID, Constants.BLSteerGains), 
        new SteeringSensor(Constants.BLSensorID,Constants.BLSensorOffset));

    
    public static void driveRobotInit() {
        FrontRightSwerveModule.swerveRobotInit();
        BackRightSwerveModule.swerveRobotInit();
        FrontLeftSwerveModule.swerveRobotInit();
        BackLeftSwerveModule.swerveRobotInit();
        //ahrs = new AHRS(Port.kMXP);
    }

    public static class SteeringMotor extends WPI_TalonFX{  
        public Constants.Gains kGAINS;

        public SteeringMotor(int _talonID, Constants.Gains _gains) {
            super(_talonID);
            kGAINS = _gains;
        }
    }

    public static class DriveMotor extends WPI_TalonFX{
        public TalonFXInvertType kWheelDirectionType;
        public Constants.Gains kGAINS;
        public DriveMotor (int _talonID, TalonFXInvertType _direction, Constants.Gains _gains){
            super(_talonID);
            kWheelDirectionType = _direction;
            kGAINS=_gains;
        }
    }

    public static class SteeringSensor extends CANCoder{
        public double kOffsetDegrees;

        public SteeringSensor (int _sensorID, double _offsetDegrees){
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
    public static class SwerveModule {
        public final SteeringMotor mSteeringMotor;
        public final SteeringSensor mSteeringSensor;
        public final DriveMotor mDriveMotor;

        public SwerveModule (DriveMotor _DriveMotor, SteeringMotor _SteeringMotor, SteeringSensor _SteeringSensor){
            mSteeringMotor = _SteeringMotor;
            mSteeringSensor = _SteeringSensor;
            mDriveMotor = _DriveMotor;
                       
        }
        public void swerveRobotInit(){
            //Setup the drive motor
            mDriveMotor.setInverted(mDriveMotor.kWheelDirectionType);
            mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kDefaultTimeout);
            mDriveMotor.config_kF(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kF, Constants.kDefaultTimeout);
            mDriveMotor.config_kP(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kP, Constants.kDefaultTimeout);
            mDriveMotor.config_kI(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kI, Constants.kDefaultTimeout);
            mDriveMotor.config_kD(Constants.kDefaultPIDSlotID, mDriveMotor.kGAINS.kD, Constants.kDefaultTimeout);  
            mDriveMotor.config_IntegralZone(0, mDriveMotor.kGAINS.kIzone);
 
            //Setup the Steering Sensor
            mSteeringSensor.configSensorDirection(false);
            mSteeringSensor.configMagnetOffset(mSteeringSensor.kOffsetDegrees);
            mSteeringSensor.setPositionToAbsolute();
            //Setup the the closed-loop PID for the steering module loop
            
            mSteeringMotor.configFactoryDefault();
            mSteeringMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,0,Constants.kDefaultTimeout);
            mSteeringMotor.configRemoteFeedbackFilter(mSteeringSensor, 0);
            mSteeringMotor.configAllowableClosedloopError(Constants.kDefaultPIDSlotID, Constants.kDefaultClosedLoopError, Constants.kDefaultTimeout);
            mSteeringMotor.config_kF(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kF, Constants.kDefaultTimeout);
            mSteeringMotor.config_kP(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kP, Constants.kDefaultTimeout);
            mSteeringMotor.config_kI(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kI, Constants.kDefaultTimeout);
            mSteeringMotor.config_kD(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kD, Constants.kDefaultTimeout);  
        }
        public void swerveDisabledInit(){
            mDriveMotor.setNeutralMode(NeutralMode.Coast);
            mSteeringMotor.setNeutralMode(NeutralMode.Coast);
        }
        public void swerveEnabledInit(){
            mDriveMotor.setNeutralMode(NeutralMode.Brake);
            mSteeringMotor.setNeutralMode(NeutralMode.Brake);
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(mDriveMotor.getSelectedSensorVelocity()*10, new Rotation2d(Math.toRadians(mSteeringSensor.getPosition()*360/4096)));
        }

        public void setDesiredState(SwerveModuleState desiredState){
            SwerveModuleState kState = optimize(desiredState, new Rotation2d(Math.toRadians(mSteeringSensor.getPosition())));
            double convertedspeed = kState.speedMetersPerSecond*(Constants.SECONDSper100MS)*Constants.TICKSperREVOLUTION/(Constants.METERSperREVOLUTION);           
            setSteeringAngle(kState.angle.getDegrees());
            mDriveMotor.set(ControlMode.Velocity, convertedspeed);
            
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
            newAngleDemand = _angle*4096/360;
            if(newAngleDemand - mSteeringMotor.getSelectedSensorPosition() > 2200){
                newAngleDemand -= 4096;
            } else if (newAngleDemand - mSteeringMotor.getSelectedSensorPosition() < -2200){
                newAngleDemand += 4096;
            }
            mSteeringMotor.set(ControlMode.Position, newAngleDemand);
        }

        public double getSteeringAngle(){
            return mSteeringSensor.getAbsolutePosition();
        }

        public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
          var delta = desiredState.angle.minus(currentAngle);
          if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
          } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
          }
        }
      
    
    }
    
}
