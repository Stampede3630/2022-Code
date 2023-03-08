package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveMap {
    public static AHRS GYRO;
    public static TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    public static SimpleMotorFeedforward driveMotorFeedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
    public static final SwerveModule FrontRightSwerveModule = new SwerveModule(
        new DriveMotor(Constants.FRDriveID, Constants.FRInvertType, Constants.FRDriveGains), 
        new SteeringMotor(Constants.FRSteerID, Constants.FRSteerGains), 
        new SteeringSensor(Constants.FRSensorID,Constants.FRSensorOffset));
    public static final SwerveModule FrontLeftSwerveModule = new SwerveModule(
        new DriveMotor(Constants.FLDriveID,Constants.FLInvertType, Constants.FLDriveGains),
        new SteeringMotor(Constants.FLSteerID, Constants.FLSteerGains), 
        new SteeringSensor(Constants.FLSensorID,Constants.FLSensorOffset));
    public static final SwerveModule BackRightSwerveModule = new SwerveModule(
        new DriveMotor(Constants.BRDriveID,Constants.BRInvertType, Constants.BRDriveGains) , 
        new SteeringMotor(Constants.BRSteerID, Constants.BRSteerGains), 
        new SteeringSensor(Constants.BRSensorID,Constants.BRSensorOffset));
    public static final SwerveModule BackLeftSwerveModule = new SwerveModule(
        new DriveMotor(Constants.BLDriveID,Constants.BLInvertType, Constants.BLDriveGains), 
        new SteeringMotor(Constants.BLSteerID, Constants.BLSteerGains), 
        new SteeringSensor(Constants.BLSensorID,Constants.BLSensorOffset));

    public static Rotation2d getRobotAngle(){
        return GYRO.getRotation2d();
        //return new Rotation2d(-Math.toRadians(GYRO.getAngle()));
    }
    public static void driveRobotInit() {
        FrontRightSwerveModule.swerveRobotInit();
        BackRightSwerveModule.swerveRobotInit();
        FrontLeftSwerveModule.swerveRobotInit();
        BackLeftSwerveModule.swerveRobotInit();
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

            //Setup the drive motor, but first set EVERYTHING to DEFAULT
            mDriveMotor.configFactoryDefault();
            
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
            mSteeringMotor.configFeedbackNotContinuous(false, Constants.kDefaultTimeout);
            mSteeringMotor.configSelectedFeedbackCoefficient(1/Constants.TICKSperTALONFX_DEGREE,0,Constants.kDefaultTimeout);
            mSteeringMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,Constants.kDefaultTimeout);

            mSteeringMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,1,Constants.kDefaultTimeout);
            mSteeringMotor.configRemoteFeedbackFilter(mSteeringSensor, 0);
            mSteeringMotor.configSelectedFeedbackCoefficient(Constants.STEERING_SENSOR_DEGREESperTICKS, 1, Constants.kDefaultTimeout);
            mSteeringMotor.configAllowableClosedloopError(Constants.kDefaultPIDSlotID, Constants.kDefaultClosedLoopError, Constants.kDefaultTimeout);
            mSteeringMotor.config_kF(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kF, Constants.kDefaultTimeout);
            mSteeringMotor.config_kP(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kP, Constants.kDefaultTimeout);
            mSteeringMotor.config_kI(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kI, Constants.kDefaultTimeout);
            mSteeringMotor.config_kD(Constants.kDefaultPIDSlotID, mSteeringMotor.kGAINS.kD, Constants.kDefaultTimeout);  
            zeroSwerveAngle();
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
            mSteeringMotor.setSelectedSensorPosition(mSteeringSensor.getAbsolutePosition(),0,Constants.kDefaultTimeout);
        }
        public void REzeroSwerveAngle() {
            mSteeringMotor.setSelectedSensorPosition(mSteeringSensor.getAbsolutePosition()-mSteeringMotor.getSelectedSensorPosition(),0,Constants.kDefaultTimeout);
        }

        public SwerveModuleState getState() {

            return new SwerveModuleState( 
                mDriveMotor.getSelectedSensorVelocity()*
                Constants.METERSperWHEEL_REVOLUTION/(Constants.DRIVE_MOTOR_TICKSperREVOLUTION*
                Constants.SECONDSper100MS), new Rotation2d(Math.toRadians(mSteeringMotor.getSelectedSensorPosition())));
        }
        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                    mDriveMotor.getSelectedSensorPosition() / Constants.DRIVE_MOTOR_TICKSperREVOLUTION
                            * Constants.METERSperWHEEL_REVOLUTION,
                    new Rotation2d(Math.toRadians(mSteeringMotor.getSelectedSensorPosition())));
        }
        public void setDesiredState(SwerveModuleState desiredState){
            SwerveModuleState kState = optimize(desiredState, new Rotation2d(Math.toRadians(mSteeringMotor.getSelectedSensorPosition())));
            double convertedspeed = kState.speedMetersPerSecond*(Constants.SECONDSper100MS)*Constants.DRIVE_MOTOR_TICKSperREVOLUTION/(Constants.METERSperWHEEL_REVOLUTION);           
            setSteeringAngle(kState.angle.getDegrees());
            
            if (Robot.CHARACTERIZE_ROBOT){

                mDriveMotor.set(ControlMode.PercentOutput, kState.speedMetersPerSecond/Constants.MAX_SPEED_METERSperSECOND); 
            } else if(Constants.kS == 0 && Constants.kV == 0) {
                
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

    
    

