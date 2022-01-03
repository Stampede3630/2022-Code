package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class Constants {
    public static final double XBOXDEADBAND = .1;
    
    //SWERVE MODULE CHARACTERISTICS
    public static final double WHEEL_RADIUS_METERS = .05138;
    public static final double WHEEL_BASE_METERS = 18 * 2.54/100; //18 inch wheel base to meters
    public static final double MAX_SPEED_TICKSper100MS = 21900;
    public static final double STEERING_MOTOR_GEARING = 12.8;
    public static final double DRIVE_MOTOR_GEARING = 6.86;
    public static final double SPEED_GOVERNOR =.11; //.11 is a good safe start. Unlock it to "1" when you're confident with the robot
    
    
    //SWERVE Drive Default Values
    public static final double ROBOTHoldAngleKP = 7; //Start at .7 and see where you go from there
    public static final boolean DEFAULT_HOLD_ROBOT_ANGLE = true;
	public static final boolean DEFAULT_FIELD_RELATIVE_DRIVE = true;
	public static final double DEFAULT_HOLD_ROBOT_ANGLE_SETPOINT = 0; 

    //Swerve Drive Motor IDs
    public static final int FRDriveID = 6;
    public static final int FLDriveID = 12;
    public static final int BRDriveID = 8;
    public static final int BLDriveID = 10;

    //Swerve Steer Motor IDs
    public static final int FRSteerID = 7;
    public static final int FLSteerID = 5;
    public static final int BRSteerID = 9;
    public static final int BLSteerID = 11;

    //Swerve CANCoder Sensor IDs
    public static final int FRSensorID = 3;
    public static final int FLSensorID = 4;
    public static final int BRSensorID = 2;
    public static final int BLSensorID = 1;

    //Swerve CANCoder Sensort offsets
    //CHANGE TO 0 first, reset the sensor, 
    //PHYSICALLY zero out the motor 
    //place the OPPOSITE of the value
    public static double FRSensorOffset = -3.955;
    public static double FLSensorOffset = 59.326;
    public static double BRSensorOffset = -176.045;
    public static double BLSensorOffset = 26.455;


    //Give a positive input on the joystick or phoenix tuner
    //Switch this if it goes opposite the desired direction
    //Because of the gearing the convention could be reversed (GUESS AND CHECK)
    public static TalonFXInvertType FRInvertType = TalonFXInvertType.Clockwise;
    public static TalonFXInvertType FLInvertType = TalonFXInvertType.CounterClockwise;
    public static TalonFXInvertType BRInvertType = TalonFXInvertType.Clockwise;
    public static TalonFXInvertType BLInvertType = TalonFXInvertType.CounterClockwise;

    //Swerve Steering PIDs (kP, kI, kD)
    public static Gains FRSteerGains = new Gains(8, 0, 0);
    public static Gains FLSteerGains = new Gains(10, 0, 0);
    public static Gains BRSteerGains = new Gains(8, 0, 0);
    public static Gains BLSteerGains = new Gains(8, 0, 0);

    //Swerve Driving PIDs (kP, kI, kD)
    //Once characterized the drive PIDs are meaningless
    public static Gains FRDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static Gains FLDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static Gains BRDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static Gains BLDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static final double kS = 0.60968;
    public static final double kV = 2.2691;
    public static final double kA = 0.19335;

    //CTRE CAN-based constants (shouldn't need to change these)
    public static final int kDefaultPIDSlotID = 0;
    public static final int kDefaultTimeout = 30;//milliseconds
    public static final int kDefaultClosedLoopError = 3; //degrees 
    
    //Constants for conversion maths (RARELY THESE SHOULD BE CHANGED)
    public static final double SECONDSper100MS = .1;
    public static final double STEERING_SENSOR_TICKSperROTATION = 4096;
    public static final double STEERING_SENSOR_DEGREESperTICKS = 360/STEERING_SENSOR_TICKSperROTATION;
    public static final double TICKSperTALONFX_Rotation = 2048;
    public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING*TICKSperTALONFX_Rotation;
    public static final double METERSperWHEEL_REVOLUTION = 2*Math.PI*WHEEL_RADIUS_METERS;
    public static final double METERSperROBOT_REVOLUTION =  2*Math.PI*(WHEEL_BASE_METERS/2*1.414213);//Math.sqrt(2)
    public static final double MAX_SPEED_METERSperSECOND = MAX_SPEED_TICKSper100MS/SECONDSper100MS/DRIVE_MOTOR_TICKSperREVOLUTION*METERSperWHEEL_REVOLUTION;
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND/METERSperROBOT_REVOLUTION*(2*Math.PI);
    public static final double TICKSperTALONFX_DEGREE = TICKSperTALONFX_Rotation*STEERING_MOTOR_GEARING/360;

    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final int kIzone;
        public final double kPeakOutput;
        /**
         * @param _kP
         * @param _kI
         * @param _kD
         */
        public Gains(double _kP, double _kI, double _kD){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = 0;
            kIzone = 0;
            kPeakOutput = 1;
        }
        public Gains(double _kP, double _kI, double _kD, double _kF){
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = 300;
            kPeakOutput = 1;
        }
    }
}