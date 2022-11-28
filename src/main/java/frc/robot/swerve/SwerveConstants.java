package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
public class SwerveConstants {
    public static final boolean OPTIMIZESTEERING = false;
    public static final boolean CHARACTERIZE_ROBOT = false;
    public static final boolean RUN_TRAJECTORY = true;

    //SWERVE MODULE CHARACTERISTICS
    // OG WHEEL_RADIUS_METERS = 0.10033/2
    // NEW WHEEL_RADIUS_METERS = 0.13000/2
    public static final double WHEEL_RADIUS_METERS = .10033/2;
    public static final double WHEEL_BASE_METERS = 24.125 * 2.54/100; //18 inch wheel base to meters track width is 24in and wheel base is 22.5 in
    public static final double MAX_SPEED_TICKSper100MS = 21900;
    public static final double STEERING_MOTOR_GEARING = 150.0/7.0; //12.8
    public static final double DRIVE_MOTOR_GEARING = 57.0/7.0; // 1/(14/50*25/19*15/45)
    public static final double SPEED_GOVERNOR = 1; //.11 is a good safe start. Unlock it to "1" when you're confident with the robot
    public static final double TRACK_WIDE = 24.685 * 2.54/100;
    
    //SWERVE Drive Default Values
    public static final double ROBOTHoldAngleKP = 15; //Start at .7 and see where you go from there
    public static final boolean DEFAULT_HOLD_ROBOT_ANGLE = false;
	public static final boolean DEFAULT_FIELD_RELATIVE_DRIVE = true;
	public static final double DEFAULT_HOLD_ROBOT_ANGLE_SETPOINT = 0; 

    //Swerve Drive Motor IDs
    public static final int FRDriveID = 2;
    public static final int FLDriveID = 13;
    public static final int BRDriveID = 11;
    public static final int BLDriveID = 12;

    //Swerve Steer Motor IDs
    public static final int FRSteerID = 5;
    public static final int FLSteerID = 1;
    public static final int BRSteerID = 3;
    public static final int BLSteerID = 4;

    //Swerve CANCoder Sensor IDs
    public static final int FRSensorID = 16;
    public static final int FLSensorID = 14;
    public static final int BRSensorID = 17;
    public static final int BLSensorID = 15;

    //Swerve CANCoder Sensort offsets
    //CHANGE TO 0 first, reset the sensor, 
    //PHYSICALLY zero out the motor 
    //place the OPPOSITE of the value
    public static double FRSensorOffset =41.748; //-327.129;
    public static double FLSensorOffset =-37.969;// -41.133;
    public static double BRSensorOffset =125.244;// 124.980;
    public static double BLSensorOffset =50.537;// -243.721;


    //Give a positive input on the joystick or phoenix tuner
    //Switch this if it goes opposite the desired direction
    //Because of the gearing the convention could be reversed (GUESS AND CHECK)
    public static TalonFXInvertType FRInvertType = TalonFXInvertType.CounterClockwise;
    public static TalonFXInvertType FLInvertType = TalonFXInvertType.CounterClockwise;
    public static TalonFXInvertType BRInvertType = TalonFXInvertType.CounterClockwise;
    public static TalonFXInvertType BLInvertType = TalonFXInvertType.CounterClockwise;

    //Swerve Steering PIDs (kP, kI, kD)
    public static Gains FRSteerGains = new Gains(25, 0, 0);
    public static Gains FLSteerGains = new Gains(25, 0, 0);
    public static Gains BRSteerGains = new Gains(25, 0, 0);
    public static Gains BLSteerGains = new Gains(25, 0, 0);

    //Swerve Driving PIDs (kP, kI, kD)
    //Once characterized the drive PIDs are meaningless
    public static Gains FRDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static Gains FLDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static Gains BRDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static Gains BLDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
    public static final double kS = 0.4148;
    public static final double kV = 2.55;
    public static final double kA = 3.4537;
    public static final double kP = 11.469;//previous value (for testing): 4.4561 -e

    //CTRE CAN-based constants (shouldn't need to change these)
    public static final int kDefaultPIDSlotID = 0;
    public static final int kDefaultClosedLoopError = 1; //degrees 

 
    //Constants for conversion maths (RARELY THESE SHOULD BE CHANGED)
    public static final double SECONDSper100MS = .1;
    public static final double TICKSperTALONFX_Rotation = 2048;
    public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING*TICKSperTALONFX_Rotation;
    public static final double METERSperWHEEL_REVOLUTION = 2*Math.PI*WHEEL_RADIUS_METERS;
    public static final double METERSperROBOT_REVOLUTION =  2*Math.PI*pythagoreanTheorem(TRACK_WIDE, WHEEL_BASE_METERS);
    public static final double MAX_SPEED_METERSperSECOND = MAX_SPEED_TICKSper100MS/SECONDSper100MS/DRIVE_MOTOR_TICKSperREVOLUTION*METERSperWHEEL_REVOLUTION;
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND/METERSperROBOT_REVOLUTION*(2*Math.PI);
    public static final double TICKSperTALONFX_STEERING_DEGREE = TICKSperTALONFX_Rotation*STEERING_MOTOR_GEARING/360;
    
    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final double kIzone;
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

    private static double pythagoreanTheorem(double side1, double side2) {
        double radius = Math.sqrt(Math.pow(side1, 2.0) + Math.pow(side2, 2.0));
        return radius;
    }
}
