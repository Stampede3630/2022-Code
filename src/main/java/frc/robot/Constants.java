package frc.robot;



public class Constants{
    public static final double XBOXDEADBAND = .1;
    
    //Constants for conversion maths
    public static final double WHEEL_RADIUS_METERS = .0508;
    public static final double WHEEL_BASE_METERS = 9 * 2.54/100; //9 inch wheel base to meters
    public static final double MAX_SPEED_TICKSper100MS = 21900;
    public static final double SECONDSper100MS = .1;
    public static final double TICKSperREVOLUTION = 6.86*2048;//Gearing * 1 full rotation of Talon
    public static final double METERSperREVOLUTION = 2*Math.PI*WHEEL_RADIUS_METERS;
    public static final double MAX_SPEED_METERSperSECOND = MAX_SPEED_TICKSper100MS/SECONDSper100MS/TICKSperREVOLUTION*METERSperREVOLUTION;
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND/(WHEEL_BASE_METERS*Math.sqrt(2));
    public static final double SPEED_GOVERNOR =.15;
    
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

    //Swerve CANCoder offsets
    //CHANGE TO 0 first, reset the sensor, zero out the motor and place the OPPOSITE of the value
    public static double FRSensorOffset = -21.06+8.174;
    public static double FLSensorOffset = 52.119+2.725;
    public static double BRSensorOffset = 175.518+1.494;
    public static double BLSensorOffset = -34.453+10.371;

    //Swerve Steering PIDs (kP, kI, kD)
    public static Gains FRSteerGains = new Gains(1.2, 0, 0);
    public static Gains FLSteerGains = new Gains(1.2, 0, 0);
    public static Gains BRSteerGains = new Gains(1.2, 0, 0);
    public static Gains BLSteerGains = new Gains(1.2, 0, 0);

    //Swerve Driving PIDs (kP, kI, kD)
    public static Gains FRDriveGains = new Gains(0.025, 0, 0, 1023.0/20660.0);
    public static Gains FLDriveGains = new Gains(0.025, 0, 0, 1023.0/20660.0);
    public static Gains BRDriveGains = new Gains(0.025, 0, 0, 1023.0/20660.0);
    public static Gains BLDriveGains = new Gains(0.025, 0, 0, 1023.0/20660.0);

    //CTRE CAN-based constants
    public static final int kDefaultPIDSlotID = 0;
    public static final int kDefaultTimeout = 30;//milliseconds
    public static final int kDefaultClosedLoopError = 5;//sensor units

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